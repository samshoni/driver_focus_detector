#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque

try:
    import mediapipe as mp
except ImportError:
    raise RuntimeError("Run: python3 -m pip install mediapipe")

mp_face_mesh = mp.solutions.face_mesh

LEFT_EYE = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [263, 385, 387, 362, 380, 373]
MOUTH = [61, 291, 13, 14]


def euclidean(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


def ear(eye_pts):
    A = euclidean(eye_pts[1], eye_pts[5])
    B = euclidean(eye_pts[2], eye_pts[4])
    C = euclidean(eye_pts[0], eye_pts[3])
    return (A + B) / (2.0 * C + 1e-6)


def mar(mouth_pts):
    vertical = euclidean(mouth_pts[2], mouth_pts[3])
    horizontal = euclidean(mouth_pts[0], mouth_pts[1])
    return vertical / (horizontal + 1e-6)


# ----------- OpenCV window -----------
cv2.namedWindow("Driver Focus Detection", cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_EXPANDED)
cv2.resizeWindow("Driver Focus Detection", 960, 720)


class DriverFocusNode(Node):
    def __init__(self):
        super().__init__('driver_focus_node')
        self.pub = self.create_publisher(String, 'driver_state', 10)

        self.declare_parameter('camera_index', 0)
        self.cap = cv2.VideoCapture(self.get_parameter('camera_index').value)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera.")

        self.face_mesh = mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True)
        self.ear_buffer = deque(maxlen=6)
        self.mar_buffer = deque(maxlen=6)

        self.EAR_THRESH = 0.26
        self.MAR_THRESH = 0.65
        self.EAR_FRAMES = 10
        self.MAR_FRAMES = 15

        # Calibration
        self.auto_calibrate_on_start = True
        self.calib_samples = []
        self.calib_needed_frames = 60
        self.ear_ratio_factor = 0.78
        self.calib_done = False

        self.eye_counter = 0
        self.yawn_counter = 0
        self.last_state = "OK"

        self.timer = self.create_timer(1 / 30, self.update)
        self.get_logger().info("🚗 Driver Focus Detector – Professional Edition started")

    def gather_points(self, lm, ids, w, h):
        return [(lm[i].x * w, lm[i].y * h) for i in ids]

    def draw_slider(self, frame, x, y, w, h, ratio, label):
        # Draw slider background
        cv2.rectangle(frame, (x, y), (x + w, y + h), (40, 40, 40), -1)
        # Filled portion (eye openness)
        fill_w = int(w * np.clip(ratio, 0.0, 1.0))
        cv2.rectangle(frame, (x, y), (x + fill_w, y + h), (0, 255, 255), -1)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 180, 180), 1)
        cv2.putText(frame, label, (x, y - 5), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)

    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(rgb)

        state, alert_text, color = "OK", "", (0, 255, 0)

        if results.multi_face_landmarks:
            lm = results.multi_face_landmarks[0].landmark
            left_eye = self.gather_points(lm, LEFT_EYE, w, h)
            right_eye = self.gather_points(lm, RIGHT_EYE, w, h)
            mouth = self.gather_points(lm, MOUTH, w, h)

            left_ear = ear(left_eye)
            right_ear = ear(right_eye)
            avg_ear = (left_ear + right_ear) / 2.0
            avg_mar = mar(mouth)

            self.ear_buffer.append(avg_ear)
            self.mar_buffer.append(avg_mar)
            ear_smooth = np.mean(self.ear_buffer)
            mar_smooth = np.mean(self.mar_buffer)

            # ---------- Calibration ----------
            if self.auto_calibrate_on_start and not self.calib_done:
                if avg_mar < 0.55:
                    self.calib_samples.append(avg_ear)
                cv2.rectangle(frame, (0, 0), (w, 60), (0, 0, 0), -1)
                cv2.putText(frame,
                            f"Calibrating... Keep eyes open ({len(self.calib_samples)}/{self.calib_needed_frames})",
                            (20, 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 255), 2)
                if len(self.calib_samples) >= self.calib_needed_frames:
                    baseline = float(np.median(self.calib_samples))
                    self.EAR_THRESH = baseline * self.ear_ratio_factor
                    self.calib_done = True
                    self.get_logger().info(
                        f"Calibration done – EAR baseline {baseline:.3f} TH {self.EAR_THRESH:.3f}")
            else:
                # ---------- Detection ----------
                self.eye_counter = self.eye_counter + 1 if ear_smooth < self.EAR_THRESH else 0
                self.yawn_counter = self.yawn_counter + 1 if mar_smooth > self.MAR_THRESH else 0

                if self.eye_counter >= self.EAR_FRAMES:
                    state, alert_text, color = "DROWSY", "⚠️ Eyes Closed!", (0, 0, 255)
                elif self.yawn_counter >= self.MAR_FRAMES:
                    state, alert_text, color = "YAWN", "😮 Yawning Detected!", (0, 165, 255)

                # ---------- Visualization ----------
                cv2.polylines(frame, [np.array(left_eye, np.int32)], True, color, 2, cv2.LINE_AA)
                cv2.polylines(frame, [np.array(right_eye, np.int32)], True, color, 2, cv2.LINE_AA)
                cv2.line(frame, tuple(np.int32(mouth[2])), tuple(np.int32(mouth[3])), (0, 255, 255), 2)

                # ---------- HUD ----------
                hud_bg = frame.copy()
                cv2.rectangle(hud_bg, (10, 10), (290, 85), (0, 0, 0), -1)
                frame = cv2.addWeighted(hud_bg, 0.5, frame, 0.5, 0)
                cv2.putText(frame, f"EAR: {ear_smooth:.3f}  TH: {self.EAR_THRESH:.3f}",
                            (20, 35), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(frame, f"MAR: {mar_smooth:.3f}",
                            (20, 60), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 2)

                # ---------- Eye Openness Slider ----------
                ear_min, ear_max = 0.15, 0.35
                ratio = (ear_smooth - ear_min) / (ear_max - ear_min)
                self.draw_slider(frame, 20, 90, 240, 12, ratio, "Eye Openness")

                # ---------- Alert Banner ----------
                if alert_text:
                    banner = frame.copy()
                    cv2.rectangle(banner, (0, h - 100), (w, h), color, -1)
                    frame = cv2.addWeighted(banner, 0.4, frame, 0.6, 0)
                    cv2.putText(frame, alert_text, (60, h - 35),
                                cv2.FONT_HERSHEY_DUPLEX, 1.2, (255, 255, 255), 3)
        else:
            state = "NO_FACE"
            cv2.putText(frame, "No face detected", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # ---------- ROS Publish ----------
        if state != self.last_state:
            msg = String()
            msg.data = state
            self.pub.publish(msg)
            self.last_state = state
            self.get_logger().info(f"Driver state: {state}")

        # ---------- Display ----------
        cv2.imshow("Driver Focus Detection", frame)
        cv2.waitKey(10)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = DriverFocusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

