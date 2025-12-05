#!/usr/bin/env python3
import time
import numpy as np
import cv2
import threading
from collections import deque
import sys

# ---------------- ROS2 ----------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


# ==========================================================
# ROS2 Publisher Node
# ==========================================================
class GesturePublisher(Node):
    def __init__(self):
        super().__init__("pose_gesture_pub")

        # Gesture topics
        self.pub_up      = self.create_publisher(Bool, "up", 10)
        self.pub_down    = self.create_publisher(Bool, "down", 10)
        self.pub_pick    = self.create_publisher(Bool, "pick", 10)
        self.pub_release = self.create_publisher(Bool, "release", 10)

    def send_up(self, v):      self.pub_up.publish(Bool(data=v))
    def send_down(self, v):    self.pub_down.publish(Bool(data=v))
    def send_pick(self, v):    self.pub_pick.publish(Bool(data=v))
    def send_release(self, v): self.pub_release.publish(Bool(data=v))


# ==========================================================
# Skeleton edges for drawing
# ==========================================================
KEYPOINT_EDGES = [
    (0,1),(1,3),(0,2),(2,4),
    (5,7),(7,9),(6,8),(8,10),
    (5,6),(5,11),(6,12),(11,12),
    (11,13),(13,15),(12,14),(14,16)
]

def draw_pose(frame, kp, thr=0.25):
    h, w = frame.shape[:2]
    # draw keypoints
    for y, x, s in kp:
        if s < thr: continue
        cv2.circle(frame, (int(x*w), int(y*h)), 4, (0,255,0), -1)

    # draw bones
    for a, b in KEYPOINT_EDGES:
        y1, x1, s1 = kp[a]
        y2, x2, s2 = kp[b]
        if s1 < thr or s2 < thr: continue
        cv2.line(frame,
                 (int(x1*w), int(y1*h)),
                 (int(x2*w), int(y2*h)),
                 (0,255,255), 2)


# ==========================================================
# Camera Thread (FAST)
# ==========================================================
class CameraThread:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        self.ret = False
        self.frame = None
        self.stop_flag = False
        self.lock = threading.Lock()

        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while not self.stop_flag:
            ret, frame = self.cap.read()
            frame = cv2.flip(frame, 1)     # FIX: mirror
            with self.lock:
                self.ret = ret
                self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is None: return False, None
            return self.ret, self.frame.copy()

    def stop(self):
        self.stop_flag = True
        self.cap.release()


# ==========================================================
# Load TFLite Interpreter
# ==========================================================
try:
    import tensorflow as tf
    Interpreter = tf.lite.Interpreter
except:
    from tflite_runtime.interpreter import Interpreter


def preprocess(frame, w, h, dtype):
    img = cv2.resize(frame, (w, h))
    if dtype == np.float32:
        img = img.astype(np.float32) / 255.0
    else:
        img = img.astype(np.uint8)
    return np.expand_dims(img, 0)


# ==========================================================
# MAIN
# ==========================================================
def main():
    MODEL_PATH = "/home/surendar/Drone_pro/ros2_arm_ws/src/human_pose_det/human_pose_det/4.tflite"

    rclpy.init()
    node = GesturePublisher()

    interpreter = Interpreter(model_path=MODEL_PATH, num_threads=4)
    interpreter.allocate_tensors()

    input_details  = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    _, in_h, in_w, in_c = input_details[0]["shape"]
    in_dtype = input_details[0]["dtype"]

    cam = CameraThread(0)

    margin_y = 0.10    # ↑ slightly increased (better detection)
    fps_list = deque(maxlen=30)
    last_t = time.time()

    print("\n🤖 FINAL POSE CONTROL (Corrected: Left=Up/Down, Right=Pick/Release)\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            ret, frame = cam.read()
            if not ret:
                continue

            # === Pose Inference ===
            inp = preprocess(frame, in_w, in_h, in_dtype)
            interpreter.set_tensor(input_details[0]["index"], inp)
            interpreter.invoke()

            kp = interpreter.get_tensor(output_details[0]["index"]).reshape(-1, 3)

            draw_pose(frame, kp)

            # ==========================================================
            # LEFT HAND = UP/DOWN  (Corrected)
            # ==========================================================
            ls_y,_,ls_s = kp[5]   # left shoulder
            lw_y,_,lw_s = kp[9]   # left wrist

            up = False
            down = False

            if lw_s > 0.22 and ls_s > 0.22:
                if lw_y < ls_y - margin_y:  up = True
                elif lw_y > ls_y + margin_y: down = True

            # ==========================================================
            # RIGHT HAND = PICK/RELEASE  (Corrected)
            # ==========================================================
            rs_y,_,rs_s = kp[6]   # right shoulder
            rw_y,_,rw_s = kp[10]  # right wrist

            pick = False
            release = False

            if rw_s > 0.22 and rs_s > 0.22:
                if rw_y < rs_y - margin_y:  pick = True
                elif rw_y > rs_y + margin_y: release = True

            # === Publish ===
            node.send_up(up)
            node.send_down(down)
            node.send_pick(pick)
            node.send_release(release)

            # Console debug
            print(f"L_UP={up} L_DOWN={down} | R_PICK={pick} R_RELEASE={release}", end="\r")

            # === FPS display ===
            now = time.time()
            fps_list.append(1/(now-last_t))
            last_t = now

            cv2.putText(frame, f"FPS:{sum(fps_list)/len(fps_list):.1f}",
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            cv2.imshow("Pose Gesture Control", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        pass

    cam.stop()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

