#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import json
import time
import math

# ---------------------------------------------------------
# SERIAL SETUP
# ---------------------------------------------------------
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0)
time.sleep(1)

# Update every 20ms (50Hz)
UPDATE_DT = 0.02

# Smooth movement tuning
MAX_SPEED = 0.04       # rad/sec per cycle
ACCEL = 0.015          # acceleration factor
DAMPING = 0.15         # prevents overshoot & shaking


# ---------------------------------------------------------
# Smooth Controller
# ---------------------------------------------------------
class SmoothArmController(Node):
    def __init__(self):
        super().__init__("smooth_serial_controller")

        # joints L1–L5
        self.j = [0.0, 0.0, 0.0, 0.0, 0.0]     # actual joint
        self.v = [0.0, 0.0, 0.0, 0.0, 0.0]     # velocity
        self.t = [0.0, 0.0, 0.0, 0.0, 0.0]     # target

        # Subscriptions
        self.create_subscription(Bool, "up",      self.cb_up, 10)
        self.create_subscription(Bool, "down",    self.cb_down, 10)
        self.create_subscription(Bool, "pick",    self.cb_pick, 10)
        self.create_subscription(Bool, "release", self.cb_release, 10)

        # 50Hz motion update timer
        self.timer = self.create_timer(UPDATE_DT, self.update_motion)

        self.get_logger().info("🟢 SMOOTH ARM CONTROLLER ACTIVE (50Hz)")


    # ---------------------------------------------------------
    # Callbacks → set targets
    # ---------------------------------------------------------
    def cb_up(self, msg):
        if msg.data:
            self.t = [
                self.j[0],
                1.047,   # L2
                0.890,   # L3
                -1.328,  # L4
                self.j[4],
            ]

    def cb_down(self, msg):
        if msg.data:
            self.t = [
                self.j[0],
                -0.250,
                -0.980,
                -1.328,
                self.j[4],
            ]

    def cb_pick(self, msg):
        if msg.data:
            self.t[4] = -1.341

    def cb_release(self, msg):
        if msg.data:
            self.t[4] =  1.341


    # ---------------------------------------------------------
    # SMOOTH MOTION ENGINE (acceleration + damping)
    # ---------------------------------------------------------
    def update_motion(self):
        moved = False

        for i in range(5):
            diff = self.t[i] - self.j[i]

            # Deadzone to stop small jitter
            if abs(diff) < 0.001:
                self.v[i] = 0
                continue

            # Accelerate toward target
            self.v[i] += ACCEL * math.copysign(1, diff)

            # Velocity limit
            if self.v[i] >  MAX_SPEED: self.v[i] =  MAX_SPEED
            if self.v[i] < -MAX_SPEED: self.v[i] = -MAX_SPEED

            # Damping (smooth stopping)
            self.v[i] *= (1 - DAMPING)

            # Apply motion
            self.j[i] += self.v[i]
            moved = True

        if moved:
            self.send_to_serial()


    # ---------------------------------------------------------
    # SERIAL OUTPUT PACKET
    # ---------------------------------------------------------
    def send_to_serial(self):
        P = [
            self.map_servo(self.j[0], -1, 1),
            self.map_servo(self.j[1], -1, 3),
            self.map_servo(self.j[2], -1, 1),
            self.map_servo(self.j[3],  1, 1),
            self.map_servo(self.j[4], -1, 1),
        ]

        pkt = json.dumps({
            "T": 3,
            "P1": P[0], "P2": P[1], "P3": P[2], "P4": P[3], "P5": P[4],
            "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0,
            "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60
        })

        try:
            ser.write(pkt.encode())
        except:
            pass


    # ---------------------------------------------------------
    # Convert radians → servo position
    # ---------------------------------------------------------
    def map_servo(self, rad, direction, multiplier):
        if rad == 0:
            return 2047
        return int(2047 + (direction * rad / math.pi * 2048 * multiplier))


# ---------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SmoothArmController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

