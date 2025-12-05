#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import json
import serial
import time

# --- SERIAL CONFIG ---
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
time.sleep(2)

# --- SMOOTH MOVEMENT ---
STEP = 0.015
UPDATE_INTERVAL = 0.05  # 50 ms

class RoArmSerialController(Node):
    def __init__(self):
        super().__init__('roarm_serial_ctrl')

        # Current joint angles
        self.joint_pos  = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Target joint angles
        self.target_pos = self.joint_pos.copy()

        # One-time trigger system
        self.gestures = ['up','down','left','right','pick','release','normal']

        # Subscribe to all gesture topics
        for g in self.gestures:
            self.create_subscription(Bool, f'/{g}', self.make_callback(g), 10)

        # Timer for smooth movement
        self.timer = self.create_timer(UPDATE_INTERVAL, self.update_motion)

        self.get_logger().info("🔥 Serial Controller Ready (One command → Full movement)")

    # -------------------------
    # CALLBACK: trigger target only once
    # -------------------------
    def make_callback(self, gesture_name):
        def callback(msg):
            if msg.data:  # only on True
                self.set_target_positions(gesture_name)
                self.get_logger().info(f"🎯 Triggered: {gesture_name}")
        return callback

    # -------------------------
    # SET TARGET POSITIONS
    # -------------------------
    def set_target_positions(self, gesture):
        new_target = self.joint_pos.copy()

        if gesture == 'up':
            new_target[1] = 1.047
            new_target[2] = 0.890
            new_target[3] = -1.328

        elif gesture == 'down':
            new_target[1] = -0.250
            new_target[2] = -0.980
            new_target[3] = -1.328

        elif gesture == 'left':
            new_target[0] = 3.140

        elif gesture == 'right':
            new_target[0] = -3.140

        elif gesture == 'pick':
            new_target[4] = -1.341

        elif gesture == 'release':
            new_target[4] = 1.341

        elif gesture == 'normal':
            new_target = [0.0, 0.781, -1.450, -0.950, 0.0]

        self.target_pos = new_target

    # -------------------------
    # CONTINUOUS MOTION TOWARD TARGET
    # -------------------------
    def update_motion(self):

        moved = False
        target = self.target_pos

        for i in range(5):
            diff = target[i] - self.joint_pos[i]
            if abs(diff) > STEP:
                self.joint_pos[i] += STEP if diff > 0 else -STEP
                moved = True
            else:
                self.joint_pos[i] = target[i]

        if moved:
            self.send_to_serial()

    # -------------------------
    # SEND JSON PACKET
    # -------------------------
    def send_to_serial(self):
        j = self.joint_pos

        join1 = self.posGet(j[0], -1, 1)
        join2 = self.posGet(j[1], -1, 3)
        join3 = self.posGet(j[2], -1, 1)
        join4 = self.posGet(j[3],  1, 1)
        join5 = self.posGet(j[4], -1, 1)

        data = json.dumps({
            'T':3,
            'P1':join1,'P2':join2,'P3':join3,'P4':join4,'P5':join5,
            'S1':0,'S2':0,'S3':0,'S4':0,'S5':0,
            'A1':60,'A2':60,'A3':60,'A4':60,'A5':60
        })

        try:
            ser.write(data.encode())
            self.get_logger().info(f"➡️ {data}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial write failed: {e}")

    # -------------------------
    # SERVO POSITION MAP
    # -------------------------
    def posGet(self, radInput, direcInput, multiInput):
        if radInput == 0:
            return 2047
        return int(2047 + (direcInput * radInput / 3.1415926 * 2048 * multiInput) + 2.1)


# -------------------------
# MAIN
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RoArmSerialController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutdown")
    finally:
        node.destroy_node()
        ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

