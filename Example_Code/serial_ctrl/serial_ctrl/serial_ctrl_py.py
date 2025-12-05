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

# --- SMOOTHNESS PARAMETERS ---
STEP = 0.005          # radians per update
UPDATE_INTERVAL = 0.05  # 50 ms between updates

class RoArmSerialController(Node):
    def __init__(self):
        super().__init__('roarm_serial_ctrl')

        # Current joint angles [L1, L2, L3, L4, L5]
        self.joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_pos = self.joint_pos.copy()

        # Gesture flags
        self.gestures = ['up', 'down', 'left', 'right', 'pick', 'release', 'normal']
        self.gesture_flags = {g: False for g in self.gestures}

        # Subscribe to each gesture topic
        for g in self.gestures:
            self.create_subscription(Bool, f'/{g}', self.make_callback(g), 10)

        # Timer for smooth motion updates
        self.timer = self.create_timer(UPDATE_INTERVAL, self.update_motion)

        self.get_logger().info("✅ RoArm serial controller ready (multi-joint /up fixed)")

    # -------------------------------
    # Gesture subscription callbacks
    # -------------------------------
    def make_callback(self, gesture_name):
        def callback(msg):
            self.gesture_flags[gesture_name] = msg.data
            if msg.data:
                self.set_target_positions(gesture_name)
                self.get_logger().info(f"➡️ Gesture '{gesture_name}' activated")
        return callback

    # -------------------------------
    # Define target joint positions
    # -------------------------------
    def set_target_positions(self, gesture):
        new_target = self.joint_pos.copy()

        if gesture == 'up':
            # All joints move to given positions
            new_target = [0.0, 1.047, 0.890, -1.328, 0.0]

        elif gesture == 'down':
            new_target = [0.0, -0.379, 0.890, -1.328, 0.0]

        elif gesture == 'left':
            new_target = [3.140, self.joint_pos[1], self.joint_pos[2],
                          self.joint_pos[3], self.joint_pos[4]]

        elif gesture == 'right':
            new_target = [-3.140, self.joint_pos[1], self.joint_pos[2],
                          self.joint_pos[3], self.joint_pos[4]]

        elif gesture == 'pick':
            new_target = [self.joint_pos[0], self.joint_pos[1], self.joint_pos[2],
                          self.joint_pos[3], -1.341]

        elif gesture == 'release':
            new_target = [self.joint_pos[0], self.joint_pos[1], self.joint_pos[2],
                          self.joint_pos[3], 1.341]

        elif gesture == 'normal':
            new_target = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.target_pos = new_target

    # -------------------------------
    # Smooth joint movement toward targets
    # -------------------------------
    def update_motion(self):
        active = [g for g, v in self.gesture_flags.items() if v]
        if not active:
            return

        target = self.target_pos
        moved = False

        for i in range(5):
            diff = target[i] - self.joint_pos[i]
            if abs(diff) > STEP:
                self.joint_pos[i] += STEP if diff > 0 else -STEP
                moved = True
            else:
                self.joint_pos[i] = target[i]

        if moved:
            self.send_to_serial()

    # -------------------------------
    # Send command to RoArm over serial
    # -------------------------------
    def send_to_serial(self):
        j = self.joint_pos
        join1 = self.posGet(j[0], -1, 1)
        join2 = self.posGet(j[1], -1, 3)
        join3 = self.posGet(j[2], -1, 1)
        join4 = self.posGet(j[3],  1, 1)
        join5 = self.posGet(j[4], -1, 1)

        data = json.dumps({
            'T': 3,
            'P1': join1, 'P2': join2, 'P3': join3, 'P4': join4, 'P5': join5,
            'S1': 0, 'S2': 0, 'S3': 0, 'S4': 0, 'S5': 0,
            'A1': 60, 'A2': 60, 'A3': 60, 'A4': 60, 'A5': 60
        })

        try:
            ser.write(data.encode())
            self.get_logger().info(f"Sent: {data}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # -------------------------------
    # Convert radians → servo position
    # -------------------------------
    def posGet(self, radInput, direcInput, multiInput):
        if radInput == 0:
            return 2047
        else:
            return int(2047 + (direcInput * radInput / 3.1415926 * 2048 * multiInput) + 0.5)


# -------------------------------
# MAIN
# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RoArmSerialController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutdown requested")
    finally:
        node.destroy_node()
        ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

