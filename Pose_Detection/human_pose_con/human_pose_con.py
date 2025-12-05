#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial, json, time

# -----------------------------------
# ULTRA-FAST SERIAL (no delay)
# -----------------------------------
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0)
time.sleep(1)

STEP = 0.02
DT = 0.03


# -----------------------------------
# FAST CONTROLLER
# -----------------------------------
class ArmController(Node):
    def __init__(self):
        super().__init__("pose_serial_fast")

        self.j = [0,0,0,0,0]
        self.t = [0,0,0,0,0]

        self.create_subscription(Bool,"/up",      self.cb_up,10)
        self.create_subscription(Bool,"/down",    self.cb_down,10)
        self.create_subscription(Bool,"/pick",    self.cb_pick,10)
        self.create_subscription(Bool,"/release", self.cb_release,10)

        self.timer = self.create_timer(DT, self.update)
        self.get_logger().info("⚡ FAST SERIAL CONTROLLER ONLINE")

    # -----------------------------------  
    # Callbacks  
    # -----------------------------------
    def cb_up(self,msg):
        if msg.data:
            self.t = [self.j[0], 1.047, 0.890, -1.328, self.j[4]]

    def cb_down(self,msg):
        if msg.data:
            self.t = [self.j[0], -0.250, -0.980, -1.328, self.j[4]]

    def cb_pick(self,msg):
        if msg.data:
            self.t[4] = -1.341

    def cb_release(self,msg):
        if msg.data:
            self.t[4] = 1.341

    # -----------------------------------  
    # Smooth motor control  
    # -----------------------------------
    def update(self):
        moved = False
        for i in range(5):
            d = self.t[i] - self.j[i]
            if abs(d) > STEP:
                self.j[i] += STEP if d > 0 else -STEP
                moved = True
            else:
                self.j[i] = self.t[i]

        if moved:
            self.send()

    # -----------------------------------
    # Fast serial packet
    # -----------------------------------
    def send(self):
        P = [
            self.map(self.j[0], -1,1),
            self.map(self.j[1], -1,3),
            self.map(self.j[2], -1,1),
            self.map(self.j[3],  1,1),
            self.map(self.j[4], -1,1)
        ]

        pkt = json.dumps({
            "T":3,"P1":P[0],"P2":P[1],"P3":P[2],
            "P4":P[3],"P5":P[4],
            "S1":0,"S2":0,"S3":0,"S4":0,"S5":0,
            "A1":60,"A2":60,"A3":60,"A4":60,"A5":60
        })

        try:
            ser.write(pkt.encode())
        except:
            pass

    def map(self,rad,dir,mul):
        if rad == 0: return 2047
        return int(2047 + (dir * rad / 3.1415926 * 2048 * mul))


# -----------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
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

