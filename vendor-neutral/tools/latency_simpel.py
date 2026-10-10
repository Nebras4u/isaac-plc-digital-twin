import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState

class Latency(Node):
    def __init__(self):
        super().__init__('latency')
        self.t_cmd = None
        self.samples = []

        # when a new cmd_ang arrives, mark time
        self.create_subscription(JointState, '/plc/cmd_ang', self.on_cmd, 10)
        # when servo_setpoints arrives right after, measure delta
        self.create_subscription(JointState, '/servo_setpoints', self.on_servo, 10)

        self.create_timer(2.0, self.report)

    def on_cmd(self, msg):
        self.t_cmd = time.monotonic()

    def on_servo(self, msg):
        if self.t_cmd is not None:
            dt = (time.monotonic() - self.t_cmd) * 1000.0
            if 0 < dt < 200:
                self.samples.append(dt)
            self.t_cmd = None

    def report(self):
        if self.samples:
            n = len(self.samples)
            avg = sum(self.samples) / n
            lo = min(self.samples)
            hi = max(self.samples)
            print(f"latency: avg={avg:.1f}ms  min={lo:.1f}  max={hi:.1f}  n={n}")
            self.samples.clear()

rclpy.init()
rclpy.spin(Latency())
#_______________________________________________________
""" 🎯 What these 11.5 ms consist of

PLC value changes
    ↓ (0-1 ms) OPC UA server detects change
Bridge reads /plc/cmd_ang
    ↓ (1-3 ms) OPC UA read round-trip
Bridge publishes /plc/cmd_ang (ROS 2)
    ↓ (1-3 ms) DDS delivery
krc_twin receives
    ↓ (0-2 ms) Ruckig computes next setpoint
krc_twin publishes /servo_setpoints
    ↓ (1-3 ms) DDS delivery
Isaac receives
    ↓ (0-1 ms) applies position target
Total: ~5-13 ms """