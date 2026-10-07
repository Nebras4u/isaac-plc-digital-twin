#!/usr/bin/env python3
# check_accuracy.py — قارن setpoints مع actual
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class Checker(Node):
    def __init__(self):
        super().__init__("check_accuracy")
        self.sp = None
        self.js = None
        self.create_subscription(JointState, "/servo_setpoints", self.on_sp, 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_timer(1.0, self.report)

    def on_sp(self, msg): self.sp = msg
    def on_js(self, msg): self.js = msg

    def report(self):
        if not self.sp or not self.js:
            return
        sp = dict(zip(self.sp.name, self.sp.position))
        js = dict(zip(self.js.name, self.js.position))

        print("=" * 60)
        print(f"{'joint':<10} {'setpoint':>12} {'actual':>12} {'error':>12}")
        print("-" * 60)
        for name in self.sp.name:
            s = sp.get(name, 0.0)
            a = js.get(name, 0.0)
            e = a - s
            deg_s = math.degrees(s)
            deg_a = math.degrees(a)
            deg_e = math.degrees(e)
            marker = "  ← OK" if abs(e) < 0.01 else "  ← DEV!"
            print(f"{name:<10} {deg_s:>10.4f}° {deg_a:>10.4f}° {deg_e:>10.4f}°{marker}")
        print("=" * 60)


def main():
    rclpy.init()
    node = Checker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()