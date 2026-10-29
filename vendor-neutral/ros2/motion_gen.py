#!/usr/bin/env python3
"""Deterministic motion generator for latency testing.

Publishes /plc/cmd_ang with proper p; tags (5 parts).
Traj adds sp_pub (6), physics adds js_pub (7), bridge records.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time, math, sys

class MotionGen(Node):
    def __init__(self, duration_s):
        super().__init__('motion_gen')
        self.pub = self.create_publisher(JointState, '/plc/cmd_ang', 10)
        self.duration = duration_s

    def run(self):
        rate_hz = 20.0
        period = 1.0 / rate_hz
        t = 0.0
        start = time.monotonic()

        print(f"[motion_gen] running {self.duration}s at {rate_hz} Hz")
        cmd_count = 0
        while (time.monotonic() - start) < self.duration:
            now = time.monotonic_ns()
            tag = f"p;{now};{now};{now};{now}"

            msg = JointState()
            msg.header.frame_id = tag
            msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
            # Sinusoids around home: (-90, +90)
            # joint_1 ±30°, joint_2 -90±15°, joint_3 +90±15°, joint_4 ±20°
            msg.position = [
                30.0 * math.sin(2 * math.pi * 0.10 * t),
                -90.0 + 15.0 * math.sin(2 * math.pi * 0.15 * t),
                90.0 + 15.0 * math.sin(2 * math.pi * 0.15 * t + 1.0),
                20.0 * math.sin(2 * math.pi * 0.20 * t),
            ]
            self.pub.publish(msg)
            cmd_count += 1
            t += period

            next_t = start + cmd_count * period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)

        print(f"[motion_gen] done, {cmd_count} commands sent")

def main():
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 300
    rclpy.init()
    node = MotionGen(duration)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
