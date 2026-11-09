#!/usr/bin/env python3
"""Minimal pass-through controller for closed-loop latency measurement.

Subscribes to /plc/cmd_ang, copies the frame_id tag unchanged, and
publishes to /joint_states. The bridge writes act_ang back to the PLC
and computes cycle_ns = write_done_ns - t_rs.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PassThroughController(Node):
    def __init__(self):
        super().__init__("pass_through_controller")
        self.sub = self.create_subscription(
            JointState, "/plc/cmd_ang", self.on_cmd, 10)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.get_logger().info("pass_through_controller started")

    def on_cmd(self, msg):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id   # preserve hb;t_rs;t_rd
        out.name = msg.name
        out.position = msg.position
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PassThroughController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()