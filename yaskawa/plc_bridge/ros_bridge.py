# ros_bridge.py
"""ROS 2 node. منطق منقول حرفياً من plc_ros_bridge4.py"""

import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState

from config import CONTROL_HZ
from joint_map import plc_deg_to_ros_rad, ROS_ORDER


class PlcRosBridge(Node):
    def __init__(self):
        super().__init__("plc_ros_bridge")
        self.cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.latest_states = {}
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.lock = threading.Lock()
        self.plc_cmd_angles_deg = [0.0, 0.0, 0.0, 0.0]
        self.plc_enable = False
        self.create_timer(1.0 / CONTROL_HZ, self.publish_command)
        self.get_logger().info("PLC <-> ROS 2 bridge started.")

    def on_joint_states(self, msg):
        with self.lock:
            self.latest_states = dict(zip(msg.name, msg.position))

    def publish_command(self):
        with self.lock:
            enable = self.plc_enable
            cmd_deg = list(self.plc_cmd_angles_deg)
        if not enable:
            return

        ros_rad = {
            "joint_1": plc_deg_to_ros_rad("A1", cmd_deg[0]),
            "joint_2": plc_deg_to_ros_rad("A2", cmd_deg[1]),
            "joint_3": plc_deg_to_ros_rad("A3", cmd_deg[2]),
            "joint_4": plc_deg_to_ros_rad("A4", cmd_deg[3]),
            "joint_5": 0.0,
        }

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ROS_ORDER
        msg.position = [ros_rad[j] for j in ROS_ORDER]
        self.cmd_pub.publish(msg)


def run_ros(bridge, stop_event):
    try:
        while not stop_event.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(bridge, timeout_sec=0.1)
            except (ExternalShutdownException, KeyboardInterrupt):
                break
    except Exception as e:
        print(f"[ROS2 thread error] {e}")