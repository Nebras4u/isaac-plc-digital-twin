# plc_ros_bridge.py
# Bridges PLCSIM <-> ROS 2.
# Also implements the ROS side of the heartbeat protocol.

import json
import math
import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState


WINDOWS_IP   = "192.168.100.18"
WINDOWS_PORT = 5000
POLL_HZ      = 20.0

DB = "MAIN_KUKA_KR210_L150_L"

ROS_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4"]

JS_TO_AXIS = {
    "joint_1": 0,
    "joint_2": 1,
    "joint_3": 2,
    "joint_4": 3,
}


class PlcRosBridge(Node):
    def __init__(self):
        super().__init__("plc_ros_bridge")

        # ---- Publishers ----
        self.pub_velocity = self.create_publisher(Float64,    "/plc/velocity", 10)
        self.pub_enable   = self.create_publisher(Bool,       "/plc/axis_enable", 10)
        self.pub_estop    = self.create_publisher(Bool,       "/plc/emergency_stop", 10)
        self.pub_cmd_ang  = self.create_publisher(JointState, "/plc/cmd_ang", 10)
        self.pub_alive    = self.create_publisher(Bool,       "/plc/ros_alive", 10)
        self.pub_cmd_ack  = self.create_publisher(Bool,       "/plc/cmd_ack", 10)

        # ---- Feedback state ----
        self._js_lock = threading.Lock()
        self._js_rad  = [0.0, 0.0, 0.0, 0.0]
        self._js_ok   = False

        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)

        # ---- TCP ----
        self.sock = None
        self.sock_lock = threading.Lock()
        self.connect_to_windows()

        # ---- Counters ----
        self._last_ack = -1

        # ---- Timer ----
        self.create_timer(1.0 / POLL_HZ, self.poll_tags)
        self.get_logger().info("PLC <-> ROS 2 bridge started.")

    # ---------------------------------------------------------------------
    def connect_to_windows(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1.0)
            self.sock.connect((WINDOWS_IP, WINDOWS_PORT))
            self.sock.settimeout(None)
            self.get_logger().info(f"Connected to {WINDOWS_IP}:{WINDOWS_PORT}")
        except Exception as e:
            self.get_logger().error(f"TCP connect failed: {e}")
            self.sock = None

    def send_request(self, req):
        if self.sock is None:
            self.connect_to_windows()
            if self.sock is None:
                return None
        try:
            with self.sock_lock:
                self.sock.sendall((json.dumps(req) + "\n").encode())
                buf = b""
                while b"\n" not in buf:
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        raise ConnectionError("server closed")
                    buf += chunk
                line, _ = buf.split(b"\n", 1)
                return json.loads(line.decode("utf-8"))
        except Exception as e:
            self.get_logger().warn(f"TCP error: {e}")
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            return None

    # ---------------------------------------------------------------------
    def on_joint_states(self, msg: JointState):
        with self._js_lock:
            for name, pos in zip(msg.name, msg.position):
                idx = JS_TO_AXIS.get(name)
                if idx is not None:
                    self._js_rad[idx] = float(pos)
            self._js_ok = True

    # ---------------------------------------------------------------------
    def poll_tags(self):
        # ---- 1. اقرأ من PLCSIM (بما فيها Heartbeat_Counter) ----
        read_names = [
            f"{DB}.MainRobotLeft_CmdAng[1]",
            f"{DB}.MainRobotLeft_CmdAng[2]",
            f"{DB}.MainRobotLeft_CmdAng[3]",
            f"{DB}.MainRobotLeft_CmdAng[4]",
            f"{DB}.MainRobotLeft_CmdEnable",
            f"{DB}.MainRobotLeft_Vlcty",
            f"{DB}.MainRobotLeft_Emergency_Stop_Active",
            f"{DB}.Heartbeat_Counter",
            f"{DB}.Comm_State",
            f"{DB}.ROS_Alive",
        ]
        resp = self.send_request({"cmd": "read", "tags": read_names})
        if not resp or "values" not in resp:
            return

        v = resp["values"]
        now = self.get_clock().now().to_msg()

        # ---- 2. Ack heartbeat: اكتب ROS_Ack_Counter = Heartbeat_Counter ----
        hb = v.get(f"{DB}.Heartbeat_Counter")
        if isinstance(hb, (int, float)) and int(hb) != self._last_ack:
            self.send_request({
                "cmd": "write",
                "tags": {f"{DB}.ROS_Ack_Counter": int(hb)}
            })
            self._last_ack = int(hb)

        # ---- 3. انشر الحالة ----
        m = Float64(); m.data = float(v.get(f"{DB}.MainRobotLeft_Vlcty", 0.0))
        self.pub_velocity.publish(m)

        m = Bool(); m.data = bool(v.get(f"{DB}.MainRobotLeft_CmdEnable", False))
        self.pub_enable.publish(m)

        m = Bool(); m.data = bool(v.get(f"{DB}.MainRobotLeft_Emergency_Stop_Active", False))
        self.pub_estop.publish(m)

        m = Bool(); m.data = bool(v.get(f"{DB}.ROS_Alive", False))
        self.pub_alive.publish(m)

        # ---- 4. CmdAng (JointState, degrees) ----
        cmd = JointState()
        cmd.header.stamp = now
        cmd.name = list(ROS_JOINTS)
        cmd.position = [
            float(v.get(f"{DB}.MainRobotLeft_CmdAng[1]", 0.0)),
            float(v.get(f"{DB}.MainRobotLeft_CmdAng[2]", 0.0)),
            float(v.get(f"{DB}.MainRobotLeft_CmdAng[3]", 0.0)),
            float(v.get(f"{DB}.MainRobotLeft_CmdAng[4]", 0.0)),
        ]
        self.pub_cmd_ang.publish(cmd)

        # ---- 5. اكتب ActAng من /joint_states ----
        with self._js_lock:
            if self._js_ok:
                self.send_request({
                    "cmd": "write",
                    "tags": {
                        f"{DB}.MainRobotLeft_ActAng[1]": math.degrees(self._js_rad[0]),
                        f"{DB}.MainRobotLeft_ActAng[2]": math.degrees(self._js_rad[1]),
                        f"{DB}.MainRobotLeft_ActAng[3]": math.degrees(self._js_rad[2]),
                        f"{DB}.MainRobotLeft_ActAng[4]": math.degrees(self._js_rad[3]),
                        f"{DB}.MainRobotLeft_CmdAck": True,
                    }
                })


def main():
    rclpy.init()
    node = PlcRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()