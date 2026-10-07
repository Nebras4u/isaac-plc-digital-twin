# plc_ros_bridge4.py
"""
PLC <-> ROS 2 bridge. Runs in the ROS 2 container.

Terminal output (1 Hz):
    Actual joint stat: A1: X.X, A2: X.X, A3: X.X, A4: X.X
    [WCS]: X=..., Y=..., Z=..., R=...
    (WCS read from TIA Portal: MainRobotLeft_ActPos[1..4])

Note:
    joint_5 is a FREE axis. It is not commanded by this bridge.
    It only appears in ROS_ORDER so the JointState message stays
    compatible with the Isaac subscriber.
"""

import asyncio
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from asyncua import Client, ua

from config import (
    PLC_URL, NS, DB_TCP,
    JOINT_SIGN, MAMES_DEG, ISAAC_HOME_OFFSET,
)

# ---------- Node IDs ----------
NODE_CMD_ENABLE = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_CmdEnable"'
NODE_CMD_ACK    = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_CmdAck"'
NODE_ACT_ANG    = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActAng"'
NODE_ACT_POS    = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActPos"'
NODE_CMD_ANG    = NODE_ACT_ANG    # same as your original code

# ---------- Joint mapping ----------
PLC_TO_ROS = {"A1": "joint_1", "A2": "joint_2", "A3": "joint_3", "A4": "joint_4"}
ROS_ORDER  = ["joint_1", "joint_2", "joint_3", "joint_5", "joint_4"]
CONTROL_HZ = 20

PRINT_PERIOD_SEC = 1.0


def plc_deg_to_ros_rad(ax, deg):
    urdf_deg = (deg - MAMES_DEG.get(ax, 0.0) + ISAAC_HOME_OFFSET.get(ax, 0.0)) / JOINT_SIGN[ax]
    return math.radians(urdf_deg)


def ros_rad_to_plc_deg(ax, ros_rad):
    urdf_deg = math.degrees(ros_rad)
    return JOINT_SIGN[ax] * urdf_deg + MAMES_DEG.get(ax, 0.0) - ISAAC_HOME_OFFSET.get(ax, 0.0)


async def opc_write(node, value, variant_type):
    wv = ua.WriteValue()
    wv.NodeId = node.nodeid
    wv.AttributeId = ua.AttributeIds.Value
    wv.Value = ua.DataValue(ua.Variant(value, variant_type))
    wv.Value.StatusCode = None
    wv.Value.SourceTimestamp = None
    wv.Value.ServerTimestamp = None

    params = ua.WriteParameters()
    params.NodesToWrite = [wv]
    await node.write_params(params)


# =========================================================
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
            # joint_5: FREE — not commanded. Keep it at 0 in the message
            # so the JointState message stays well-formed.
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


# =========================================================
async def main_loop():
    print(f"Connecting to PLC at {PLC_URL} ...")
    client = Client(url=PLC_URL)
    await client.connect()
    print("PLC connected.")

    cmd_ang_node    = client.get_node(NODE_CMD_ANG)
    cmd_enable_node = client.get_node(NODE_CMD_ENABLE)
    cmd_ack_node    = client.get_node(NODE_CMD_ACK)
    act_ang_node    = client.get_node(NODE_ACT_ANG)
    act_pos_node    = client.get_node(NODE_ACT_POS)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    bridge = PlcRosBridge()
    stop_event = threading.Event()
    ros_thread = threading.Thread(
        target=run_ros, args=(bridge, stop_event), daemon=True
    )
    ros_thread.start()

    dt = 1.0 / CONTROL_HZ
    step = 0
    last_print_t = 0.0

    try:
        while not stop_event.is_set():
            try:
                enable = bool(await cmd_enable_node.read_value())
            except Exception:
                enable = False

            try:
                cmd = list(await cmd_ang_node.read_value())
            except Exception:
                cmd = [0.0, 0.0, 0.0, 0.0]

            try:
                act_pos = list(await act_pos_node.read_value())
            except Exception:
                act_pos = [0.0, 0.0, 0.0, 0.0]

            with bridge.lock:
                bridge.plc_enable = enable
                bridge.plc_cmd_angles_deg = cmd[:4]

            with bridge.lock:
                states = dict(bridge.latest_states)

            a_vals = [0.0, 0.0, 0.0, 0.0]
            for i, ax in enumerate(["A1", "A2", "A3", "A4"]):
                if PLC_TO_ROS[ax] in states:
                    a_vals[i] = ros_rad_to_plc_deg(ax, states[PLC_TO_ROS[ax]])

            try:
                await opc_write(act_ang_node, a_vals, ua.VariantType.Double)
            except Exception:
                pass

            try:
                await opc_write(cmd_ack_node, bool(enable), ua.VariantType.Boolean)
            except Exception:
                pass

            # ---- throttled operator print (1 Hz) ----
            now = time.monotonic()
            if now - last_print_t >= PRINT_PERIOD_SEC:
                last_print_t = now

                parts = [
                    f"{ax}: {a_vals[i]: .1f}"
                    for i, ax in enumerate(["A1", "A2", "A3", "A4"])
                ]
                print("Actual joint stat: " + ", ".join(parts))

                if len(act_pos) >= 3:
                    x, y, z = act_pos[0], act_pos[1], act_pos[2]
                    fourth = act_pos[3] if len(act_pos) > 3 else None
                    if fourth is not None:
                        print(f"[WCS]: X={x: .2f}, Y={y: .2f}, Z={z: .2f}, R={fourth: .2f}")
                    else:
                        print(f"[WCS]: X={x: .2f}, Y={y: .2f}, Z={z: .2f}")
                else:
                    print(f"[WCS]: {act_pos}")

            step += 1
            await asyncio.sleep(dt)

    except (KeyboardInterrupt, asyncio.CancelledError):
        print("Interrupted.")
    finally:
        stop_event.set()
        await client.disconnect()
        ros_thread.join(timeout=1.0)
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        asyncio.run(main_loop())
    except KeyboardInterrupt:
        print("Terminated.")