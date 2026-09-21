import asyncio
import threading
import math
import sys

from asyncua import Client

# ---------------- ROS2 ----------------
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

# Required to parse the Isaac Sim Action Graph TF trees
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# =========================================================
# Configuration
# =========================================================

# OPC UA endpoint of the Siemens S7-1500T PLC.
PLC_URL = "opc.tcp://111.111.111.10:4840"

# OPC UA namespace index used by TIA Portal DBs (usually 3 for S7-1500).
NS = 3

# Name of the data block that holds the KUKA KR210 data inside TIA Portal.
DB_TCP = "MAIN_KUKA_KR210_L150_L"


# ---------------------------------------------------------
# Isaac Sim Transform Tree Configuration
# Match these with your Isaac Stage names if you experience TF lookup errors.
# ---------------------------------------------------------
WORLD_FRAME = "World"   # Isaac Sim root frame (often "World" or "world")
TCP_FRAME   = "tool0"   # Isaac Sim end-effector frame (often "tool0" or "link_6")


# ---------------------------------------------------------
# Arrays to read from the PLC.
# ---------------------------------------------------------
PLC_ARRAYS = {
    "MainRobotLeft_ActPos":    f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActPos"',
    "MainRobotLeft_ActAng":    f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActAng"',
    "MainRobotLeft_Pos4Maint": f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Pos4Maint"',
}


# ---------------------------------------------------------
# Boolean flags to read from the PLC.
# ---------------------------------------------------------
PLC_FLAGS = {
    "MainRobotLeft_Go2Pos4Maint": f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Go2Pos4Maint"',
    "MainRobotLeft_Group_Fault":  f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Group_Fault"',
    "MainRobotLeft_Axis1_Fault":  f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Axis1_Fault"',
    "MainRobotLeft_Axis2_Fault":  f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Axis2_Fault"',
    "MainRobotLeft_Axis3_Fault":  f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Axis3_Fault"',
    "MainRobotLeft_Axis4_Fault":  f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Axis4_Fault"',
}


# =========================================================
# $MAMES — Mastering offsets configuration for 4-Axis Profile
# =========================================================
# تم تحديث المراجع الميكانيكية لتطابق الصفر الافتراضي لوضعية الـ Home
MAMES_DEG = {
    "A1":   0.0,
    "A2":  -90.0,   
    "A3":   90.0,   
    "A4":   0.0,
}

ISAAC_HOME_OFFSET = {
    "A1": 0.0,
    "A2": 0.0,
    "A3": 0.0,
    "A4": 0.0,
}


# ---------------------------------------------------------
# التعديل الجوهري: ربط المحور الرابع للـ PLC بالمحور الخامس للمحاكي مباشر
# ---------------------------------------------------------
PLC_TO_ROS = {
    "A1": "joint_a1",
    "A2": "joint_a2",
    "A3": "joint_a3",
    "A4": "joint_a5",  # ربط الـ PLC A4 بالـ ROS joint_a5
}
ROS_TO_PLC = {v: k for k, v in PLC_TO_ROS.items()}


# =========================================================
# Reference frame conversions
# =========================================================

def kuka_math_to_mech(q_math_deg: dict) -> dict:
    """Convert KUKA mathematical angles to mechanical angles using $MAMES offsets."""
    return {
        ax: q_math_deg.get(ax, 0.0) - MAMES_DEG.get(ax, 0.0)
        for ax in q_math_deg
    }


def isaac_raw_to_mech(q_ros_deg: dict) -> dict:
    """Convert raw Isaac Sim ROS joint angles to mechanical PLC angles."""
    out = {}
    for ax_plc, ax_ros in PLC_TO_ROS.items():
        if ax_ros in q_ros_deg:
            out[ax_plc] = q_ros_deg[ax_ros] - ISAAC_HOME_OFFSET.get(ax_plc, 0.0)
    return out


def quat_to_kuka_a(w, x, y, z):
    """Extract KUKA A (yaw) angle in degrees from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


# =========================================================
# ROS2 subscriber & TF listener node
# =========================================================

class JointStateListener(Node):
    def __init__(self, holder: dict):
        super().__init__("joint_state_listener")
        self.holder = holder
        self.create_subscription(JointState, "/joint_states", self.cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.lookup_tf)

        self.get_logger().info("Subscribed to /joint_states and active tracking /tf tree.")

    def cb(self, msg: JointState):
        self.holder["msg"] = msg

    def lookup_tf(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(WORLD_FRAME, TCP_FRAME, now)

            x_mm = trans.transform.translation.x * 1000.0
            y_mm = trans.transform.translation.y * 1000.0
            z_mm = trans.transform.translation.z * 1000.0

            q = trans.transform.rotation
            a_deg = quat_to_kuka_a(q.w, q.x, q.y, q.z)

            self.holder["tcp_wcs"] = [x_mm, y_mm, z_mm, a_deg]
            self.holder["tf_error"] = None
        except TransformException as ex:
            self.holder["tf_error"] = str(ex)


def run_ros2(holder: dict, stop_event: threading.Event):
    """Run the ROS2 spin loop in a separate thread."""
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = JointStateListener(holder)

        while not stop_event.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except (ExternalShutdownException, KeyboardInterrupt):
                break
    except Exception as e:
        print(f"[ROS2 Thread Error] {e}", file=sys.stderr)
    finally:
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


# =========================================================
# Formatting helpers
# =========================================================

def fmt(v, nd=3):
    if v is None:
        return "None"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def arr4(vals):
    """Normalize any list/tuple to exactly 4 elements (None padded)."""
    if not isinstance(vals, (list, tuple)):
        return [None, None, None, None]
    return list(vals) + [None] * (4 - len(vals))


# =========================================================
# OPC UA bridge loop (main asyncio coroutine)
# =========================================================

async def opcua_loop(holder: dict, stop_event: threading.Event):
    print(f"Connecting to PLC via OPC UA at {PLC_URL} ...")

    try:
        async with Client(url=PLC_URL) as client:
            print("OPC UA connected successfully.\n")

            mames_str = ", ".join(f"{k}={v:+.1f}" for k, v in MAMES_DEG.items())
            home_str  = ", ".join(f"{k}={v:+.3f}" for k, v in ISAAC_HOME_OFFSET.items())
            print(f"KUKA $MAMES (deg): {mames_str}")
            print(f"Isaac Home offset (deg): {home_str}\n")

            arr_nodes  = {k: client.get_node(v) for k, v in PLC_ARRAYS.items()}
            flag_nodes = {k: client.get_node(v) for k, v in PLC_FLAGS.items()}

            while not stop_event.is_set():

                # --- 1) Read arrays ---
                try:
                    arr_vals = await client.read_values(list(arr_nodes.values()))
                    arrays = dict(zip(arr_nodes.keys(), arr_vals))
                except Exception as e:
                    arrays = {"error": str(e)}

                # --- 2) Read flags ---
                try:
                    flag_vals = await client.read_values(list(flag_nodes.values()))
                    flags = dict(zip(flag_nodes.keys(), flag_vals))
                except Exception as e:
                    flags = {"error": str(e)}

                # --- 3) Extract arrays ---
                act_pos = arr4(arrays.get("MainRobotLeft_ActPos"))     # [X, Y, Z, A]
                act_ang = arr4(arrays.get("MainRobotLeft_ActAng"))     # [A1, A2, A3, A4]
                pos4    = arr4(arrays.get("MainRobotLeft_Pos4Maint"))

                # --- 4) PLC values frame conversion ---
                plc_math = {f"A{i+1}": act_ang[i] for i in range(len(act_ang)) if act_ang[i] is not None}
                plc_mech = kuka_math_to_mech(plc_math)

                # --- 5) ROS values frame conversion ---
                ros_msg  = holder.get("msg")
                ros_rad  = dict(zip(ros_msg.name, ros_msg.position)) if ros_msg else {}
                ros_deg  = {k: math.degrees(v) for k, v in ros_rad.items()}
                ros_mech = isaac_raw_to_mech(ros_deg)

                # --- 6) Print current snapshot ---
                print("=" * 96)
                print(f"PLC ActPos [mm/deg] : X={fmt(act_pos[0])}  Y={fmt(act_pos[1])}  Z={fmt(act_pos[2])}  A={fmt(act_pos[3])}")

                act_ang_str = ", ".join(f"A{i+1}={fmt(act_ang[i])}" for i in range(len(act_ang)))
                print(f"PLC ActAng (math)   : {act_ang_str}")
                print(f"PLC Pos4Maint       : {[fmt(x) for x in pos4]}")
                print(f"PLC Flags           : {flags}")

                # --- 7) Comparison table ---
                if ros_deg:
                    print("-" * 96)
                    hdr = (
                        f"{'Axis/Coord':<12}{'PLC math':>12}{'PLC mech':>12}"
                        f"{'ROS raw':>12}{'ROS mech':>12}{'Δ(PLC-ROS)':>14}"
                    )
                    print(hdr)

                    for ax_plc, ax_ros in PLC_TO_ROS.items():
                        if ax_plc not in plc_math or ax_ros not in ros_deg:
                            continue

                        m_plc = plc_math[ax_plc]
                        k_plc = plc_mech.get(ax_plc)
                        m_ros = ros_deg[ax_ros]
                        k_ros = ros_mech.get(ax_plc)

                        d = (k_plc - k_ros) if (k_plc is not None and k_ros is not None) else None

                        k_plc_s = f"{k_plc:>12.3f}" if k_plc is not None else f"{'None':>12}"
                        k_ros_s = f"{k_ros:>12.3f}" if k_ros is not None else f"{'None':>12}"
                        d_s     = f"{d:>14.3f}"     if d     is not None else f"{'None':>14}"

                        print(f"{ax_plc:<12}{m_plc:>12.3f}{k_plc_s}{m_ros:>12.3f}{k_ros_s}{d_s}")
                else:
                    print("-" * 96)
                    print("ROS Joints          : (waiting for /joint_states...)")

                # --- 8) Cartesian TCP WCS Setup ---
                print("-" * 96)
                print(f"{'TCP WCS (Cartesian Position & Delta)':<96}")
                print("-" * 96)

                tf_err = holder.get("tf_error")
                if tf_err:
                    print(f" [TF Wait] Active Status Lookup State: {tf_err}")
                else:
                    isaac_tcp  = holder.get("tcp_wcs", [None, None, None, None])
                    tcp_labels = ["X (mm)", "Y (mm)", "Z (mm)", "A (deg)"]

                    for i in range(4):
                        p_val = act_pos[i]
                        i_val = isaac_tcp[i]
                        d_tcp = (p_val - i_val) if (p_val is not None and i_val is not None) else None

                        p_str = f"{p_val:.3f}" if isinstance(p_val, (int, float)) else "None"
                        i_str = f"{i_val:.3f}" if isinstance(i_val, (int, float)) else "None"
                        d_str = f"{d_tcp:.3f}" if isinstance(d_tcp, (int, float)) else "None"

                        print(f"{tcp_labels[i]:<12}{p_str:>12}{'-':>12}{i_str:>12}{'-':>12}{d_str:>14}")

                print("=" * 96)
                await asyncio.sleep(0.2)

    except Exception as e:
        print(f"\n[OPC UA Loop Fatal Failure]: {e}", file=sys.stderr)


# =========================================================
# Entry point
# =========================================================

async def main():
    holder = {
        "msg": None,
        "tcp_wcs": [None, None, None, None],
        "tf_error": "Initializing...",
    }
    stop_event = threading.Event()

    ros_thread = threading.Thread(target=run_ros2, args=(holder, stop_event), daemon=True)
    ros_thread.start()

    await asyncio.sleep(0.5)

    try:
        await opcua_loop(holder, stop_event)
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\nInterrupted by user.")
    finally:
        stop_event.set()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProcess terminated.")