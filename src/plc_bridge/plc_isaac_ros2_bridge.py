import asyncio
import threading
import math

from asyncua import Client

# ---------------- ROS2 ----------------
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


# =========================================================
# Configuration
# =========================================================
# OPC UA endpoint of the Siemens S7-1500T PLC.
PLC_URL = "opc.tcp://111.111.111.10:4840"

# OPC UA namespace index used by TIA Portal DBs (usually 3 for S7-1500).
NS      = 3

# Name of the data block that holds the KUKA KR210 data inside TIA Portal.
DB_TCP  = "MAIN_KUKA_KR210_L150_L"

# ---------------------------------------------------------
# Arrays to read from the PLC.
# Each entry: { friendly_key : full OPC UA NodeId }
# The friendly_key is what we later use in `arrays.get(...)`.
# ---------------------------------------------------------
PLC_ARRAYS = {
    "MainRobotLeft_ActPos":    f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActPos"',
    "MainRobotLeft_ActAng":    f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActAng"',
    "MainRobotLeft_Pos4Maint": f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Pos4Maint"',
}

# ---------------------------------------------------------
# Boolean flags to read from the PLC.
# Fault bits and command handshake bits.
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
# $MAMES — Mastering offsets read from $MACHINE.DAT (KUKA KRC)
# =========================================================
# $MAMES defines the angular difference between the KUKA
# mathematical zero (used inside the controller and PLC)
# and the mechanical zero (physical mastering position).
#
# Conversion rule:
#     Q_mechanical = Q_mathematical - $MAMES
#
# Values taken directly from $MACHINE.DAT lines 84-89.
MAMES_DEG = {
    "A1":   0.0,
    "A2": -90.0,   # A2 mathematical zero = +90 mechanical
    "A3":  90.0,   # A3 mathematical zero = -90 mechanical
    "A4":   0.0,
    "A5":   0.0,
    "A6":   0.0,
}

# ---------------------------------------------------------
# Isaac Sim home offset.
# Values read from /joint_states when Isaac is in its
# default "Home" pose. These remove the small URDF bias
# so that Isaac's mechanical zero matches KUKA's mechanical zero.
# ---------------------------------------------------------
ISAAC_HOME_OFFSET = {
    "A1":  0.802,
    "A2":  5.105,
    "A3": -5.094,
    "A4": -1.702,
    "A5":  0.0,
    "A6":  0.0,
}

# ---------------------------------------------------------
# Name mapping between PLC axes and ROS/Isaac joint names.
# ---------------------------------------------------------
PLC_TO_ROS = {
    "A1": "joint_a1",
    "A2": "joint_a2",
    "A3": "joint_a3",
    "A4": "joint_a4",
    "A5": "joint_a5",
    "A6": "joint_a6",
}
# Reverse map: joint_a1 -> A1, etc.
ROS_TO_PLC = {v: k for k, v in PLC_TO_ROS.items()}


# =========================================================
# Reference frame conversions
# =========================================================
def kuka_math_to_mech(q_math_deg: dict) -> dict:
    """
    Convert KUKA mathematical angles (as read from PLC ActAng)
    to mechanical angles (the same convention used by Isaac Sim).
        Q_mech = Q_math - $MAMES
    Input : dict {"A1": deg, "A2": deg, ...}
    Output: dict {"A1": deg, "A2": deg, ...} in mechanical frame.
    """
    return {
        ax: q_math_deg.get(ax, 0.0) - MAMES_DEG.get(ax, 0.0)
        for ax in q_math_deg
    }


def isaac_raw_to_mech(q_ros_deg: dict) -> dict:
    """
    Convert raw joint angles from Isaac Sim (/joint_states, in degrees)
    to the same mechanical frame used by kuka_math_to_mech.
    The Isaac "Home" pose is not exactly zero, so we subtract
    ISAAC_HOME_OFFSET to normalize.
        Q_mech = Q_isaac_raw - ISAAC_HOME_OFFSET
    """
    out = {}
    # Iterate over the mapping (PLC axis -> ROS joint name).
    for ax_plc, ax_ros in PLC_TO_ROS.items():
        # Only process joints that Isaac actually publishes.
        if ax_ros in q_ros_deg:
            out[ax_plc] = q_ros_deg[ax_ros] - ISAAC_HOME_OFFSET.get(ax_plc, 0.0)
    return out


# =========================================================
# ROS2 subscriber node
# =========================================================
class JointStateListener(Node):
    """
    Minimal ROS2 node that subscribes to /joint_states
    and stores the last received message in a shared dict.
    The shared dict is used by the asyncio loop to read the
    latest joint values without blocking.
    """

    def __init__(self, holder: dict):
        super().__init__("joint_state_listener")
        self.holder = holder
        self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.get_logger().info("Subscribed to /joint_states")

    def cb(self, msg: JointState):
        """Callback: store the latest JointState message."""
        self.holder["msg"] = msg


def run_ros2(holder: dict, stop_event: threading.Event):
    """
    Run the rclpy spin loop in a dedicated thread.
    This avoids conflicts between rclpy and asyncio in the main thread.
    Uses SignalHandlerOptions.NO so Ctrl+C goes to asyncio only.
    """
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = JointStateListener(holder)
    try:
        while not stop_event.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except (ExternalShutdownException, KeyboardInterrupt):
                break
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


# =========================================================
# Formatting helpers
# =========================================================
def fmt(v, nd=3):
    """
    Format any value for display.
    Floats -> fixed decimal notation with nd digits.
    Others -> str(v).
    """
    if v is None:
        return "None"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def arr4(vals):
    """
    Ensure we always have a list of 4 items.
    If the PLC returns fewer items (e.g. only 3 elements),
    pad with None so downstream code does not crash.
    """
    if not isinstance(vals, (list, tuple)):
        return [None, None, None, None]
    return list(vals) + [None] * (4 - len(vals))


# =========================================================
# OPC UA bridge loop (main asyncio coroutine)
# =========================================================
async def opcua_loop(holder: dict, stop_event: threading.Event):
    """
    Main bridge loop:
      - Connect to the PLC over OPC UA.
      - Every 200 ms:
          * Read ActPos / ActAng / Pos4Maint arrays.
          * Read fault and handshake flags.
          * Read the latest joint angles from ROS2.
          * Convert both to the mechanical reference frame.
          * Print a comparison table.
    """

    # ---- Connect to the PLC ----
    print(f"Connecting to PLC via OPC UA at {PLC_URL} ...")
    async with Client(url=PLC_URL) as client:
        print("OPC UA connected.\n")

        # Informational banner: show the mastering offsets in use.
        print("KUKA $MAMES (deg): "
              + ", ".join(f"{k}={v:+.1f}" for k, v in MAMES_DEG.items()))
        print("Isaac Home offset (deg): "
              + ", ".join(f"{k}={v:+.3f}" for k, v in ISAAC_HOME_OFFSET.items()))
        print()

        # Pre-resolve all OPC UA node objects once (faster than resolving each loop).
        arr_nodes  = {k: client.get_node(v) for k, v in PLC_ARRAYS.items()}
        flag_nodes = {k: client.get_node(v) for k, v in PLC_FLAGS.items()}

        # ---- Main loop ----
        while not stop_event.is_set():

            # --- 1) Read the three arrays (one network round-trip) ---
            try:
                arr_vals = await client.read_values(list(arr_nodes.values()))
                arrays = dict(zip(arr_nodes.keys(), arr_vals))
            except Exception as e:
                arrays = {"error": str(e)}

            # --- 2) Read all flags (one network round-trip) ---
            try:
                flag_vals = await client.read_values(list(flag_nodes.values()))
                flags = dict(zip(flag_nodes.keys(), flag_vals))
            except Exception as e:
                flags = {"error": str(e)}

            # --- 3) Extract arrays using the friendly keys ---
            act_pos = arr4(arrays.get("MainRobotLeft_ActPos"))    # [X, Y, Z, A]
            act_ang = arr4(arrays.get("MainRobotLeft_ActAng"))    # [A1, A2, A3, A4]
            pos4    = arr4(arrays.get("MainRobotLeft_Pos4Maint")) # target pose

            # --- 4) PLC values: mathematical -> mechanical ---
            # Build {"A1": deg, "A2": deg, ...} from the raw array.
            plc_math = {f"A{i+1}": act_ang[i] for i in range(len(act_ang))
                        if act_ang[i] is not None}
            plc_mech = kuka_math_to_mech(plc_math)

            # --- 5) ROS values: raw -> mechanical ---
            # holder["msg"] was set by the ROS2 thread.
            ros_msg  = holder.get("msg")
            # Build {"joint_a1": rad, "joint_a2": rad, ...}
            ros_rad  = dict(zip(ros_msg.name, ros_msg.position)) if ros_msg else {}
            # Convert to degrees for display and comparison.
            ros_deg  = {k: math.degrees(v) for k, v in ros_rad.items()}
            ros_mech = isaac_raw_to_mech(ros_deg)

            # --- 6) Print the current snapshot ---
            print("=" * 96)
            print(f"PLC ActPos [mm/deg] : X={fmt(act_pos[0])}  Y={fmt(act_pos[1])}  "
                  f"Z={fmt(act_pos[2])}  A={fmt(act_pos[3])}")
            print(f"PLC ActAng (math)   : "
                  + ", ".join(f"A{i+1}={fmt(act_ang[i])}" for i in range(len(act_ang))))
            print(f"PLC Pos4Maint       : {[fmt(x) for x in pos4]}")
            print(f"PLC Flags           : {flags}")

            # --- 7) Comparison table (only if ROS data is available) ---
            if ros_deg:
                print("-" * 96)
                hdr = (f"{'Axis':<5}{'PLC math':>12}{'PLC mech':>12}"
                       f"{'ROS raw':>12}{'ROS mech':>12}{'Δ(PLC-ROS)':>14}")
                print(hdr)

                # Iterate over the axis mapping and print one row per axis.
                for ax_plc, ax_ros in PLC_TO_ROS.items():
                    # Skip if the axis is missing on either side.
                    if ax_plc not in plc_math or ax_ros not in ros_deg:
                        continue
                    m_plc = plc_math[ax_plc]
                    k_plc = plc_mech.get(ax_plc)
                    m_ros = ros_deg[ax_ros]
                    k_ros = ros_mech.get(ax_plc)
                    # Delta is meaningful only in the mechanical frame.
                    d = (k_plc - k_ros) if (k_plc is not None and k_ros is not None) else None
                    print(f"{ax_plc:<5}{m_plc:>12.3f}{k_plc:>12.3f}"
                          f"{m_ros:>12.3f}{k_ros:>12.3f}{d:>14.3f}")
            else:
                print("ROS Joints          : (waiting...)")

            print("=" * 96)

            # Sleep before the next iteration (5 Hz refresh).
            await asyncio.sleep(0.2)


# =========================================================
# Entry point
# =========================================================
async def main():
    """
    Top-level coroutine:
      1. Start the ROS2 subscriber in a background thread.
      2. Run the OPC UA bridge loop in the main asyncio loop.
      3. On shutdown, signal the ROS thread and wait for it.
    """
    # Shared container between threads. Only "msg" is used.
    holder = {"msg": None}

    # Cooperative stop flag shared between main thread and ROS thread.
    stop_event = threading.Event()

    # Start ROS2 in its own thread (daemon so it dies with the process).
    ros_thread = threading.Thread(target=run_ros2,
                                  args=(holder, stop_event), daemon=True)
    ros_thread.start()

    # Give ROS2 a moment to receive the first /joint_states message.
    await asyncio.sleep(0.5)

    try:
        await opcua_loop(holder, stop_event)
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\nInterrupted.")
    finally:
        # Ask the ROS thread to stop and wait briefly for it to finish.
        stop_event.set()
        ros_thread.join(timeout=2.0)


if __name__ == "__main__":
    asyncio.run(main())