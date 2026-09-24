# main.py
"""نقطة الدخول الرئيسية — تربط كل الوحدات معًا."""
import asyncio
import threading
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from config import (
    WORLD_FRAME, TCP_FRAME, TRACKED_FRAMES, FRAME_PREFIX, SHOW_RPY,
    PLC_TO_ROS, JOINT_SIGN, MAMES_DEG, APPLY_PLC_ALIGNMENT,
    ISAAC_TO_PLC_SIGN, ISAAC_TO_PLC_OFFSET,
)
from opcua_bridge import OpcUaBridge
from plc_view import extract_plc_data, print_plc_view
from isaac_view import build_isaac_data, print_isaac_view, quat_to_kuka_a, quat_to_euler_zyx
from comparator import print_full_comparison


def full_frame(name: str) -> str:
    return f"{FRAME_PREFIX}{name}" if FRAME_PREFIX else name


# =========================================================
# ROS 2 Listener (يعبئ holder بالبيانات الخام فقط)
# =========================================================
class JointStateListener(Node):
    def __init__(self, holder: dict):
        super().__init__("joint_state_listener")
        self.holder = holder
        self.create_subscription(JointState, "/joint_states", self.cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.lookup_tf)

        self.holder.setdefault("frames", {})
        self.holder.setdefault("tf_error", "Initializing...")
        for f in TRACKED_FRAMES:
            self.holder["frames"][f] = {"ok": False, "err": "not checked yet"}

        self.get_logger().info(
            f"Tracking TF frames: {', '.join(TRACKED_FRAMES)} "
            f"(root='{WORLD_FRAME}', tcp='{TCP_FRAME}')"
        )

    def cb(self, msg: JointState):
        self.holder["msg"] = msg

    def lookup_tf(self):
        # TCP
        try:
            trans = self.tf_buffer.lookup_transform(
                full_frame(WORLD_FRAME), full_frame(TCP_FRAME), rclpy.time.Time()
            )
            tr = trans.transform.translation
            q = trans.transform.rotation
            self.holder["tcp_wcs"] = [
                tr.x * 1000.0, tr.y * 1000.0, tr.z * 1000.0,
                quat_to_kuka_a(q.w, q.x, q.y, q.z),
            ]
            self.holder["tf_error"] = None
        except TransformException as ex:
            self.holder["tf_error"] = str(ex)

        # Frames
        for f in TRACKED_FRAMES:
            try:
                t = self.tf_buffer.lookup_transform(
                    full_frame(WORLD_FRAME), full_frame(f), rclpy.time.Time()
                )
                tr = t.transform.translation
                q = t.transform.rotation
                entry = {
                    "ok": True, "err": None,
                    "xyz_mm": (tr.x * 1000.0, tr.y * 1000.0, tr.z * 1000.0),
                    "quat": (q.x, q.y, q.z, q.w),
                    "a_deg": quat_to_kuka_a(q.w, q.x, q.y, q.z),
                }
                if SHOW_RPY:
                    entry["rpy"] = quat_to_euler_zyx(q.w, q.x, q.y, q.z)
                self.holder["frames"][f] = entry
            except TransformException as ex:
                self.holder["frames"][f] = {"ok": False, "err": str(ex)}


def run_ros2(holder: dict, stop_event: threading.Event):
    """حلقة ROS 2 في Thread منفصل."""
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
# Main Loop
# =========================================================
async def main_loop():
    holder = {
        "msg": None,
        "tcp_wcs": [None, None, None, None],
        "tf_error": "Initializing...",
        "frames": {f: {"ok": False, "err": "pending"} for f in TRACKED_FRAMES},
    }
    stop_event = threading.Event()

    # تشغيل ROS 2 في الخلفية
    ros_thread = threading.Thread(target=run_ros2, args=(holder, stop_event), daemon=True)
    ros_thread.start()
    await asyncio.sleep(0.5)

    # الاتصال بـ PLC
    bridge = OpcUaBridge()
    await bridge.connect()

    # طبع معلومات الإعدادات مرة واحدة
    print(f"Joint signs : {JOINT_SIGN}")
    print(f"$MAMES      : {MAMES_DEG}")
    print(f"Alignment   : {'ON' if APPLY_PLC_ALIGNMENT else 'OFF'}")
    print()

    try:
        while not stop_event.is_set():
            arrays, flags = await bridge.read_all()

            # 1) استخراج بيانات PLC
            plc_data = extract_plc_data(arrays)

            # 2) استخراج بيانات Isaac
            isaac_data = build_isaac_data(holder)

            # 3) عرض كل شيء (يمكنك تعطيل أي طباعة حسب الحاجة)
            print_plc_view(plc_data, flags)
            print_isaac_view(isaac_data)
            print_full_comparison(plc_data, isaac_data, flags)

            await asyncio.sleep(0.2)

    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\nInterrupted by user.")
    finally:
        stop_event.set()
        await bridge.close()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    try:
        asyncio.run(main_loop())
    except KeyboardInterrupt:
        print("\nProcess terminated.")