# main.py
"""Orchestrator. منطق منقول حرفياً من plc_ros_bridge4.py"""

import asyncio
import threading
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from asyncua import Client, ua

from config import PLC_URL, CONTROL_HZ, PRINT_PERIOD_SEC
from plc_client import (
    NODE_CMD_ANG, NODE_CMD_ENABLE, NODE_CMD_ACK,
    NODE_ACT_ANG, NODE_ACT_POS, opc_write,
)
from joint_map import PLC_TO_ROS, ros_rad_to_plc_deg
from ros_bridge import PlcRosBridge, run_ros


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