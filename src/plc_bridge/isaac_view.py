# isaac_view.py
"""تحويل وعرض بيانات Isaac Sim (ROS 2 / TF)."""
import math
from config import (
    JOINT_SIGN, MAMES_DEG, ISAAC_HOME_OFFSET, PLC_TO_ROS,
    ISAAC_TO_PLC_SIGN, ISAAC_TO_PLC_OFFSET, APPLY_PLC_ALIGNMENT,
    WORLD_FRAME, TRACKED_FRAMES, SHOW_RPY,
)


def isaac_raw_to_mech(q_ros_deg: dict) -> dict:
    """تحويل زوايا URDF إلى الإطار الميكانيكي.

    mechanical = JOINT_SIGN * URDF + $MAMES - HOME_OFFSET
    """
    out = {}
    for ax_plc, ax_ros in PLC_TO_ROS.items():
        if ax_ros in q_ros_deg:
            v = JOINT_SIGN[ax_plc] * q_ros_deg[ax_ros]
            v += MAMES_DEG.get(ax_plc, 0.0)
            v -= ISAAC_HOME_OFFSET.get(ax_plc, 0.0)
            out[ax_plc] = v
    return out


def quat_to_kuka_a(w, x, y, z):
    """استخراج زاوية A (yaw) من quaternion."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))


def quat_to_euler_zyx(w, x, y, z):
    """(roll, pitch, yaw) بالدرجات، اتفاقية ZYX."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))

    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.degrees(math.asin(sinp))

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))

    return roll, pitch, yaw


def isaac_tcp_to_plc(tcp_isaac):
    """Apply exact static offset transformation and handle angle wrapping."""
    keys = ["X", "Y", "Z", "A"]
    out = []
    for i, k in enumerate(keys):
        v = tcp_isaac[i]
        if v is None:
            out.append(None)
            continue
        out.append(ISAAC_TO_PLC_SIGN[k] * v + ISAAC_TO_PLC_OFFSET[k])
    
    # تهذيب التفاف الزاوية للحفاظ على استقرار الحسابات
    if out[3] is not None:
        if out[3] > 180.0:
            out[3] -= 360.0
        elif out[3] < -180.0:
            out[3] += 360.0
            
    return out


def build_isaac_data(holder: dict) -> dict:
    """استخراج كل بيانات Isaac من holder (قاموس مشترك)."""
    ros_msg = holder.get("msg")
    ros_rad = dict(zip(ros_msg.name, ros_msg.position)) if ros_msg else {}
    ros_deg = {k: math.degrees(v) for k, v in ros_rad.items()}
    ros_mech = isaac_raw_to_mech(ros_deg)

    tcp_raw = holder.get("tcp_wcs", [None, None, None, None])
    tcp_aligned = isaac_tcp_to_plc(tcp_raw) if APPLY_PLC_ALIGNMENT else tcp_raw

    return {
        "ros_deg":     ros_deg,
        "ros_mech":    ros_mech,
        "tcp_raw":     tcp_raw,
        "tcp_aligned": tcp_aligned,
        "tf_error":    holder.get("tf_error"),
        "frames":      holder.get("frames", {}),
    }


def print_isaac_view(isaac_data: dict):
    """طباعة عرض كامل لبيانات Isaac."""
    print("=" * 110)
    print("            [ Isaac Sim — ROS 2 View ]")
    print("=" * 110)

    # Joints
    if isaac_data["ros_deg"]:
        print(f"{'Joint':<12}{'Raw URDF':>14}{'Mechanical':>14}")
        print("-" * 110)
        for ax_plc, ax_ros in PLC_TO_ROS.items():
            if ax_ros not in isaac_data["ros_deg"]:
                continue
            raw = isaac_data["ros_deg"][ax_ros]
            mech = isaac_data["ros_mech"].get(ax_plc)
            mech_s = f"{mech:>14.3f}" if mech is not None else f"{'None':>14}"
            print(f"{ax_plc:<12}{raw:>14.3f}{mech_s}")
    else:
        print("(waiting for /joint_states...)")

    # TCP
    print("-" * 110)
    tcp = isaac_data["tcp_aligned"]
    labels = ["X (mm)", "Y (mm)", "Z (mm)", "A (deg)"]
    print(f"{'TCP (aligned to PLC frame)':<40}")
    for i, lbl in enumerate(labels):
        v = tcp[i]
        v_str = f"{v:.3f}" if isinstance(v, (int, float)) else "None"
        print(f"  {lbl:<12}{v_str:>12}")

    # Frames
    print("-" * 110)
    print(f"{'Frame':<10}{'X (mm)':>12}{'Y (mm)':>12}{'Z (mm)':>12}{'A (deg)':>12}")
    print("-" * 110)
    for f in TRACKED_FRAMES:
        info = isaac_data["frames"].get(f, {})
        if info.get("ok"):
            x, y, z = info["xyz_mm"]
            print(f"{f:<10}{x:>12.3f}{y:>12.3f}{z:>12.3f}{info['a_deg']:>12.3f}")
            if SHOW_RPY and "rpy" in info:
                r, p_, yw = info["rpy"]
                print(f"{'':<10}  RPY: R={r:.3f}  P={p_:.3f}  Y={yw:.3f}")
        else:
            err = (info.get("err") or "no data")[:70]
            print(f"{f:<10} FAIL  {err}")

    print("=" * 110)