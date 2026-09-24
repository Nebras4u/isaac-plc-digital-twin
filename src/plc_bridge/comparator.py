# comparator.py
"""مقارنة بيانات PLC و Isaac — الجدول الأهم في التوأمة."""
from config import PLC_TO_ROS, TRACKED_FRAMES, WORLD_FRAME


def s(v, w=12, p=3):
    """تنسيق القيمة بعرض محدد، أو 'None'."""
    return f"{v:>{w}.{p}f}" if isinstance(v, (int, float)) else f"{'None':>{w}}"


def print_joint_comparison(plc_data: dict, isaac_data: dict):
    """مقارنة زوايا المفاصل."""
    print("-" * 110)
    print(f"{'Axis':<12}{'PLC mech':>12}{'ROS raw':>12}{'ROS mech':>12}{'d(PLC-ROS)':>14}")
    print("-" * 110)

    for ax_plc, ax_ros in PLC_TO_ROS.items():
        if ax_plc not in plc_data["plc_mech"] or ax_ros not in isaac_data["ros_deg"]:
            continue
        k_plc = plc_data["plc_mech"].get(ax_plc)
        m_ros = isaac_data["ros_deg"][ax_ros]
        k_ros = isaac_data["ros_mech"].get(ax_plc)
        d = (k_plc - k_ros) if (k_plc is not None and k_ros is not None) else None
        print(f"{ax_plc:<12}{s(k_plc)}{m_ros:>12.3f}{s(k_ros)}{s(d, 14)}")


def print_tcp_comparison(plc_data: dict, isaac_data: dict):
    """مقارنة TCP Cartesian."""
    print("-" * 110)
    print("TCP WCS Comparison (PLC vs Isaac)")
    print("-" * 110)

    tcp = isaac_data["tcp_aligned"]
    labels = ["X (mm)", "Y (mm)", "Z (mm)", "A (deg)"]

    for i in range(4):
        p_val = plc_data["act_pos"][i]
        i_val = tcp[i]
        d = (p_val - i_val) if (p_val is not None and i_val is not None) else None
        p_str = f"{p_val:.3f}" if isinstance(p_val, (int, float)) else "None"
        i_str = f"{i_val:.3f}" if isinstance(i_val, (int, float)) else "None"
        d_str = f"{d:.3f}" if isinstance(d, (int, float)) else "None"
        print(f"{labels[i]:<12}{p_str:>12}{'-':>12}{i_str:>12}{'-':>12}{d_str:>14}")


def print_wcs_table(plc_data: dict, isaac_data: dict):
    """جدول كامل لكل الـ Frames مقابل PLC."""
    print("-" * 110)
    print(f"WCS of all frames (relative to {WORLD_FRAME})")
    print("-" * 110)

    print(f"{'Frame':<10}{'X (mm)':>12}{'Y (mm)':>12}{'Z (mm)':>12}{'A (deg)':>10}"
          f"   |{'PLC X':>12}{'PLC Y':>12}{'PLC Z':>12}{'PLC A':>10}"
          f"   |{'dX':>10}{'dY':>10}{'dZ':>10}{'dA':>9}")
    print("-" * 110)

    plc_ref = plc_data["act_pos"]
    frames = isaac_data["frames"]

    for f in TRACKED_FRAMES:
        info = frames.get(f, {})
        if info.get("ok"):
            x, y, z = info["xyz_mm"]
            a_deg = info["a_deg"]

            dx = (x - plc_ref[0]) if plc_ref[0] is not None else None
            dy = (y - plc_ref[1]) if plc_ref[1] is not None else None
            dz = (z - plc_ref[2]) if plc_ref[2] is not None else None
            da = (a_deg - plc_ref[3]) if plc_ref[3] is not None else None

            print(f"{f:<10}{x:>12.3f}{y:>12.3f}{z:>12.3f}{a_deg:>10.3f}"
                  f"   |{s(plc_ref[0])}{s(plc_ref[1])}{s(plc_ref[2])}{s(plc_ref[3], 10)}"
                  f"   |{s(dx, 10)}{s(dy, 10)}{s(dz, 10)}{s(da, 9)}")
        else:
            err = (info.get("err") or "no data")[:70]
            print(f"{f:<10} FAIL  {err}")


def print_full_comparison(plc_data: dict, isaac_data: dict, flags: dict):
    """التقرير الكامل: PLC → Isaac → المقارنة."""
    print("=" * 110)
    print("            [ FULL TWIN COMPARISON ]")
    print("=" * 110)

    # مفاصل
    print_joint_comparison(plc_data, isaac_data)

    # TCP
    print_tcp_comparison(plc_data, isaac_data)

    # جميع الـ Frames
    print_wcs_table(plc_data, isaac_data)
    print("=" * 110)