# plc_view.py
"""تحويل وعرض بيانات PLC (TIA Portal)."""
from config import PLC_TO_ROS


def arr4(vals):
    """توحيد أي قائمة إلى 4 عناصر بالضبط."""
    if not isinstance(vals, (list, tuple)):
        return [None, None, None, None]
    return list(vals) + [None] * (4 - len(vals))


def plc_math_to_mech(act_ang_math: dict) -> dict:
    """PLC ActAng أصلاً في الإطار الميكانيكي — لا تحويل."""
    return dict(act_ang_math)


def fmt(v, nd=3):
    return f"{v:.{nd}f}" if isinstance(v, (int, float)) else "None"


def extract_plc_data(arrays: dict) -> dict:
    """استخراج البيانات الخام من قاموس المصفوفات."""
    act_pos = arr4(arrays.get("MainRobotLeft_ActPos"))
    act_ang = arr4(arrays.get("MainRobotLeft_ActAng"))
    pos4    = arr4(arrays.get("MainRobotLeft_Pos4Maint"))

    # تحويل زوايا PLC من رياضية إلى ميكانيكية
    plc_math = {f"A{i+1}": act_ang[i] for i in range(len(act_ang)) if act_ang[i] is not None}
    plc_mech = plc_math_to_mech(plc_math)

    return {
        "act_pos":  act_pos,
        "act_ang":  act_ang,
        "pos4":     pos4,
        "plc_math": plc_math,
        "plc_mech": plc_mech,
    }


def print_plc_view(plc_data: dict, flags: dict):
    """طباعة عرض كامل لبيانات PLC."""
    act_pos = plc_data["act_pos"]
    act_ang = plc_data["act_ang"]
    pos4    = plc_data["pos4"]

    print("=" * 110)
    print("            [ PLC — TIA Portal View ]")
    print("=" * 110)

    print(f"ActPos [mm/deg] : X={fmt(act_pos[0])}  Y={fmt(act_pos[1])}  "
          f"Z={fmt(act_pos[2])}  A={fmt(act_pos[3])}")

    ang_str = ", ".join(f"A{i+1}={fmt(act_ang[i])}" for i in range(len(act_ang)))
    print(f"ActAng (math)   : {ang_str}")
    print(f"Pos4Maint       : {[fmt(x) for x in pos4]}")
    print(f"Flags           : {flags}")
    print("=" * 110)