# config.py
"""جميع إعدادات المشروع في مكان واحد."""

import logging

# ---------- Logging ----------
LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
LOG_LEVEL = logging.INFO

def setup_logging():
    logging.basicConfig(format=LOG_FORMAT, level=LOG_LEVEL)

# ---------- OPC UA ----------
PLC_URL = "opc.tcp://192.168.100.50:4840"
NS = 3
DB_TCP = "MAIN_KUKA_KR210_L150_L"

PLC_ARRAYS = {
    "MainRobotLeft_ActPos":    f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActPos"',
    "MainRobotLeft_ActAng":    f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActAng"',
    "MainRobotLeft_Pos4Maint": f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Pos4Maint"',
}

PLC_FLAGS = {
    "MainRobotLeft_Go2Pos4Maint": f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Go2Pos4Maint"',
    "MainRobotLeft_Group_Fault":  f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Group_Fault"',
    "MainRobotLeft_Axis1_Fault":  f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Axis1_Fault"',
    "MainRobotLeft_Axis2_Fault":  f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Axis2_Fault"',
    "MainRobotLeft_Axis3_Fault":  f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Axis3_Fault"',
    "MainRobotLeft_Axis4_Fault":  f'ns={NS};s="{DB_TCP}"."MainRobotLeft_Axis4_Fault"',
}

PLC_COMMANDS = {
    "CmdAng":    f'ns={NS};s="{DB_TCP}"."MainRobotLeft_CmdAng"',
    "CmdEnable": f'ns={NS};s="{DB_TCP}"."MainRobotLeft_CmdEnable"',
    "CmdAck":    f'ns={NS};s="{DB_TCP}"."MainRobotLeft_CmdAck"',
}

# ---------- Isaac Sim TF ----------
WORLD_FRAME = "base_link"
TCP_FRAME = "tool"
TRACKED_FRAMES = ["base_link", "link_1", "link_2", "link_3", "link_4", "tool"]
FRAME_PREFIX = ""
SHOW_RPY = False

# ---------- Sign & Offset ----------
JOINT_SIGN = {"A1": +1.0, "A2": -1.0, "A3": -1.0, "A4": +1.0}
MAMES_DEG = {"A1": 0.0, "A2": 0.0, "A3": 0.0, "A4": 0.0}
ISAAC_HOME_OFFSET = {"A1": 0.0, "A2": 0.0, "A3": 0.0, "A4": 0.0}

PLC_TO_ROS = {"A1": "joint_1", "A2": "joint_2", "A3": "joint_3", "A4": "joint_4"}
ROS_TO_PLC = {v: k for k, v in PLC_TO_ROS.items()}

ISAAC_TO_PLC_SIGN   = {"X": +1.0, "Y": +1.0, "Z": +1.0, "A": +1.0}
ISAAC_TO_PLC_OFFSET = {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0}
APPLY_PLC_ALIGNMENT = True

# ---------- Control Loop ----------
CONTROL_HZ = 20
ROBOT_PATH = "/World/Main_Robot_L_URDF/base_link"
DOF_ORDER  = ["joint_1", "joint_2", "joint_3", "joint_5", "joint_4"]
PLC_AXES_ORDER = ["A1", "A2", "A3", "A4"]

# ---------- Safety ----------
ENABLE_CONTROL = True
ANGLE_LIMITS_DEG = {
    "A1": (-185.0, 185.0),
    "A2": (-188.0,  35.0),
    "A3": (-114.6, 143.2),
    "A4": (-350.0, 350.0),
}

# ---------- Logging Thresholds ----------
PRINT_PERIOD_SEC = 1.0