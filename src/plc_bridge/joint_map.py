# mapping.py
"""Joint mapping & unit conversion. منطق منقول حرفياً من plc_ros_bridge4.py"""

import math
import logging
from config import JOINT_SIGN, MAMES_DEG, ISAAC_HOME_OFFSET

log = logging.getLogger("bridge.mapping")

PLC_TO_ROS = {"A1": "joint_1", "A2": "joint_2", "A3": "joint_3", "A4": "joint_4"}
ROS_ORDER  = ["joint_1", "joint_2", "joint_3", "joint_5", "joint_4"]


def plc_deg_to_ros_rad(ax, deg):
    urdf_deg = (deg - MAMES_DEG.get(ax, 0.0) + ISAAC_HOME_OFFSET.get(ax, 0.0)) / JOINT_SIGN[ax]
    return math.radians(urdf_deg)


def ros_rad_to_plc_deg(ax, ros_rad):
    urdf_deg = math.degrees(ros_rad)
    return JOINT_SIGN[ax] * urdf_deg + MAMES_DEG.get(ax, 0.0) - ISAAC_HOME_OFFSET.get(ax, 0.0)