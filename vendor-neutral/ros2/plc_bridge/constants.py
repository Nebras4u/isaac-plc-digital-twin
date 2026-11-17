from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float64, Int32, String

MSG_TYPES = {"bool": Bool, "int": Int32, "float": Float64, "string": String}
CAST = {"bool": bool, "int": int, "float": float, "string": str}

JS_STALE_AGE_S = 0.5
ACT_ANG_DEADBAND_DEG = 0.001
ACT_ANG_REFRESH_S = 0.5
_INT_TOL = 1e-9

PUB_STREAM_QOS = QoSProfile(depth=1)
SUB_STREAM_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

_SERVER_TYPE_MAP = {
    "Bool": "bool", "Boolean": "bool",
    "LReal": "float", "Double": "float", "Real": "float",
    "DInt": "int", "Int32": "int", "Int": "int", "Int16": "int",
}

try:
    from diagnostic_updater import Updater, FunctionDiagnosticTask
    from diagnostic_msgs.msg import DiagnosticStatus
    _HAS_DIAG = True
except Exception:
    _HAS_DIAG = False
    Updater = None
    FunctionDiagnosticTask = None
    DiagnosticStatus = None
