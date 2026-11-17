import threading
from collections import defaultdict

import numpy as np
import yaml
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from bridge_logger import setup_logging
from trajectory.callbacks import TrajectoryTwinCallbacks
from trajectory.limits import _validate_limits
from trajectory.node_state import IDLE, RUNNING, STOPPING
from trajectory.planners import PassthroughPlanner, build_planner
from trajectory.servo import TrajectoryTwinServo


class TrajectoryTwin(TrajectoryTwinCallbacks, TrajectoryTwinServo, Node):
    def __init__(self):
        super().__init__("trajectory_twin")
        self.declare_parameter("robot", "robot.yaml")
        with open(self.get_parameter("robot").value) as f:
            cfg = yaml.safe_load(f)["robot"]

        self.robot_name        = cfg["name"]
        self.all_joints        = cfg["all_joints"]
        self.controlled_joints = cfg["controlled_joints"]
        self.limits_deg        = cfg["limits_deg"]

        _validate_limits(self.limits_deg, self.controlled_joints)

        self.all_dof  = len(self.all_joints)
        self.ctrl_dof = len(self.controlled_joints)
        self.name_to_idx  = {n: i for i, n in enumerate(self.all_joints)}
        self.ctrl_indices = [self.name_to_idx[j] for j in self.controlled_joints]

        # v4.0: type-safety state
        self._warn_cooldown = float(cfg.get("warn_cooldown", 5.0))
        self._stats         = defaultdict(lambda: defaultdict(int))
        self._warned        = {}

        r = cfg["ruckig"]
        # Ruckig is the planner for normal operation. A pass-through planner
        # exists ONLY for benchmarking and is used ONLY while Test_Mode is
        # true AND robot.yaml sets ruckig.test_mode_passthrough: true.
        self.planner_ruckig = build_planner(r, self.ctrl_dof)
        pt = r.get("test_mode_passthrough", False)
        if not isinstance(pt, bool):
            raise ValueError(
                f"ruckig.test_mode_passthrough must be true or false, got {pt!r}")
        self.planner_pass = PassthroughPlanner(self.ctrl_dof) if pt else None
        self.planner = self.planner_ruckig
        self._test_mode = False     # unknown -> treated as normal (Ruckig)

        log_cfg = {"logging": cfg.get("logging", {})} if "logging" in cfg else {
            "logging": {"level": "INFO", "console": True, "to_ros": False,
                        "file": "/workspace/plc_bridge_logs/trajectory.log",
                        "json_file": "/workspace/plc_bridge_logs/trajectory.jsonl",
                        "max_bytes": 10 * 1024 * 1024, "backup_count": 5}
        }
        self.log = setup_logging(
            log_cfg, logger_name="trajectory_twin", ros_logger=self.get_logger()
        )

        stream_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.setpoint_pub = self.create_publisher(
            JointState, "/servo_setpoints", stream_qos)

        # ------------------------------------------------------------------
        # State (all guarded by self.lock unless noted)
        # ------------------------------------------------------------------
        self.lock = threading.Lock()

        self._actual       = np.zeros(self.all_dof)
        self._has_actual   = False
        self._last_js_t    = 0.0

        self._enable       = False
        self._estop        = True

        self._cmd_deg      = [0.0] * self.ctrl_dof
        self._last_received_cmd_deg = [None] * self.ctrl_dof
        self._last_applied_cmd_deg  = None

        self._cmd_ok       = False
        self._last_cmd_t   = 0.0
        self._state        = IDLE

        self._msg = JointState()
        self._msg.name     = list(self.all_joints)
        self._msg.position = [0.0] * self.all_dof
        self._msg.velocity = [0.0] * self.all_dof
        self._msg.header.frame_id = ""

        self._ctrl_actual = np.zeros(self.ctrl_dof)
        self._ctrl_target = np.zeros(self.ctrl_dof)

        self._last_setpoint_log  = 0.0
        self._cmd_timeout_logged = False
        self._no_feedback_logged = False
        self._bad_cmd_logged     = False
        self._cmd_tag            = ""
        self._cmd_tag_t          = 0.0

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        state_qos = QoSProfile(depth=1,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, "/joint_states",
                                 self.on_js, stream_qos)
        self.create_subscription(JointState, "/plc/cmd_ang",
                                 self.on_cmd, stream_qos)
        self.create_subscription(Bool, "/plc/axis_enable",
                                 self.on_enable, state_qos)
        self.create_subscription(Bool, "/plc/emergency_stop",
                                 self.on_estop, state_qos)
        self.create_subscription(Bool, "/plc/test_mode",
                                 self.on_test_mode, state_qos)

        self.create_timer(1.0 / r["control_hz"], self.servo_tick)

        self.log.info("twin_started", extra={
            "robot":   self.robot_name,
            "dof":     self.ctrl_dof,
            "rate_hz": r["control_hz"],
            "planner": type(self.planner).__name__,
        })

    def destroy_node(self):
        try:
            self.log.info("twin_shutdown")
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TrajectoryTwin()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
