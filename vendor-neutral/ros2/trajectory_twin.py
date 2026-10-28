# trajectory_twin.py
# Vendor-neutral Ruckig-based trajectory generator.
# Reads robot definition from robot.yaml.
#
# Fixes in this revision (v3.6):
#   * [A] Removed np.interp on the full joint vector (passive axes preserved).
#   * [B] needs_target_update compares against _last_applied_cmd_deg.
#   * [C] _last_applied_cmd_deg guarded under lock.
#   * [D] STOPPING→IDLE clears _last_applied_cmd_deg.
#   * [E] RuckigPlanner validates all params at construction.
#   * [F] _cmd_tag is NOT consumed by servo_tick anymore. It is only
#     cleared by _publish() AFTER the tag has actually been attached to
#     an outgoing /servo_setpoints message. Previously every tick cleared
#     it — so ticks that didn't publish (needs_target_update=False,
#     STOPPING without step, etc.) silently ate the tag, producing
#     /servo_setpoints messages with a 5-part frame_id instead of 6 and
#     breaking the latency-probe chain.

import math
import os
import threading
import time

import numpy as np
import yaml
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from ruckig import Ruckig, InputParameter, OutputParameter, Result, ControlInterface
from bridge_logger import setup_logging

IDLE, RUNNING, STOPPING = 0, 1, 2


def _validate_limits(limits_deg, joints):
    for j in joints:
        if j not in limits_deg:
            raise ValueError(f"robot.yaml: joint '{j}' missing limits_deg")
        lim = limits_deg[j]
        if not (isinstance(lim, (list, tuple)) and len(lim) == 2):
            raise ValueError(f"joint '{j}': limits_deg must be [lower, upper]")
        lo, hi = lim
        if lo >= hi:
            raise ValueError(f"joint '{j}': lower {lo}° >= upper {hi}°")
        if hi - lo > 720.0:
            raise ValueError(f"joint '{j}': range {hi - lo:.1f}° is implausibly large")


class RuckigPlanner:
    def __init__(self, dof, control_hz, max_vel, max_acc, max_jerk,
                 jump_threshold, brake_acc_mult, brake_jerk_mult):
        if not (0 < control_hz <= 1000):
            raise ValueError(
                f"ruckig.control_hz must be in (0, 1000], got {control_hz}")

        self.dof = dof
        self.dt  = 1.0 / control_hz
        self.otg = Ruckig(dof, self.dt)
        self.inp = InputParameter(dof)
        self.out = OutputParameter(dof)

        def to_array(val):
            if isinstance(val, (list, tuple, np.ndarray)):
                return [float(v) for v in val]
            return [float(val)] * dof

        self._max_vel  = to_array(max_vel)
        self._max_acc  = to_array(max_acc)
        self._max_jerk = to_array(max_jerk)

        for name, arr in (("max_vel",  self._max_vel),
                          ("max_acc",  self._max_acc),
                          ("max_jerk", self._max_jerk)):
            if len(arr) != dof:
                raise ValueError(
                    f"ruckig.{name} length {len(arr)} != dof {dof}")
            if any(v <= 0 for v in arr):
                raise ValueError(
                    f"ruckig.{name} must be > 0 for every axis, got {arr}")

        if jump_threshold <= 0:
            raise ValueError(
                f"ruckig.jump_threshold_rad must be > 0, got {jump_threshold}")
        if brake_acc_mult <= 0 or brake_jerk_mult <= 0:
            raise ValueError(
                "ruckig.brake_*_mult must be > 0, got "
                f"brake_acc_mult={brake_acc_mult}, "
                f"brake_jerk_mult={brake_jerk_mult}")

        self.jump_threshold = float(jump_threshold)
        self._brake_acc  = [a * float(brake_acc_mult)  for a in self._max_acc]
        self._brake_jerk = [j * float(brake_jerk_mult) for j in self._max_jerk]

        self.inp.max_velocity        = self._max_vel
        self.inp.max_acceleration    = self._max_acc
        self.inp.max_jerk            = self._max_jerk
        self.inp.target_velocity     = [0.0] * dof
        self.inp.target_acceleration = [0.0] * dof

        self._initialized = False
        self._stopped     = True
        self._has_target  = False
        self._last_pos    = np.zeros(dof)
        self._target_pos  = np.zeros(dof)

    def reset(self, m):
        m = np.asarray(m, dtype=np.float64)
        self.inp.control_interface    = ControlInterface.Position
        self.inp.current_position     = m.tolist()
        self.inp.current_velocity     = [0.0] * self.dof
        self.inp.current_acceleration = [0.0] * self.dof
        self.inp.target_velocity      = [0.0] * self.dof
        self.inp.target_acceleration  = [0.0] * self.dof
        self.inp.max_velocity         = self._max_vel
        self.inp.max_acceleration     = self._max_acc
        self.inp.max_jerk             = self._max_jerk
        np.copyto(self._last_pos, m)
        self._initialized = True
        self._has_target  = False
        self._stopped     = False

    def set_target(self, t):
        t = np.asarray(t, dtype=np.float64)
        if not np.all(np.isfinite(t)):
            return False
        self.inp.target_position = t.tolist()
        np.copyto(self._target_pos, t)
        self._has_target = True
        return True

    def sync_state(self, m):
        m = np.asarray(m, dtype=np.float64)
        if np.any(np.abs(m - self._last_pos) > self.jump_threshold):
            self.inp.current_velocity     = [0.0] * self.dof
            self.inp.current_acceleration = [0.0] * self.dof
        self.inp.current_position = m.tolist()
        np.copyto(self._last_pos, m)

    def start_brake(self):
        self.inp.control_interface   = ControlInterface.Velocity
        self.inp.target_velocity     = [0.0] * self.dof
        self.inp.target_acceleration = [0.0] * self.dof
        self.inp.max_acceleration    = self._brake_acc
        self.inp.max_jerk            = self._brake_jerk
        self._stopped                = False

    def step(self):
        if not self._initialized:
            return None
        braking = self.inp.control_interface == ControlInterface.Velocity
        if not braking and not self._has_target:
            return np.array(self.inp.current_position)
        res = self.otg.update(self.inp, self.out)
        if res not in (Result.Working, Result.Finished):
            self.inp.current_velocity     = [0.0] * self.dof
            self.inp.current_acceleration = [0.0] * self.dof
            if braking:
                self._stopped = True
            return np.array(self.inp.current_position)

        self.out.pass_to_input(self.inp)
        if braking and res == Result.Finished:
            self._stopped = True
        return np.array(self.out.new_position)

    @property
    def stopped(self):
        return self._stopped


class TrajectoryTwin(Node):
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

        r = cfg["ruckig"]
        self.planner = RuckigPlanner(
            self.ctrl_dof,
            r["control_hz"],
            r.get("max_vel", 6.0),
            r.get("max_acc", 20.0),
            r.get("max_jerk", 400.0),
            r.get("jump_threshold_rad", 0.3),
            r.get("brake_acc_mult", 4.0),
            r.get("brake_jerk_mult", 8.0),
        )

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
        # v3.6 [F]: the tag is kept here until _publish() actually attaches
        # it to an outgoing message. Never consumed inside servo_tick.
        self._cmd_tag            = ""
        self._cmd_tag_t          = 0.0   # monotonic time when the tag arrived

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

        self.create_timer(1.0 / r["control_hz"], self.servo_tick)

        self.log.info("twin_started", extra={
            "robot":   self.robot_name,
            "dof":     self.ctrl_dof,
            "rate_hz": r["control_hz"],
        })

    # --------------------------------------------------------------------- #
    # Callbacks
    # --------------------------------------------------------------------- #
    def on_js(self, msg):
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                i = self.name_to_idx.get(n)
                if i is not None:
                    self._actual[i] = float(p)
            self._has_actual = True
            self._last_js_t = time.monotonic()

    def on_cmd(self, msg):
        n = self.ctrl_dof
        with self.lock:
            if len(msg.position) >= n:
                new_cmd = [float(x) for x in msg.position[:n]]
                if new_cmd != self._last_received_cmd_deg:
                    self._last_received_cmd_deg = list(new_cmd)
                    self.log.info("cmd_received", extra={"cmd_deg": new_cmd})
                self._cmd_deg = new_cmd
                tag = msg.header.frame_id
                if tag.startswith("p;"):
                    self._cmd_tag   = tag
                    self._cmd_tag_t = time.monotonic()
                else:
                    self._cmd_tag   = ""
                    self._cmd_tag_t = 0.0
                self._cmd_ok = True
                self._last_cmd_t = time.monotonic()

    def on_enable(self, msg):
        with self.lock:
            new = bool(msg.data)
            if new != self._enable:
                self.log.info("enable_changed", extra={"enable": new})
            self._enable = new

    def on_estop(self, msg):
        with self.lock:
            new = bool(msg.data)
            if new != self._estop:
                if new:
                    self.log.error("estop_active")
                else:
                    self.log.info("estop_cleared")
            self._estop = new

    # --------------------------------------------------------------------- #
    # Servo tick
    # --------------------------------------------------------------------- #
    def servo_tick(self):
        now = time.monotonic()

        with self.lock:
            enable        = self._enable
            estop         = self._estop
            cmd           = list(self._cmd_deg)
            cmd_ok        = self._cmd_ok
            last_cmd_t    = self._last_cmd_t
            actual        = self._actual.copy()
            has_actual    = self._has_actual
            last_js_t     = self._last_js_t
            last_applied  = self._last_applied_cmd_deg
            # v3.6 [F]: read the tag but do NOT clear it here.
            tag           = self._cmd_tag
            tag_t         = self._cmd_tag_t

        # ---- freshness checks -----------------------------------------
        fresh = has_actual and (now - last_js_t) <= 0.5

        if (now - last_cmd_t) > 0.5:
            if cmd_ok and not self._cmd_timeout_logged:
                self.log.warning("cmd_timeout", extra={
                    "last_cmd_age_s": round(now - last_cmd_t, 2)})
                self._cmd_timeout_logged = True
            cmd_ok = False
        else:
            self._cmd_timeout_logged = False

        if not fresh and has_actual:
            if not self._no_feedback_logged:
                self.log.error("no_feedback", extra={
                    "last_js_age_s": round(now - last_js_t, 2)})
                self._no_feedback_logged = True
        else:
            self._no_feedback_logged = False

        want_run = enable and (not estop) and fresh and cmd_ok

        np.copyto(self._ctrl_actual, actual[self.ctrl_indices])

        # ---- run path -------------------------------------------------
        if want_run and self._state != STOPPING:
            target = self._ctrl_actual.copy()
            valid = True
            for i, j in enumerate(self.controlled_joints):
                deg = cmd[i]
                lo, hi = self.limits_deg[j]
                if not math.isfinite(deg) or deg < lo or deg > hi:
                    valid = False
                    break
                target[i] = math.radians(deg)

            if not valid:
                if not self._bad_cmd_logged:
                    self._bad_cmd_logged = True
                    self.log.warning("bad_command", extra={"cmd_deg": cmd})
                if self._state == RUNNING:
                    self._begin_brake("bad command")
            else:
                self._bad_cmd_logged = False
                if self._state == IDLE:
                    self.planner.reset(self._ctrl_actual)
                    self.log.info("state_changed",
                                  extra={"from": "IDLE", "to": "RUNNING"})
                    self._state = RUNNING

                needs_target_update = (
                    last_applied is None or cmd != last_applied
                )
                if needs_target_update:
                    if not self.planner.set_target(target):
                        self.log.warning("invalid_target",
                                         extra={"target_rad": target.tolist()})
                        self._begin_brake("invalid target")
                    else:
                        with self.lock:
                            self._last_applied_cmd_deg = list(cmd)

                self.planner.sync_state(self._ctrl_actual)

                if not self.planner.stopped:
                    sp = self.planner.step()
                    if sp is not None:
                        self._publish(sp, tag, tag_t)

        # ---- brake path -----------------------------------------------
        elif (not want_run) and self._state == RUNNING:
            reason = ("estop"      if estop
                      else "disable" if not enable
                      else "no feedback" if not fresh
                      else "cmd timeout")
            self._begin_brake(reason)

        # ---- stopping path --------------------------------------------
        if self._state == STOPPING:
            self.planner.sync_state(self._ctrl_actual)
            sp = self.planner.step()
            if sp is not None:
                self._publish(sp, tag, tag_t)
            if self.planner.stopped:
                self.log.info("state_changed",
                              extra={"from": "STOPPING", "to": "IDLE"})
                self._state = IDLE
                with self.lock:
                    self._last_applied_cmd_deg = None

    # --------------------------------------------------------------------- #
    # Publish — consumes the tag ONLY when actually attached
    # --------------------------------------------------------------------- #
    def _publish(self, ctrl_positions, tag, tag_t):
        pos = self._msg.position
        for i, idx in enumerate(self.ctrl_indices):
            pos[idx] = float(ctrl_positions[i])

        self._msg.header.stamp = self.get_clock().now().to_msg()
        if tag:
            self._msg.header.frame_id = f"{tag};{time.monotonic_ns()}"
        else:
            self._msg.header.frame_id = ""
        self.setpoint_pub.publish(self._msg)

        # v3.6 [F]: consume the tag ONLY after it has been attached to a
        # real outgoing message. A subsequent tick that publishes without
        # a new command will see tag="" (correctly) because the previous
        # command already completed its chain.
        if tag:
            with self.lock:
                # Only clear if no newer tag has arrived in the meantime.
                if self._cmd_tag_t == tag_t:
                    self._cmd_tag = ""
                    self._cmd_tag_t = 0.0

        now = time.monotonic()
        if now - self._last_setpoint_log >= 1.0:
            self._last_setpoint_log = now
            lines = [
                f"{'joint':<10}{'cmd_deg':>12}{'actual_deg':>14}{'diff_deg':>14}",
                "-" * 50,
            ]
            for i, j in enumerate(self.controlled_joints):
                idx = self.ctrl_indices[i]
                c = (math.degrees(float(self.planner._target_pos[i]))
                     if self.planner._has_target else None)
                a = math.degrees(float(self._actual[idx]))
                d = (c - a) if c is not None else None
                cs = f"{c:>12.3f}" if c is not None else f"{'--':>12}"
                as_ = f"{a:>14.3f}"
                ds = f"{d:>14.3f}" if d is not None else f"{'--':>14}"
                lines.append(f"{j:<10}{cs}{as_}{ds}")
            self.log.info("trajectory_status",
                          extra={"table": "\n".join(lines)})

    def _begin_brake(self, reason):
        if self._state == RUNNING:
            self.log.warning("state_changed", extra={
                "from": "RUNNING", "to": "STOPPING", "reason": reason})
        self.planner.start_brake()
        self._state = STOPPING

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


if __name__ == "__main__":
    main()