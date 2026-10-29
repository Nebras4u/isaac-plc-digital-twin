# trajectory_twin.py
# Vendor-neutral Ruckig-based trajectory generator.
# Reads robot definition from robot.yaml.
#
# v4.0 — industrial type-safety (mirrors plc_ros_bridge.py / physics_twin.py):
#   * on_js  : name/position length check, per-joint type check, NaN/Inf
#              rejection. Refuses to contaminate _actual[] with non-finite.
#   * on_cmd : length check, per-joint type check, NaN/Inf rejection.
#              A bad message is rejected entirely (no partial state update,
#              no tag processing) so the 0.5 s command timeout remains the
#              braking safety mechanism.
#   * _publish : defense-in-depth check on Ruckig output. If the planner
#              ever yields a non-finite position, /servo_setpoints is not
#              published (tag preserved for the next valid publish).
#   * Per-reason rejection counters (`_stats`), surfaced in the 1-s status
#     log and in throttled warnings (`_warn`).
#   * `warn_cooldown` param (default 5 s) read from robot.yaml.
#
# v3.6 changes preserved:
#   * [A] no np.interp on the passive axes
#   * [B] needs_target_update vs _last_applied_cmd_deg
#   * [C] _last_applied_cmd_deg guarded by lock
#   * [D] STOPPING→IDLE clears _last_applied_cmd_deg
#   * [E] RuckigPlanner validates all params at construction
#   * [F] _cmd_tag consumed only by _publish() after actual attachment

import math
import os
import threading
import time
from collections import defaultdict

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
        # v4.0: refuse to feed non-finite state into Ruckig
        if not np.all(np.isfinite(m)):
            return
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

        # v4.0: type-safety state
        self._warn_cooldown = float(cfg.get("warn_cooldown", 5.0))
        self._stats         = defaultdict(lambda: defaultdict(int))
        self._warned        = {}

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

        self.create_timer(1.0 / r["control_hz"], self.servo_tick)

        self.log.info("twin_started", extra={
            "robot":   self.robot_name,
            "dof":     self.ctrl_dof,
            "rate_hz": r["control_hz"],
        })

    # --------------------------------------------------------------------- #
    # v4.0 validation helpers
    # --------------------------------------------------------------------- #
    def _bump(self, name, reason):
        key = reason.split(":", 1)[0] if isinstance(reason, str) else str(reason)
        if not key:
            key = "bad_value"
        self._stats[name][key] += 1
        self._stats[name]["rejected_total"] += 1

    def _warn(self, name, message):
        now = time.monotonic()
        if now - self._warned.get(name, 0.0) > self._warn_cooldown:
            self._warned[name] = now
            counts = dict(self._stats.get(name, {}))
            try:
                self.log.warning(message, extra={"counts": counts})
            except Exception:
                pass

    # --------------------------------------------------------------------- #
    # Callbacks
    # --------------------------------------------------------------------- #
    def on_js(self, msg):
        if len(msg.name) != len(msg.position):
            self._bump("__joint_states__", "length_mismatch")
            self._warn("__joint_states__",
                       f"length mismatch: name={len(msg.name)} "
                       f"pos={len(msg.position)}")
            return

        valid_count = 0
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                i = self.name_to_idx.get(n)
                if i is None:
                    self._bump(n, "unknown_joint")
                    self._warn(n, f"unknown joint in /joint_states: {n!r}")
                    continue
                try:
                    f = float(p)
                except (TypeError, ValueError):
                    self._bump(n, "bad_type")
                    self._warn(n, f"bad joint_states type: {type(p).__name__}")
                    continue
                if not math.isfinite(f):
                    self._bump(n, "non_finite")
                    self._warn(n, f"non-finite joint_states[{n}]: {f!r}")
                    continue
                self._actual[i] = f
                valid_count += 1
            if valid_count > 0:
                self._has_actual = True
                self._last_js_t = time.monotonic()

    def on_cmd(self, msg):
        n = self.ctrl_dof
        if len(msg.position) < n:
            self._bump("__cmd__", f"short_cmd:{len(msg.position)}<{n}")
            self._warn("__cmd__",
                       f"cmd too short: {len(msg.position)} < {n}")
            return

        new_cmd = []
        for i, x in enumerate(msg.position[:n]):
            jname = self.controlled_joints[i]
            try:
                f = float(x)
            except (TypeError, ValueError):
                self._bump(jname, "bad_type")
                self._warn(jname, f"bad cmd type: {type(x).__name__}")
                return
            if not math.isfinite(f):
                self._bump(jname, "non_finite")
                self._warn(jname, f"non-finite cmd[{jname}]: {f!r}")
                return
            new_cmd.append(f)

        with self.lock:
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
    # Servo tick (v3.6 logic preserved)
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
            tag           = self._cmd_tag
            tag_t         = self._cmd_tag_t

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
                    self._bump(j, "below_min" if math.isfinite(deg) and deg < lo
                               else "above_max" if math.isfinite(deg)
                               else "non_finite_at_servo")
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
    # Publish — v4.0: defense-in-depth on planner output
    # --------------------------------------------------------------------- #
    def _publish(self, ctrl_positions, tag, tag_t):
        # v4.0: refuse to publish non-finite to /servo_setpoints.
        # Tag is preserved (not consumed) so the next valid publish can
        # still attach it if still relevant — the latency probe will
        # reject stale chains by age, not here.
        try:
            positions_list = [float(x) for x in ctrl_positions]
        except (TypeError, ValueError):
            self._bump("__publish__", "bad_type")
            self._warn("__publish__", "non-numeric ctrl_positions")
            return
        if not all(math.isfinite(x) for x in positions_list):
            self._bump("__publish__", "non_finite")
            self._warn("__publish__", "non-finite ctrl_positions")
            return

        pos = self._msg.position
        for i, idx in enumerate(self.ctrl_indices):
            pos[idx] = positions_list[i]

        self._msg.header.stamp = self.get_clock().now().to_msg()
        if tag:
            self._msg.header.frame_id = f"{tag};{time.monotonic_ns()}"
        else:
            self._msg.header.frame_id = ""
        self.setpoint_pub.publish(self._msg)

        if tag:
            with self.lock:
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

            # v4.0: rejection summary
            total_rej = sum(c.get("rejected_total", 0)
                            for c in self._stats.values())
            if total_rej > 0:
                lines.append("-" * 50)
                lines.append(f"rejections_total: {total_rej}")
                for name, counts in sorted(self._stats.items()):
                    rej = {k: v for k, v in counts.items()
                           if k != "rejected_total" and v > 0}
                    if rej or counts.get("rejected_total", 0) > 0:
                        lines.append(f"  {name}: {dict(counts)}")

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