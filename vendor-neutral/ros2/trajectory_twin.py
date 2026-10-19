# trajectory_twin.py
# Vendor-neutral Ruckig-based trajectory generator.
# Reads robot definition from robot.yaml.
#
# Logging: reuses bridge_logger.setup_logging for structured output
# (human-readable + JSON Lines, with rotation).

import math
import os
import threading
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from ruckig import Ruckig, InputParameter, OutputParameter, Result, ControlInterface
from bridge_logger import setup_logging

IDLE, RUNNING, STOPPING = 0, 1, 2


class RuckigPlanner:
    def __init__(self, dof, control_hz, max_vel, max_acc, max_jerk, jump_threshold):
        self.dof = dof
        self.dt  = 1.0 / control_hz
        self.jump_threshold = jump_threshold
        self.otg = Ruckig(dof, self.dt)
        self.inp = InputParameter(dof)
        self.out = OutputParameter(dof)

        # Normal-motion limits (from robot.yaml).
        self._max_vel  = max_vel
        self._max_acc  = max_acc
        self._max_jerk = max_jerk

        # Aggressive braking profile: stop as fast as mechanically
        # reasonable when estop / disable fires.
        self._brake_acc  = max_acc  * 4.0
        self._brake_jerk = max_jerk * 8.0

        self.inp.max_velocity     = [max_vel]  * dof
        self.inp.max_acceleration = [max_acc]  * dof
        self.inp.max_jerk         = [max_jerk] * dof
        self.inp.target_velocity     = [0.0] * dof
        self.inp.target_acceleration = [0.0] * dof
        self._initialized = False
        self._stopped = True
        self._has_target = False
        self._last_pos = np.zeros(dof)

    def reset(self, m):
        m = np.asarray(m, dtype=np.float64)
        self.inp.control_interface    = ControlInterface.Position
        self.inp.current_position     = m.tolist()
        self.inp.current_velocity     = [0.0] * self.dof
        self.inp.current_acceleration = [0.0] * self.dof
        self.inp.target_velocity      = [0.0] * self.dof
        self.inp.target_acceleration  = [0.0] * self.dof
        # Restore normal-motion limits.
        self.inp.max_velocity     = [self._max_vel]  * self.dof
        self.inp.max_acceleration = [self._max_acc]  * self.dof
        self.inp.max_jerk         = [self._max_jerk] * self.dof
        np.copyto(self._last_pos, m)
        self._initialized = True
        self._has_target = False
        self._stopped = False

    def set_target(self, t):
        t = np.asarray(t, dtype=np.float64)
        if not np.all(np.isfinite(t)): return False
        self.inp.target_position = t.tolist()
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
        self.inp.control_interface = ControlInterface.Velocity
        self.inp.target_velocity = [0.0] * self.dof
        self.inp.target_acceleration = [0.0] * self.dof
        # Switch to the aggressive braking profile.
        self.inp.max_acceleration = [self._brake_acc]  * self.dof
        self.inp.max_jerk         = [self._brake_jerk] * self.dof
        self._stopped = False

    def step(self):
        if not self._initialized: return None
        braking = self.inp.control_interface == ControlInterface.Velocity
        if not braking and not self._has_target:
            return np.array(self.inp.current_position)
        res = self.otg.update(self.inp, self.out)
        if res not in (Result.Working, Result.Finished):
            self.inp.current_velocity     = [0.0] * self.dof
            self.inp.current_acceleration = [0.0] * self.dof
            if braking: self._stopped = True
            return np.array(self.inp.current_position)
        self.out.pass_to_input(self.inp)
        if braking and res == Result.Finished: self._stopped = True
        return np.array(self.out.new_position)

    @property
    def stopped(self): return self._stopped


class TrajectoryTwin(Node):
    def __init__(self):
        super().__init__("trajectory_twin")
        self.declare_parameter("robot", "robot.yaml")
        with open(self.get_parameter("robot").value) as f:
            cfg = yaml.safe_load(f)["robot"]

        self.robot_name = cfg["name"]
        self.all_joints = cfg["all_joints"]
        self.controlled_joints = cfg["controlled_joints"]
        self.limits_deg = cfg["limits_deg"]
        self.dof = len(self.all_joints)
        self.name_to_idx = {n: i for i, n in enumerate(self.all_joints)}

        r = cfg["ruckig"]
        self.planner = RuckigPlanner(
            self.dof, r["control_hz"], r["max_vel"], r["max_acc"],
            r["max_jerk"], r["jump_threshold_rad"],
        )

        # ---- structured logging ----
        log_cfg = {"logging": cfg.get("logging", {})} if "logging" in cfg else {
            "logging": {
                "level": "INFO",
                "console": True,
                "to_ros": False,
                "file": "/workspace/plc_bridge_logs/trajectory.log",
                "json_file": "/workspace/plc_bridge_logs/trajectory.jsonl",
                "max_bytes": 10 * 1024 * 1024,
                "backup_count": 5,
            }
        }
        self.log = setup_logging(
            log_cfg,
            logger_name="trajectory_twin",
            ros_logger=self.get_logger(),
        )

        self.setpoint_pub = self.create_publisher(JointState, "/servo_setpoints", 10)
        self.lock = threading.Lock()
        self._actual = np.zeros(self.dof)
        self._has_actual = False
        self._last_js_t = 0.0
        self._enable = False
        self._estop = False
        self._cmd_deg = [0.0] * len(self.controlled_joints)
        self._cmd_ok = False
        self._last_cmd_t = 0.0
        self._state = IDLE
        self._msg = JointState()
        self._msg.name = list(self.all_joints)

        # ---- logging state (throttling) ----
        self._last_setpoint_log = 0.0
        self._cmd_timeout_logged = False
        self._no_feedback_logged = False
        self._estop_logged = False
        self._last_cmd_logged = None
        self._bad_cmd_logged = False
        self._cmd_tag = ""   # latency-probe tag from the bridge (optional)

        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(JointState, "/plc/cmd_ang", self.on_cmd, 10)
        self.create_subscription(Bool, "/plc/axis_enable", self.on_enable, 10)
        self.create_subscription(Bool, "/plc/emergency_stop", self.on_estop, 10)
        self.create_timer(1.0 / r["control_hz"], self.servo_tick)

        self.log.info(
            "twin_started",
            extra={
                "robot": self.robot_name,
                "dof": self.dof,
                "controlled": self.controlled_joints,
                "rate_hz": r["control_hz"],
            },
        )

    # ------------------------------------------------------------------
    def on_js(self, msg):
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                i = self.name_to_idx.get(n)
                if i is not None: self._actual[i] = float(p)
            self._has_actual = True
            self._last_js_t = time.monotonic()

    def on_cmd(self, msg):
        n = len(self.controlled_joints)
        with self.lock:
            if len(msg.position) >= n:
                new_cmd = [float(x) for x in msg.position[:n]]
                if new_cmd != self._last_cmd_logged:
                    self._last_cmd_logged = list(new_cmd)
                    self.log.info(
                        "cmd_received",
                        extra={"cmd_deg": new_cmd},
                    )
                self._cmd_deg = new_cmd
                tag = msg.header.frame_id
                self._cmd_tag = tag if tag.startswith("p;") else ""
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

    def get_feedback(self):
        with self.lock:
            fresh = self._has_actual and (time.monotonic() - self._last_js_t) <= 0.5
            return fresh, self._actual.copy()

    def build_target(self, cmd_deg, actual):
        t = actual.copy()
        for i, j in enumerate(self.controlled_joints):
            idx = self.name_to_idx[j]
            deg = cmd_deg[i]
            lo, hi = self.limits_deg[j]
            if not math.isfinite(deg) or deg < lo or deg > hi: return None
            t[idx] = math.radians(deg)
        return t

    def servo_tick(self):
        with self.lock:
            enable, estop = self._enable, self._estop
            cmd, cmd_ok = list(self._cmd_deg), self._cmd_ok
            last_cmd_t = self._last_cmd_t
        fresh, actual = self.get_feedback()

        # ---- watchdog: cmd_ang must arrive within 0.5 s ----
        if (time.monotonic() - last_cmd_t) > 0.5:
            if cmd_ok and not self._cmd_timeout_logged:
                self.log.warning(
                    "cmd_timeout",
                    extra={"last_cmd_age_s": round(time.monotonic() - last_cmd_t, 2)},
                )
                self._cmd_timeout_logged = True
            cmd_ok = False
        else:
            self._cmd_timeout_logged = False

        # ---- no feedback watchdog ----
        if not fresh and self._has_actual:
            if not self._no_feedback_logged:
                self.log.error(
                    "no_feedback",
                    extra={"last_js_age_s": round(time.monotonic() - self._last_js_t, 2)},
                )
                self._no_feedback_logged = True
        else:
            self._no_feedback_logged = False

        want_run = enable and (not estop) and fresh and cmd_ok

        # While STOPPING, the brake always runs to completion (handled at the
        # bottom). A restart is only possible from IDLE, so a persistent bad
        # command can never cancel a brake in progress.
        if want_run and self._state != STOPPING:
            target = self.build_target(cmd, actual)
            if target is None:
                if not self._bad_cmd_logged:
                    self._bad_cmd_logged = True
                    self.log.warning(
                        "bad_command",
                        extra={
                            "cmd_deg": cmd,
                            "limits": {j: self.limits_deg[j]
                                       for j in self.controlled_joints},
                        },
                    )
                if self._state == RUNNING:
                    self._begin_brake("bad command")
                # IDLE + bad command: stay idle, do not start.
            else:
                self._bad_cmd_logged = False
                if self._state == IDLE:
                    self.planner.reset(actual)
                    self.log.info("state_changed",
                                  extra={"from": "IDLE", "to": "RUNNING"})
                    self._state = RUNNING
                if not self.planner.set_target(target):
                    self.log.warning("invalid_target",
                                     extra={"target_rad": target.tolist()})
                    self._begin_brake("invalid target")
                else:
                    self.planner.sync_state(actual)
                    sp = self.planner.step()
                    if sp is not None: self.publish(sp, cmd, actual)
        elif (not want_run) and self._state == RUNNING:
            reason = "fault or disable"
            if estop: reason = "estop"
            elif not enable: reason = "disable"
            elif not fresh: reason = "no feedback"
            elif not cmd_ok: reason = "cmd timeout"
            self._begin_brake(reason)

        if self._state == STOPPING:
            sp = self.planner.step()
            if sp is not None: self.publish(sp, cmd, actual)
            if self.planner.stopped:
                self.log.info("state_changed",
                              extra={"from": "STOPPING", "to": "IDLE"})
                self._state = IDLE

    def publish(self, positions, cmd_deg=None, actual=None):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._msg.position = positions.tolist()
        tag = self._cmd_tag
        self._msg.header.frame_id = (
            f"{tag};{time.monotonic_ns()}" if tag else "")
        self.setpoint_pub.publish(self._msg)

        now = time.monotonic()
        if now - self._last_setpoint_log >= 1.0:
            self._last_setpoint_log = now

            # Terminal-only column-aligned table:
            #   cmd_deg    = target from /plc/cmd_ang
            #   actual_deg = measured from /joint_states
            #   diff_deg   = cmd_deg - actual_deg (tracking error)
            lines = [f"{'joint':<10}{'cmd_deg':>12}{'actual_deg':>14}{'diff_deg':>14}",
                     "-" * 50]
            for i, j in enumerate(self.controlled_joints):
                idx = self.name_to_idx[j]
                c = cmd_deg[i] if cmd_deg is not None else None
                a = math.degrees(float(actual[idx])) if actual is not None else None
                d = (c - a) if (c is not None and a is not None) else None
                cs = f"{c:>12.3f}" if c is not None else f"{'--':>12}"
                as_ = f"{a:>14.3f}" if a is not None else f"{'--':>14}"
                ds = f"{d:>14.3f}" if d is not None else f"{'--':>14}"
                lines.append(f"{j:<10}{cs}{as_}{ds}")
            print("\n".join(lines), flush=True)

    def _begin_brake(self, reason):
        if self._state == RUNNING:
            self.log.warning(
                "state_changed",
                extra={"from": "RUNNING", "to": "STOPPING", "reason": reason},
            )
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == "__main__":
    main()
