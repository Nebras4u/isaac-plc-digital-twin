# trajectory_twin.py
# Vendor-neutral Ruckig-based trajectory generator.
# Reads robot definition from robot.yaml.

import math
import threading
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from ruckig import Ruckig, InputParameter, OutputParameter, Result, ControlInterface

IDLE, RUNNING, STOPPING = 0, 1, 2


class RuckigPlanner:
    def __init__(self, dof, control_hz, max_vel, max_acc, max_jerk, jump_threshold):
        self.dof = dof
        self.dt  = 1.0 / control_hz
        self.jump_threshold = jump_threshold
        self.otg = Ruckig(dof, self.dt)
        self.inp = InputParameter(dof)
        self.out = OutputParameter(dof)
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

        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(JointState, "/plc/cmd_ang", self.on_cmd, 10)
        self.create_subscription(Bool, "/plc/axis_enable", self.on_enable, 10)
        self.create_subscription(Bool, "/plc/emergency_stop", self.on_estop, 10)
        self.create_timer(1.0 / r["control_hz"], self.servo_tick)

        self.get_logger().info(
            f"TrajectoryTwin ready | robot={self.robot_name} | dof={self.dof} | "
            f"controlled={self.controlled_joints} | rate={r['control_hz']} Hz"
        )

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
                self._cmd_deg = [float(x) for x in msg.position[:n]]
                self._cmd_ok = True
                self._last_cmd_t = time.monotonic()

    def on_enable(self, msg):
        with self.lock: self._enable = bool(msg.data)

    def on_estop(self, msg):
        with self.lock: self._estop = bool(msg.data)

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
        # watchdog: cmd_ang must arrive within 0.5 s
        if (time.monotonic() - last_cmd_t) > 0.5:
            cmd_ok = False
        want_run = enable and (not estop) and fresh and cmd_ok

        if want_run:
            if self._state != RUNNING:
                self.planner.reset(actual)
                self._state = RUNNING
                self.get_logger().info("→ RUNNING")
            target = self.build_target(cmd, actual)
            if target is None:
                self._begin_brake("bad command")
            elif not self.planner.set_target(target):
                self._begin_brake("invalid target")
            else:
                self.planner.sync_state(actual)
                sp = self.planner.step()
                if sp is not None: self.publish(sp)
        elif self._state == RUNNING:
            self._begin_brake("fault or disable")

        if self._state == STOPPING:
            sp = self.planner.step()
            if sp is not None: self.publish(sp)
            if self.planner.stopped:
                self._state = IDLE
                self.get_logger().info("→ IDLE")

    def publish(self, positions):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._msg.position = positions.tolist()
        self.setpoint_pub.publish(self._msg)

    def _begin_brake(self, reason):
        if self._state == RUNNING:
            self.get_logger().warn(f"→ STOPPING ({reason})")
        self.planner.start_brake()
        self._state = STOPPING


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