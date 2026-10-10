# krc_twin.py
# Digital twin of the KUKA Robot Controller (KRC).
# Ruckig-based online trajectory generation.
# NO OPC UA — commands come from /plc/*, feedback from /joint_states.

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from ruckig import (
    Ruckig, InputParameter, OutputParameter,
    Result, ControlInterface,
)


# =========================================================================
# Configuration
# =========================================================================
CONTROL_HZ = 60.0

# ترتيب المفاصل في Isaac (5 مفاصل)
ROS_ORDER = ["joint_1", "joint_2", "joint_3", "joint_5", "joint_4"]
NUM_DOF = len(ROS_ORDER)

# المفاصل الأربعة التي نتحكم بها (من PLC)
CONTROLLED_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4"]

# تحويل الاسم → فهرس في ROS_ORDER
NAME_TO_IDX = {name: i for i, name in enumerate(ROS_ORDER)}

# حدود KR210 (درجات)
AXIS_LIMITS_DEG = {
    "joint_1": (-185.0, 185.0),
    "joint_2": (-188.0,  35.0),
    "joint_3": (-114.6, 143.2),
    "joint_4": (-350.0, 350.0),
}

# Trajectory limits (rad/s, rad/s², rad/s³)
MAX_VEL  = 3.0   # Increase maximum joint velocity
MAX_ACC  = 10.0  # Double the acceleration limits
MAX_JERK = 80.0  # Allow punchier jerk profiles

# Jump protection
JUMP_THRESHOLD_RAD = 0.30

# States
IDLE, RUNNING, STOPPING = 0, 1, 2


# =========================================================================
# Ruckig wrapper
# =========================================================================
class RuckigPlanner:
    """Thin wrapper around Ruckig for 5-DOF position control."""

    def __init__(self):
        self.dof = NUM_DOF
        self.dt  = 1.0 / CONTROL_HZ

        self.otg = Ruckig(self.dof, self.dt)
        self.inp = InputParameter(self.dof)
        self.out = OutputParameter(self.dof)

        self.inp.max_velocity     = [MAX_VEL]  * self.dof
        self.inp.max_acceleration = [MAX_ACC]  * self.dof
        self.inp.max_jerk         = [MAX_JERK] * self.dof

        self.inp.target_velocity     = [0.0] * self.dof
        self.inp.target_acceleration = [0.0] * self.dof

        self._initialized = False
        self._stopped     = True
        self._has_target  = False
        self._last_pos    = np.zeros(self.dof)

    def reset(self, measured_pos):
        """Full reset from measured state. Called on rising edge of enable."""
        measured = np.asarray(measured_pos, dtype=np.float64)

        self.inp.control_interface    = ControlInterface.Position
        self.inp.current_position     = measured.tolist()
        self.inp.current_velocity     = [0.0] * self.dof
        self.inp.current_acceleration = [0.0] * self.dof
        self.inp.target_velocity      = [0.0] * self.dof
        self.inp.target_acceleration  = [0.0] * self.dof

        np.copyto(self._last_pos, measured)

        self._initialized = True
        self._has_target  = False
        self._stopped     = False

    def set_target(self, target):
        target = np.asarray(target, dtype=np.float64)
        if not np.all(np.isfinite(target)):
            return False
        self.inp.target_position = target.tolist()
        self._has_target = True
        return True

    def sync_state(self, measured_pos):
        """Inject measured position; reset derivatives on jump."""
        measured = np.asarray(measured_pos, dtype=np.float64)
        delta = np.abs(measured - self._last_pos)

        if np.any(delta > JUMP_THRESHOLD_RAD):
            self.inp.current_velocity     = [0.0] * self.dof
            self.inp.current_acceleration = [0.0] * self.dof

        self.inp.current_position = measured.tolist()
        np.copyto(self._last_pos, measured)

    def start_brake(self):
        """Jerk-limited braking: velocity interface, target_velocity = 0."""
        self.inp.control_interface   = ControlInterface.Velocity
        self.inp.target_velocity     = [0.0] * self.dof
        self.inp.target_acceleration = [0.0] * self.dof
        self._stopped = False

    def step(self):
        """Advance one control cycle. Returns next position or None."""
        if not self._initialized:
            return None

        braking = self.inp.control_interface == ControlInterface.Velocity

        if not braking and not self._has_target:
            return np.array(self.inp.current_position)

        res = self.otg.update(self.inp, self.out)

        if res not in (Result.Working, Result.Finished):
            # error — hold position
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


# =========================================================================
# KRC Twin ROS node
# =========================================================================
class KrcTwin(Node):

    def __init__(self):
        super().__init__("krc_twin")

        # ---- Publisher ----
        self.setpoint_pub = self.create_publisher(JointState, "/servo_setpoints", 10)

        # ---- State (protected by lock) ----
        self.lock = threading.Lock()

        self._actual      = np.zeros(NUM_DOF)
        self._has_actual  = False
        self._last_js_t   = 0.0

        self._enable      = False
        self._estop       = False
        self._cmd_deg     = [0.0, 0.0, 0.0, 0.0]   # A1..A4 من PLC
        self._cmd_ok      = False

        self._state       = IDLE

        # ---- Pre-allocated message ----
        self._msg = JointState()
        self._msg.name = list(ROS_ORDER)

        # ---- Planner ----
        self.planner = RuckigPlanner()

        # ---- Subscriptions ----
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(JointState, "/plc/cmd_ang", self.on_cmd, 10)
        self.create_subscription(Bool, "/plc/axis_enable", self.on_enable, 10)
        self.create_subscription(Bool, "/plc/emergency_stop", self.on_estop, 10)

        # ---- Timer ----
        self.create_timer(1.0 / CONTROL_HZ, self.servo_tick)

        self.get_logger().info(
            f"KRC Twin ready. DOF={NUM_DOF}, controlled={CONTROLLED_JOINTS}, "
            f"rate={CONTROL_HZ} Hz"
        )

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------
    def on_js(self, msg: JointState):
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                idx = NAME_TO_IDX.get(name)
                if idx is not None:
                    self._actual[idx] = float(pos)
            self._has_actual = True
            self._last_js_t  = time.monotonic()

    def on_cmd(self, msg: JointState):
        # msg.position = [A1, A2, A3, A4] in degrees
        with self.lock:
            if len(msg.position) >= 4:
                self._cmd_deg = [float(x) for x in msg.position[:4]]
                self._cmd_ok  = True

    def on_enable(self, msg: Bool):
        with self.lock:
            self._enable = bool(msg.data)

    def on_estop(self, msg: Bool):
        with self.lock:
            self._estop = bool(msg.data)

    # ---------------------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------------------
    def get_feedback(self):
        with self.lock:
            fresh = (self._has_actual and
                     (time.monotonic() - self._last_js_t) <= 0.5)
            return fresh, self._actual.copy()

    def build_target(self, cmd_deg, actual):
        """
        Convert PLC degrees (A1..A4) to ROS radians, then build a 5-vector
        in ROS_ORDER. joint_5 holds its measured value.
        """
        target = actual.copy()
        for i, joint in enumerate(CONTROLLED_JOINTS):
            idx = NAME_TO_IDX[joint]
            deg = cmd_deg[i]
            lo, hi = AXIS_LIMITS_DEG[joint]
            if not math.isfinite(deg) or deg < lo or deg > hi:
                return None
            target[idx] = math.radians(deg)
        return target

    # ---------------------------------------------------------------------
    # Servo loop
    # ---------------------------------------------------------------------
    def servo_tick(self):
        with self.lock:
            enable = self._enable
            estop  = self._estop
            cmd    = list(self._cmd_deg)
            cmd_ok = self._cmd_ok

        fresh, actual = self.get_feedback()

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
                if sp is not None:
                    self.publish(sp)

        elif self._state == RUNNING:
            self._begin_brake("fault or disable")

        if self._state == STOPPING:
            sp = self.planner.step()
            if sp is not None:
                self.publish(sp)
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


# =========================================================================
# Main
# =========================================================================
def main():
    rclpy.init()
    node = KrcTwin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()