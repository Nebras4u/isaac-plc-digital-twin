import numpy as np

from ruckig import Ruckig, InputParameter, OutputParameter, Result, ControlInterface


class PassthroughPlanner:
    """Planner with the RuckigPlanner API but NO trajectory smoothing.

    Used when robot.yaml has `ruckig.enabled: false`.  The limit checks, the
    state machine, the watchdogs and the v4.0 validation in TrajectoryTwin
    still apply; only the jerk/acceleration shaping is bypassed, so the
    setpoint jumps straight to the commanded target.
    """

    def __init__(self, dof):
        self.dof = dof
        self._initialized = False
        self._stopped     = True
        self._braking     = False
        self._has_target  = False
        self._last_pos    = np.zeros(dof)
        self._target_pos  = np.zeros(dof)

    def reset(self, m):
        np.copyto(self._last_pos, np.asarray(m, dtype=np.float64))
        self._initialized = True
        self._has_target  = False
        self._braking     = False
        self._stopped     = False

    def set_target(self, t):
        t = np.asarray(t, dtype=np.float64)
        if not np.all(np.isfinite(t)):
            return False
        np.copyto(self._target_pos, t)
        self._has_target = True
        return True

    def sync_state(self, m):
        m = np.asarray(m, dtype=np.float64)
        if not np.all(np.isfinite(m)):
            return
        np.copyto(self._last_pos, m)

    def start_brake(self):
        self._braking = True
        self._stopped = False

    def step(self):
        if not self._initialized:
            return None
        if self._braking:
            self._stopped = True          # hold the measured position
            return self._last_pos.copy()
        if not self._has_target:
            return self._last_pos.copy()
        return self._target_pos.copy()

    @property
    def stopped(self):
        return self._stopped


def build_planner(r, dof):
    """Planner from the robot.yaml `ruckig:` section.

    `enabled` (default true) must be a real YAML bool.  When false only
    `control_hz` is needed (it still sets the servo loop rate).
    """
    enabled = r.get("enabled", True)
    if not isinstance(enabled, bool):
        raise ValueError(
            f"ruckig.enabled must be true or false, got {enabled!r}")
    hz = r["control_hz"]
    if not enabled:
        # Normal operation (Test_Mode=false, CmdEnable=true) must execute
        # CmdAng through Ruckig ONLY. No pass-through bypass is allowed.
        raise ValueError(
            "ruckig.enabled=false is not allowed: every CmdAng must be "
            "executed through Ruckig. Set ruckig.enabled: true in robot.yaml. "
            "(For benchmarking only: ruckig.test_mode_passthrough: true "
            "allows pass-through while /plc/test_mode is true.)")
    return RuckigPlanner(
        dof,
        hz,
        r.get("max_vel", 6.0),
        r.get("max_acc", 20.0),
        r.get("max_jerk", 400.0),
        r.get("jump_threshold_rad", 0.3),
        r.get("brake_acc_mult", 4.0),
        r.get("brake_jerk_mult", 8.0),
    )


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
        # Ruckig Velocity-mode fails when current_acceleration × dt > max_velocity.
        # Zero it to guarantee a solvable braking problem.
        self.inp.current_acceleration = [0.0] * self.dof
        self.inp.max_acceleration    = self._brake_acc
        self.inp.max_jerk            = self._brake_jerk
        self._stopped                = False

    def step(self):
        if not self._initialized:
            return None
        braking = self.inp.control_interface == ControlInterface.Velocity
        if not braking and not self._has_target:
            return np.array(self.inp.current_position)
        try:
            res = self.otg.update(self.inp, self.out)
        except Exception:
            self.inp.current_velocity     = [0.0] * self.dof
            self.inp.current_acceleration = [0.0] * self.dof
            if braking:
                self._stopped = True
            return np.array(self.inp.current_position)
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
