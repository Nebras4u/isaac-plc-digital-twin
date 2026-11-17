# trajectory_twin.py
# Vendor-neutral Ruckig-based trajectory generator.
# Reads robot definition from robot.yaml.
#
# v4.1 — `ruckig.enabled` (robot.yaml, default true):
#   * false -> PassthroughPlanner: no jerk/acc shaping, the setpoint
#     goes straight to the commanded target.  Limits, state machine,
#     watchdogs and v4.0 validation are unchanged.  ruckig.control_hz
#     still sets the servo loop rate.
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
#
# Implementation lives in the trajectory package. This file is the
# stable entry point.

from trajectory.limits import _validate_limits  # noqa: F401
from trajectory.planners import (  # noqa: F401
    PassthroughPlanner, RuckigPlanner, build_planner,
)
from trajectory.node import (  # noqa: F401
    IDLE, RUNNING, STOPPING, TrajectoryTwin, main,
)

if __name__ == "__main__":
    main()
