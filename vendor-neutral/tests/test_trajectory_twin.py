"""Tests for trajectory_twin.RuckigPlanner and TrajectoryTwin.servo_tick.

v3.6:
  * _publish() signature is (ctrl_positions, tag, tag_t)
  * _cmd_tag is NOT consumed in servo_tick; consumed by _publish().
  * _cmd_tag_t tracks arrival time.

v4.0:
  * servo_tick calls self._bump() on bad commands; the mock node needs it.
"""
import threading
import time
import types
from unittest.mock import MagicMock

import numpy as np
import pytest

from trajectory_twin import (
    RuckigPlanner, TrajectoryTwin, IDLE, RUNNING, STOPPING,
)


# ===========================================================================
# RuckigPlanner
# ===========================================================================
def test_ruckig_accepts_valid_params():
    p = RuckigPlanner(
        dof=4, control_hz=100,
        max_vel=1.0, max_acc=2.0, max_jerk=10.0,
        jump_threshold=0.1,
        brake_acc_mult=4.0, brake_jerk_mult=8.0,
    )
    assert p.dof == 4
    assert p._max_vel == [1.0, 1.0, 1.0, 1.0]


@pytest.mark.parametrize("bad", [
    {"control_hz": 0}, {"control_hz": 2000},
    {"max_vel": 0}, {"max_vel": -1},
    {"max_acc": 0}, {"max_jerk": 0},
    {"jump_threshold": 0},
    {"brake_acc_mult": 0}, {"brake_jerk_mult": -1},
])
def test_ruckig_rejects_bad_params(bad):
    args = dict(
        dof=4, control_hz=100,
        max_vel=1.0, max_acc=2.0, max_jerk=10.0,
        jump_threshold=0.1,
        brake_acc_mult=4.0, brake_jerk_mult=8.0,
    )
    args.update(bad)
    with pytest.raises(ValueError):
        RuckigPlanner(**args)


def test_ruckig_accepts_per_axis_arrays():
    p = RuckigPlanner(
        dof=3, control_hz=100,
        max_vel=[1.0, 2.0, 3.0],
        max_acc=[2.0, 3.0, 4.0],
        max_jerk=[10.0, 20.0, 30.0],
        jump_threshold=0.1,
        brake_acc_mult=4.0, brake_jerk_mult=8.0,
    )
    assert p._max_vel == [1.0, 2.0, 3.0]
    assert p._brake_acc[0] == 8.0


def test_ruckig_rejects_wrong_array_length():
    with pytest.raises(ValueError, match="length"):
        RuckigPlanner(
            dof=3, control_hz=100,
            max_vel=[1.0, 2.0],
            max_acc=2.0, max_jerk=10.0,
            jump_threshold=0.1,
            brake_acc_mult=4.0, brake_jerk_mult=8.0,
        )


def test_brake_jerk_scaled():
    p = RuckigPlanner(
        dof=2, control_hz=100,
        max_vel=[1.0, 2.0], max_acc=[2.0, 3.0], max_jerk=[10.0, 20.0],
        jump_threshold=0.1,
        brake_acc_mult=4.0, brake_jerk_mult=8.0,
    )
    assert p._brake_jerk == [80.0, 160.0]


def test_set_target_rejects_non_finite():
    p = RuckigPlanner(
        dof=3, control_hz=100,
        max_vel=1.0, max_acc=2.0, max_jerk=10.0,
        jump_threshold=0.1,
        brake_acc_mult=4.0, brake_jerk_mult=8.0,
    )
    p.reset([0.0, 0.0, 0.0])
    assert p.set_target([1.0, 2.0, 3.0]) is True
    assert p.set_target([1.0, float("nan"), 3.0]) is False


def test_sync_state_zeros_velocity_on_jump():
    p = RuckigPlanner(
        dof=2, control_hz=100,
        max_vel=1.0, max_acc=2.0, max_jerk=10.0,
        jump_threshold=0.3,
        brake_acc_mult=4.0, brake_jerk_mult=8.0,
    )
    p.reset([0.0, 0.0])
    p.inp.current_velocity = [1.0, 1.0]
    p.sync_state(np.array([10.0, 0.0]))
    assert p.inp.current_velocity == [0.0, 0.0]


def test_sync_state_refuses_non_finite_input():
    """v4.0: non-finite input must not contaminate Ruckig state."""
    p = RuckigPlanner(
        dof=2, control_hz=100,
        max_vel=1.0, max_acc=2.0, max_jerk=10.0,
        jump_threshold=0.3,
        brake_acc_mult=4.0, brake_jerk_mult=8.0,
    )
    p.reset([0.0, 0.0])
    before = list(p.inp.current_position)
    p.sync_state(np.array([float("nan"), 0.0]))
    assert list(p.inp.current_position) == before


# ===========================================================================
# _publish
# ===========================================================================
def _make_publish_node(tag="", tag_t=0.0):
    node = types.SimpleNamespace()
    node.ctrl_indices = [0, 1, 2, 4]
    node.all_dof = 5
    node.controlled_joints = ["j1", "j2", "j3", "j4"]
    node.setpoint_pub = MagicMock()
    node._last_setpoint_log = time.monotonic()
    node.log = MagicMock()
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: None)
    )
    node.lock = threading.Lock()
    node._cmd_tag = tag
    node._cmd_tag_t = tag_t
    node._msg = types.SimpleNamespace(
        name=["j1", "j2", "j3", "j5", "j4"],
        position=[0.0, 0.0, 0.0, 99.9, 0.0],
        velocity=[0.0] * 5,
        header=types.SimpleNamespace(stamp=None, frame_id=""),
    )
    node.planner = types.SimpleNamespace(
        _has_target=False, _target_pos=[0.0] * 4)
    # v4.0: bind real _bump / _warn
    from collections import defaultdict
    node._stats = defaultdict(lambda: defaultdict(int))
    node._warned = {}
    node._warn_cooldown = 5.0
    node._bump = TrajectoryTwin._bump.__get__(node)
    node._warn = TrajectoryTwin._warn.__get__(node)
    return node


def test_publish_preserves_passive_axis():
    node = _make_publish_node()
    TrajectoryTwin._publish(node, [1.0, 2.0, 3.0, 4.0], "", 0.0)
    assert node._msg.position[3] == 99.9
    assert node._msg.position[0] == 1.0
    assert node._msg.position[4] == 4.0


def test_publish_consumes_tag_when_attached():
    node = _make_publish_node(tag="p;100;200;300;400", tag_t=12345.0)
    TrajectoryTwin._publish(node, [1.0, 2.0, 3.0, 4.0],
                            "p;100;200;300;400", 12345.0)
    assert node._cmd_tag == ""
    assert node._cmd_tag_t == 0.0
    published = node.setpoint_pub.publish.call_args[0][0]
    assert published.header.frame_id.startswith("p;100;200;300;400;")
    assert published.header.frame_id.count(";") == 5   # 6 parts


def test_publish_keeps_newer_tag():
    node = _make_publish_node(tag="p;999;888;777;666", tag_t=99999.0)
    TrajectoryTwin._publish(node, [1.0, 2.0, 3.0, 4.0],
                            "p;100;200;300;400", 12345.0)
    assert node._cmd_tag == "p;999;888;777;666"
    assert node._cmd_tag_t == 99999.0


def test_publish_rejects_non_finite():
    """v4.0: NaN/Inf in ctrl_positions must not reach setpoint_pub."""
    node = _make_publish_node()
    TrajectoryTwin._publish(node, [1.0, float("nan"), 3.0, 4.0], "", 0.0)
    assert not node.setpoint_pub.publish.called
    assert node._stats["__publish__"]["non_finite"] == 1


# ===========================================================================
# servo_tick
# ===========================================================================
def _make_servo_node(**kw):
    cmd = kw.get("cmd_deg", [10.0, 20.0, 30.0, 40.0])
    node = types.SimpleNamespace()
    node.ctrl_dof = 4
    node.controlled_joints = ["j1", "j2", "j3", "j4"]
    node.limits_deg = {j: [-180.0, 180.0] for j in node.controlled_joints}
    node.name_to_idx = {n: i for i, n in enumerate(node.controlled_joints)}
    node.ctrl_indices = [0, 1, 2, 3]
    node.all_dof = 4
    node.lock = threading.Lock()

    node._enable = kw.get("enable", True)
    node._estop = kw.get("estop", False)
    node._cmd_deg = list(cmd)
    node._last_received_cmd_deg = kw.get("last_received", list(cmd))
    node._last_applied_cmd_deg = kw.get("last_applied")
    node._cmd_ok = kw.get("cmd_ok", True)
    node._last_cmd_t = time.monotonic() - kw.get("last_cmd_age_s", 0.0)
    node._cmd_tag = kw.get("cmd_tag", "")
    node._cmd_tag_t = kw.get("cmd_tag_t", 0.0)
    node._actual = np.zeros(4)
    node._has_actual = True
    node._last_js_t = time.monotonic()
    node._state = kw.get("state", IDLE)
    node._bad_cmd_logged = False
    node._cmd_timeout_logged = False
    node._no_feedback_logged = False
    node._ctrl_actual = np.zeros(4)
    node._ctrl_target = np.zeros(4)
    node._last_setpoint_log = time.monotonic()
    node.log = MagicMock()
    node._publish = MagicMock()

    planner = MagicMock()
    planner.stopped = (kw.get("brake_finishes", False)
                       or kw.get("planner_stopped", False))
    planner.set_target = MagicMock(return_value=True)
    planner.reset = MagicMock()
    planner.sync_state = MagicMock()
    planner.step = MagicMock(return_value=np.zeros(4))
    planner.start_brake = MagicMock()
    planner._target_pos = [0.0] * 4
    planner._has_target = True
    node.planner = planner

    # v4.0: bind real _bump / _warn / _begin_brake so servo_tick can call them
    from collections import defaultdict
    node._stats = defaultdict(lambda: defaultdict(int))
    node._warned = {}
    node._warn_cooldown = 5.0
    node._bump = TrajectoryTwin._bump.__get__(node)
    node._warn = TrajectoryTwin._warn.__get__(node)
    node._begin_brake = TrajectoryTwin._begin_brake.__get__(node)

    return node


def test_needs_target_update_on_new_cmd():
    node = _make_servo_node(last_applied=None, state=IDLE)
    TrajectoryTwin.servo_tick(node)
    assert node.planner.set_target.called
    assert node._last_applied_cmd_deg == [10.0, 20.0, 30.0, 40.0]
    assert node._state == RUNNING


def test_same_cmd_does_not_reset_target():
    node = _make_servo_node(
        last_applied=[10.0, 20.0, 30.0, 40.0], state=RUNNING)
    TrajectoryTwin.servo_tick(node)
    assert not node.planner.set_target.called
    assert node.planner.sync_state.called


def test_stopping_to_idle_clears_last_applied():
    node = _make_servo_node(
        enable=False, estop=True,
        last_applied=[10.0, 20.0, 30.0, 40.0],
        state=STOPPING, brake_finishes=True,
    )
    TrajectoryTwin.servo_tick(node)
    assert node._state == IDLE
    assert node._last_applied_cmd_deg is None


def test_bad_command_starts_braking():
    node = _make_servo_node(
        cmd_deg=[10.0, 20.0, 30.0, 999.0],
        last_received=[10.0, 20.0, 30.0, 999.0],
        last_applied=[10.0, 20.0, 30.0, 40.0],
        state=RUNNING,
    )
    TrajectoryTwin.servo_tick(node)
    assert node._state == STOPPING
    assert node.planner.start_brake.called
    # v4.0: rejection counter bumped for the offending joint
    assert node._stats["j4"]["above_max"] == 1


def test_estop_starts_braking():
    node = _make_servo_node(
        enable=True, estop=True, state=RUNNING,
        last_applied=[10.0, 20.0, 30.0, 40.0],
    )
    TrajectoryTwin.servo_tick(node)
    assert node._state == STOPPING
    assert node.planner.start_brake.called


def test_cmd_timeout_starts_braking():
    node = _make_servo_node(
        state=RUNNING, last_cmd_age_s=1.0,
        last_applied=[10.0, 20.0, 30.0, 40.0],
    )
    TrajectoryTwin.servo_tick(node)
    assert node._state == STOPPING
    assert node.planner.start_brake.called


def test_servo_tick_does_not_consume_tag():
    node = _make_servo_node(
        state=RUNNING,
        last_applied=[10.0, 20.0, 30.0, 40.0],
        cmd_tag="p;1;2;3;4", cmd_tag_t=12345.0,
    )
    TrajectoryTwin.servo_tick(node)
    assert node._publish.called
    call_args = node._publish.call_args[0]
    assert call_args[1] == "p;1;2;3;4"
    assert call_args[2] == 12345.0