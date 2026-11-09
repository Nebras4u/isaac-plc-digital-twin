"""Tests for the latency-probe tag chain. Stdlib only."""
import ast
import csv
import os
import subprocess
import sys
import tempfile
import textwrap
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _src(path):
    return open(os.path.join(HERE, path)).read()


def _extract(path, cls_name, func_name=None):
    tree = ast.parse(_src(path))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == cls_name:
            if func_name is None:
                return ast.get_source_segment(_src(path), node)
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == func_name:
                    return textwrap.dedent(
                        ast.get_source_segment(_src(path), item))
    raise KeyError((path, cls_name, func_name))


def _load_probe():
    ns = {"csv": csv, "time": time}
    exec(_extract("plc_ros_bridge.py", "_LatencyProbe"), ns)
    return ns["_LatencyProbe"]


def _bind_physics_v4_helpers(me):
    """v4.0: bind _bump / _warn / _check_finite to a PhysicsTwin mock.

    physics_twin.publish_states now calls self._check_finite() before
    publishing, and self._bump()/self._warn() on rejection. These are
    defined on the real class; tests use SimpleNamespace, so we bind
    them explicitly.
    """
    from collections import defaultdict
    me._stats = defaultdict(lambda: defaultdict(int))
    me._warned = {}
    me._warn_cooldown = 5.0
    for m in ("_bump", "_warn", "_check_finite"):
        src = _extract("physics_twin.py", "PhysicsTwin", m)
        local = {"np": np, "time": time}
        exec(src, local)
        setattr(me, m, local[m].__get__(me))


# Full 7-part tag (physics already appended js_pub)
T = [1000 + 10 * i for i in range(8)] + [0]


def tag7(origin=1000):
    """Complete tag after all 3 nodes appended their timestamps (7 parts):
       p;origin;read_start;read_done;cmd_pub;sp_pub;js_pub"""
    return f"p;{origin};1010;1020;1030;1040;1050"


def tag6(origin=1000):
    """Tag as it exists between traj and physics (6 parts):
       p;origin;read_start;read_done;cmd_pub;sp_pub"""
    return f"p;{origin};1010;1020;1030;1040"


# ---------------------------------------------------------------------------
# Probe tests
# ---------------------------------------------------------------------------
def test_probe_records_clean_row():
    P = _load_probe()
    with tempfile.TemporaryDirectory() as d:
        p = P(os.path.join(d, "x.csv"))
        assert p.record(tag7(), 1060, 1070) is True
        p.close()
        rows = list(csv.reader(open(os.path.join(d, "x.csv"))))
    assert rows[0] == P.COLS and len(rows) == 2
    assert rows[1] == [str(x) for x in T]
    assert p.rows == 1 and p.rejected == 0 and p.order_violations == 0


def test_probe_dedupes_same_command():
    P = _load_probe()
    with tempfile.TemporaryDirectory() as d:
        p = P(os.path.join(d, "x.csv"))
        assert p.record(tag7(), 1060, 1070) is True
        assert p.record(tag7(), 1061, 1071) is False
        assert p.record(tag7(2000), 2060, 2070) is True
    assert p.rows == 2


def test_probe_rejects_short_tags():
    P = _load_probe()
    with tempfile.TemporaryDirectory() as d:
        p = P(os.path.join(d, "x.csv"))
        assert p.record("p;1;2;3;4", 5, 6) is False
        assert p.record("x;1;2;3;4;5;6", 7, 8) is False
        assert p.rejected == 2


def test_bridge_untagged_js_does_not_erase_pending_tag():
    fn = _extract("plc_ros_bridge.py", "PlcRosBridge", "on_joint_states")
    ns = {"time": time}
    exec(fn, ns)
    on_js = ns["on_joint_states"]

    class Lock:
        def __enter__(self): return self
        def __exit__(self, *a): return False

    me = types.SimpleNamespace(
        _js_lock=Lock(), _probe=object(), _js_tag="", _js_recv_ns=0,
        joint_idx={"j1": 0}, _js_rad=[0.0], _last_js_t=0.0, _js_ok=False,
    )

    def msg(tag):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id=tag),
            name=["j1"], position=[0.5],
        )

    on_js(me, msg(tag7()))
    assert me._js_tag == tag7()
    on_js(me, msg(""))
    assert me._js_tag == tag7()


# ---------------------------------------------------------------------------
# physics_twin.publish_states
# ---------------------------------------------------------------------------
def _physics_publish(last_sp_age_s):
    fn = _extract("physics_twin.py", "PhysicsTwin", "publish_states")
    ns = {
        "time": time,
        "np": np,
        "TAG_VALID_S": 0.5,
        "STATUS_LOG_PERIOD_S": 10.0,
        "JointState": None,
    }

    class Hdr:
        stamp = None
        frame_id = ""

    class JS:
        def __init__(self):
            self.header = Hdr()
            self.name = None
            self.position = None
            self.velocity = None
            self.effort = None

    ns["JointState"] = JS
    exec(fn, ns)

    sent = []

    class Pub:
        def publish(self, m):
            sent.append(m.header.frame_id)

    class Robot:
        def get_joint_positions(self):
            return [np.array([0.1], dtype=np.float32)]

        def get_joint_velocities(self):
            return [np.array([0.0], dtype=np.float32)]

    class Clock:
        def now(self):
            return types.SimpleNamespace(to_msg=lambda: None)

    class Logger:
        def info(self, *a, **k): pass
        def warn(self, *a, **k): pass

    me = types.SimpleNamespace(
        robot=Robot(),
        get_clock=lambda: Clock(),
        get_logger=lambda: Logger(),
        js_pub=Pub(),
        mapper=types.SimpleNamespace(
            dof_order=["j1"],
            yaml_to_isaac=np.array([0], dtype=np.int32),
        ),
        _applied_tag=tag6(),                                  # 6 parts (from traj)
        _applied_tag_t=time.monotonic() - last_sp_age_s,
        _tag=tag6(),
        _tag_t=time.monotonic() - last_sp_age_s,
        _last_sp_t=time.monotonic() - last_sp_age_s,
        _estop_active=False,
        _needs_reinit=False,
        _last_pub_t=0.0,
        _last_status_log_t=time.monotonic(),
        publish_hz=1_000_000.0,
        _current_isaac=np.zeros(1, dtype=np.float32),
        _velocity_isaac=np.zeros(1, dtype=np.float32),
    )

    _bind_physics_v4_helpers(me)
    return ns["publish_states"], me, sent


def test_physics_keeps_tag_across_multiple_publishes():
    publish, me, sent = _physics_publish(0.01)
    for _ in range(3):
        me._last_pub_t = 0.0
        publish(me)
    assert all(s.startswith("p;") for s in sent), sent
    # tag6 (6 parts) + physics timestamp = 7 parts (6 semicolons)
    assert all(s.count(";") == 6 for s in sent), sent
    assert me._applied_tag != ""


def test_physics_drops_tag_after_timeout():
    publish, me, sent = _physics_publish(10.0)
    publish(me)
    assert sent[0] == ""
    assert me._applied_tag == ""


def test_physics_refuses_non_finite_joint_states():
    """v4.0: publish_states must not publish when positions are non-finite."""
    publish, me, sent = _physics_publish(0.01)
    me.robot.get_joint_positions = lambda: [np.array([float("nan")],
                                                    dtype=np.float32)]
    publish(me)
    assert sent == []
    assert me._stats["__joint_states__"]["pos_non_finite"] >= 1


# ---------------------------------------------------------------------------
# trajectory_twin._publish
# ---------------------------------------------------------------------------
def _bind_traj_v4_helpers(me):
    """v4.0: bind _bump / _warn to a TrajectoryTwin mock."""
    from collections import defaultdict
    me._stats = defaultdict(lambda: defaultdict(int))
    me._warned = {}
    me._warn_cooldown = 5.0
    for m in ("_bump", "_warn"):
        src = _extract("trajectory_twin.py", "TrajectoryTwin", m)
        local = {"time": time}
        exec(src, local)
        setattr(me, m, local[m].__get__(me))


def test_trajectory_publish_keeps_6_part_tag():
    fn = _extract("trajectory_twin.py", "TrajectoryTwin", "_publish")
    import threading
    ns = {"time": time, "math": __import__("math")}
    exec(fn, ns)

    sent = []

    class Pub:
        def publish(self, m): sent.append(m.header.frame_id)

    class Hdr:
        stamp = None
        frame_id = ""

    class Msg:
        def __init__(self):
            self.header = Hdr()
            self.position = [0.0]

    class Clock:
        def now(self):
            return types.SimpleNamespace(to_msg=lambda: None)

    class Logger:
        def info(self, *a, **k): pass

    me = types.SimpleNamespace(
        _msg=Msg(),
        get_clock=lambda: Clock(),
        log=Logger(),
        setpoint_pub=Pub(),
        _last_setpoint_log=time.monotonic(),
        ctrl_indices=[0],
        all_dof=1,
        controlled_joints=["j1"],
        lock=threading.Lock(),
        _cmd_tag=tag6(),
        _cmd_tag_t=12345.0,
    )
    _bind_traj_v4_helpers(me)
    ns["_publish"](me, [0.5], tag6(), 12345.0)
    assert sent[0].startswith(tag6() + ";")
    # traj: input tag has 6 parts (5 ";" ), traj appends 1 → 6 ";" / 7 parts
    assert sent[0].count(";") == 6


def test_trajectory_publish_rejects_non_finite_positions():
    """v4.0: _publish must not emit /servo_setpoints with NaN/Inf."""
    fn = _extract("trajectory_twin.py", "TrajectoryTwin", "_publish")
    import threading
    ns = {"time": time, "math": __import__("math")}
    exec(fn, ns)

    sent = []

    class Pub:
        def publish(self, m): sent.append(m.header.frame_id)

    class Hdr:
        stamp = None
        frame_id = ""

    class Msg:
        def __init__(self):
            self.header = Hdr()
            self.position = [0.0]

    class Clock:
        def now(self):
            return types.SimpleNamespace(to_msg=lambda: None)

    class Logger:
        def info(self, *a, **k): pass

    me = types.SimpleNamespace(
        _msg=Msg(),
        get_clock=lambda: Clock(),
        log=Logger(),
        setpoint_pub=Pub(),
        _last_setpoint_log=time.monotonic(),
        ctrl_indices=[0],
        all_dof=1,
        controlled_joints=["j1"],
        lock=threading.Lock(),
        _cmd_tag=tag6(),
        _cmd_tag_t=12345.0,
    )
    _bind_traj_v4_helpers(me)
    ns["_publish"](me, [float("nan")], tag6(), 12345.0)
    assert sent == []
    assert me._stats["__publish__"]["non_finite"] >= 1


# ---------------------------------------------------------------------------
# analyze_probe.py smoke
# ---------------------------------------------------------------------------
def test_analyze_script_smoke():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "p.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["origin_ns", "read_start_ns", "read_done_ns",
                        "cmd_pub_ns", "sp_pub_ns", "js_pub_ns",
                        "js_recv_ns", "write_done_ns", "hb_counter_val"])
            base = 10_000_000_000
            for i in range(200):
                o = base + i * 16_000_000
                w.writerow([o, o+2_000_000, o+8_000_000, o+8_100_000,
                            o+14_000_000, o+25_000_000, o+26_000_000,
                            o+34_000_000, i])
        out = subprocess.run(
            [sys.executable, os.path.join(HERE, "analyze_probe.py"), path],
            capture_output=True, text=True,
        )
    assert out.returncode == 0, out.stderr
    assert "rows: 200" in out.stdout
    assert "cmd_to_feedback" in out.stdout


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = 0
    for n, f in tests:
        try:
            f()
            print("PASS", n)
        except Exception as e:
            failed += 1
            print("FAIL", n, "->", repr(e))
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)