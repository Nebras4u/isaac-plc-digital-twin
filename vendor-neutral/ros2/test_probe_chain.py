"""Tests for the latency-probe tag chain. Stdlib only (run: python3 test_probe_chain.py).

They execute the REAL methods from the patched files (extracted with ast) against
small stubs, so no ROS / Isaac Sim is needed. The tick() write path inside the
bridge is not covered here (it needs the whole node).
"""
import ast, csv, os, subprocess, sys, tempfile, textwrap, time, types

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
                    return textwrap.dedent(ast.get_source_segment(_src(path), item))
    raise KeyError((path, cls_name, func_name))


def _load_probe():
    ns = {"csv": csv, "time": time}
    exec(_extract("plc_ros_bridge.py", "_LatencyProbe"), ns)
    return ns["_LatencyProbe"]


T = [1000 + 10 * i for i in range(8)]       # a clean, increasing chain
def tag7(origin=1000):
    return f"p;{origin};1010;1020;1030;1040;1050"


def test_probe_records_clean_row():
    P = _load_probe()
    with tempfile.TemporaryDirectory() as d:
        p = P(os.path.join(d, "x.csv"))
        assert p.record(tag7(), 1060, 1070) is True
        p.close()
        rows = list(csv.reader(open(os.path.join(d, "x.csv"))))
    assert rows[0] == P.COLS and len(rows) == 2 and rows[1] == [str(x) for x in T]
    assert p.rows == 1 and p.rejected == 0 and p.order_violations == 0


def test_probe_dedupes_same_command():
    P = _load_probe()
    with tempfile.TemporaryDirectory() as d:
        p = P(os.path.join(d, "x.csv"))
        assert p.record(tag7(), 1060, 1070) is True
        assert p.record(tag7(), 1061, 1071) is False     # same origin -> once only
        assert p.record(tag7(2000), 2060, 2070) is True
    assert p.rows == 2


def test_probe_rejects_short_tags_and_trims_long_ones():
    P = _load_probe()
    with tempfile.TemporaryDirectory() as d:
        p = P(os.path.join(d, "x.csv"))
        assert p.record("p;1;2;3;4", 5, 6) is False      # only 5 parts
        assert p.record("x;1;2;3;4;5;6", 7, 8) is False  # not a probe tag
        assert p.record("p;a;2;3;4;5;6", 7, 8) is False  # not integers
        assert p.rejected == 3
        assert p.record(tag7() + ";9999;8888", 1060, 1070) is True   # extra parts trimmed
    assert p.rows == 1


def test_probe_counts_order_violations_but_still_writes():
    P = _load_probe()
    with tempfile.TemporaryDirectory() as d:
        p = P(os.path.join(d, "x.csv"))
        assert p.record(tag7(), 1060, 1055) is True       # write_done < js_recv
    assert p.order_violations == 1 and p.rows == 1


def test_bridge_untagged_joint_state_does_not_erase_pending_tag():
    fn = _extract("plc_ros_bridge.py", "PlcRosBridge", "on_joint_states")
    ns = {"time": time}
    exec(fn, ns)
    on_js = ns["on_joint_states"]
    class Lock:
        def __enter__(self): return self
        def __exit__(self, *a): return False
    me = types.SimpleNamespace(_js_lock=Lock(), _probe=object(), _js_tag="", _js_recv_ns=0,
                               joint_idx={"j1": 0}, _js_rad=[0.0], _last_js_t=0.0, _js_ok=False)
    msg = lambda tag: types.SimpleNamespace(header=types.SimpleNamespace(frame_id=tag),
                                            name=["j1"], position=[0.5])
    on_js(me, msg(tag7()))
    assert me._js_tag == tag7() and me._js_recv_ns > 0
    on_js(me, msg(""))                                    # frame without tag
    assert me._js_tag == tag7(), "empty frame must not erase the pending tag"
    assert me._js_ok and me._js_rad == [0.5]


def _traj_publish():
    fn = _extract("trajectory_twin.py", "TrajectoryTwin", "publish")
    ns = {"time": time, "math": __import__("math")}
    exec(fn, ns)
    sent = []
    class Pub:
        def publish(self, m): sent.append(m.header.frame_id)
    class Hdr: stamp = None; frame_id = ""
    class Msg: header = Hdr(); position = None
    class Clock:
        def now(self): return types.SimpleNamespace(to_msg=lambda: None)
    me = types.SimpleNamespace(_msg=Msg(), get_clock=lambda: Clock(), setpoint_pub=Pub(),
                               _cmd_tag="", _last_setpoint_log=time.monotonic(),
                               controlled_joints=[], name_to_idx={})
    class Pos:
        def tolist(self): return [0.0]
    return ns["publish"], me, sent, Pos()


def test_trajectory_tag_travels_exactly_once():
    publish, me, sent, pos = _traj_publish()
    me._cmd_tag = tag7()
    publish(me, pos)
    publish(me, pos)       # e.g. braking / next tick without a new command
    publish(me, pos)
    assert sent[0].startswith(tag7() + ";") and sent[0].count(";") == 7
    assert sent[1] == "" and sent[2] == ""


def _physics_publish(last_sp_age_s):
    fn = _extract("physics_twin.py", "PhysicsTwin", "publish_states")
    ns = {"time": time, "TAG_VALID_S": 0.5, "DOF_ORDER": ["j1"], "JointState": None}
    class Hdr: stamp = None; frame_id = ""
    class JS:
        def __init__(self): self.header = Hdr(); self.name = None; self.position = None
    ns["JointState"] = JS
    exec(fn, ns)
    sent = []
    class Pub:
        def publish(self, m): sent.append(m.header.frame_id)
    class Robot:
        def get_joint_positions(self): return [[0.1]]
    class Clock:
        def now(self): return types.SimpleNamespace(to_msg=lambda: None)
    me = types.SimpleNamespace(robot=Robot(), get_clock=lambda: Clock(), js_pub=Pub(),
                               _applied_tag=tag7(), _tag=tag7(),
                               _last_sp_t=time.monotonic() - last_sp_age_s,
                               _pub_count=0, _needs_reinit=False)
    return ns["publish_states"], me, sent


def test_physics_attaches_fresh_tag_once_then_clears_everything():
    publish, me, sent = _physics_publish(0.01)
    publish(me); publish(me); publish(me)
    assert sent[0].startswith(tag7() + ";") and sent[0].count(";") == 7
    assert sent[1] == "" and sent[2] == ""
    assert me._applied_tag == "" and me._tag == ""


def test_physics_never_publishes_stale_tag():
    publish, me, sent = _physics_publish(5.0)            # setpoint 5 s old
    publish(me)
    assert sent == [""]


def test_analyze_script_on_synthetic_csv():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "p.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["origin_ns","read_start_ns","read_done_ns","cmd_pub_ns","sp_pub_ns",
                        "js_pub_ns","js_recv_ns","write_done_ns"])
            base = 10_000_000_000
            for i in range(200):
                o = base + i * 16_000_000
                w.writerow([o, o+2_000_000, o+8_000_000, o+8_100_000, o+14_000_000,
                            o+25_000_000, o+26_000_000, o+34_000_000])
        out = subprocess.run([sys.executable, os.path.join(HERE, "analyze_probe.py"), path],
                             capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    assert "rows: 200" in out.stdout and "WARNING" not in out.stdout
    assert "cmd_to_feedback" in out.stdout and "isaac_step" in out.stdout


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = 0
    for n, f in tests:
        try:
            f(); print("PASS", n)
        except Exception as e:
            failed += 1; print("FAIL", n, "->", repr(e))
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
