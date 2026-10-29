"""Tests for the industrial patch. Stdlib + numpy only: python3 test_industrial_patch.py

They run the REAL patched methods (extracted with ast) against small stubs, so no
ROS / Isaac Sim is needed. NOT covered: the full tick() and Isaac stepping.
"""
import ast, os, sys, textwrap, threading, time, types
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _method(path, cls, name):
    src = open(os.path.join(HERE, path)).read()
    for node in ast.parse(src).body:
        if isinstance(node, ast.ClassDef) and node.name == cls:
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == name:
                    return textwrap.dedent(ast.get_source_segment(src, item))
    raise KeyError((path, cls, name))


def _bind(path, cls, name, **ns_extra):
    ns = {"time": time, "threading": threading, "np": np}
    ns.update(ns_extra)
    exec(_method(path, cls, name), ns)
    return ns[name]


BR = "plc_ros_bridge.py"
PH = "physics_twin.py"
DEAD, REFRESH = 0.001, 0.5


# ------------------------- bridge: act_ang policy -------------------------
def _act_stub():
    me = types.SimpleNamespace(_last_written_act_ang=None, _last_act_write_t=0.0)
    consts = dict(ACT_ANG_DEADBAND_DEG=DEAD, ACT_ANG_REFRESH_S=REFRESH)
    me.needs = lambda a, now: _bind(BR, "PlcRosBridge", "_act_ang_needs_write", **consts)(me, a, now)
    me.mark = lambda a, now: _bind(BR, "PlcRosBridge", "_act_ang_mark_written")(me, a, now)
    me.fail = lambda: _bind(BR, "PlcRosBridge", "_act_ang_write_failed")(me)
    return me


def test_first_write_always_happens():
    me = _act_stub()
    assert me.needs([1.0, 2.0], 100.0) is True


def test_unchanged_angles_are_skipped_inside_refresh_window():
    me = _act_stub()
    me.mark([1.0, 2.0], 100.0)
    assert me.needs([1.0, 2.0], 100.1) is False
    assert me.needs([1.0 + DEAD / 2, 2.0], 100.1) is False       # inside deadband
    assert me.needs([1.0 + 5 * DEAD, 2.0], 100.1) is True        # real change


def test_unchanged_angles_are_rewritten_after_refresh_period():
    me = _act_stub()
    me.mark([1.0, 2.0], 100.0)
    assert me.needs([1.0, 2.0], 100.0 + REFRESH + 0.01) is True


def test_failed_write_is_retried_next_tick_even_if_angles_unchanged():
    me = _act_stub()
    me.mark([1.0, 2.0], 100.0)
    assert me.needs([1.0, 2.0], 100.01) is False                 # normally skipped
    me.fail()                                                    # write failed
    assert me.needs([1.0, 2.0], 100.02) is True                  # ...but now retried


# ------------------------- bridge: estop lock -------------------------
class _Pub:
    def __init__(self): self.sent = []
    def publish(self, m): self.sent.append(m.data)


class _Bool:
    def __init__(self, data=False): self.data = data


def _lock_stub():
    me = types.SimpleNamespace(pub_estop=_Pub(), _estop_lock=threading.Lock())
    pub_locked = _bind(BR, "PlcRosBridge", "_pub_locked")
    publish_estop = _bind(BR, "PlcRosBridge", "_publish_estop", Bool=_Bool)
    me._pub_locked = lambda pub, msg: pub_locked(me, pub, msg)
    me.publish_estop = lambda v: publish_estop(me, v)
    return me


def test_estop_publish_goes_through_lock_and_sends():
    me = _lock_stub()
    me.publish_estop(True); me.publish_estop(False)
    assert me.pub_estop.sent == [True, False]


def test_estop_is_never_blocked_by_a_busy_lock():
    me = _lock_stub()
    me._estop_lock.acquire()                    # another thread "holds" the lock
    t0 = time.monotonic()
    me.publish_estop(True)
    dt = time.monotonic() - t0
    assert me.pub_estop.sent == [True], "E-stop must still be published"
    assert dt < 0.6, f"waited too long: {dt:.2f}s"
    me._estop_lock.release()


# ------------------------- bridge: deadline accounting -------------------------
def test_io_loop_counts_deadline_misses_and_overrun():
    me = types.SimpleNamespace(
        _io_stop=threading.Event(), _poll_period=0.01, _deadline_miss=0,
        _max_overrun_ms=0.0, log=types.SimpleNamespace(error=lambda *a, **k: None))
    calls = []
    def tick():
        calls.append(1)
        time.sleep(0.03)                        # takes 3x the period
        if len(calls) >= 4:
            me._io_stop.set()
    me.tick = tick
    _bind(BR, "PlcRosBridge", "_io_loop")(me)
    assert me._deadline_miss >= 3, me._deadline_miss
    assert me._max_overrun_ms >= 15.0, me._max_overrun_ms
    stats = _bind(BR, "PlcRosBridge", "_deadline_stats")(me)
    assert stats["deadline_miss"] >= 3 and stats["max_overrun_ms"] >= 15.0
    assert me._max_overrun_ms == 0.0            # window max resets, count is cumulative
    assert me._deadline_miss >= 3


def test_io_loop_no_miss_when_tick_is_fast():
    me = types.SimpleNamespace(
        _io_stop=threading.Event(), _poll_period=0.02, _deadline_miss=0,
        _max_overrun_ms=0.0, log=types.SimpleNamespace(error=lambda *a, **k: None))
    n = []
    def tick():
        n.append(1)
        if len(n) >= 5: me._io_stop.set()
    me.tick = tick
    _bind(BR, "PlcRosBridge", "_io_loop")(me)
    assert me._deadline_miss == 0


# ------------------------- physics: E-stop hold -------------------------
class _Log:
    def warn(self, *a, **k): pass
    def info(self, *a, **k): pass


def _phys_stub(current):
    n = len(current)
    mapper = types.SimpleNamespace(yaml_to_isaac=np.arange(n), dof_order=[f"j{i}" for i in range(n)])
    me = types.SimpleNamespace(
        mapper=mapper, timeout_s=0.5,
        _estop_active=False, _estop_hold_isaac=None, _seed_done=True,
        _latest_setpoint=None, _last_sp_t=0.0, _tag="", _tag_t=0.0,
        _applied_tag="", _applied_tag_t=0.0, _fresh_this_frame=False,
        _current_isaac=np.array(current, dtype=np.float32),
        _target_isaac=np.zeros(n, dtype=np.float32),
        _last_valid_isaac=np.array([9.0] * n, dtype=np.float32),   # OLD target before the stop
        _last_valid_yaml=np.array([9.0] * n, dtype=np.float32),
        _target_yaml=np.zeros(n, dtype=np.float32),
        _refresh_ok=True, get_logger=lambda: _Log())
    def refresh():
        return me._refresh_ok
    me._refresh_current = refresh
    me.build = lambda: _bind(PH, "PhysicsTwin", "_build_target", np=np)(me)
    me.on_estop = lambda v: _bind(PH, "PhysicsTwin", "on_estop", Bool=object)(me, types.SimpleNamespace(data=v))
    return me


def test_estop_target_is_latched_once_not_following_the_arm():
    me = _phys_stub([0.5, -0.5])
    me.on_estop(True)
    assert me.build() is True
    assert np.allclose(me._target_isaac, [0.5, -0.5])
    me._current_isaac[:] = [0.49, -0.52]            # arm sags under gravity
    me.build()
    assert np.allclose(me._target_isaac, [0.5, -0.5]), "target must NOT follow the sag"
    me._current_isaac[:] = [0.40, -0.70]
    me.build()
    assert np.allclose(me._target_isaac, [0.5, -0.5])


def test_after_estop_clears_arm_holds_stop_position_not_old_target():
    me = _phys_stub([0.5, -0.5])
    me.on_estop(True); me.build()
    me.on_estop(False)                              # cleared, no new setpoint yet
    me.build()
    assert np.allclose(me._target_isaac, [0.5, -0.5]), me._target_isaac
    assert not np.allclose(me._target_isaac, [9.0, 9.0]), "must not jump to the old target"


def test_estop_refresh_failure_does_not_latch_garbage():
    me = _phys_stub([0.5, -0.5])
    me._refresh_ok = False
    me.on_estop(True)
    assert me.build() is False and me._estop_hold_isaac is None
    me._refresh_ok = True
    assert me.build() is True and np.allclose(me._estop_hold_isaac, [0.5, -0.5])


def test_second_estop_latches_a_new_position():
    me = _phys_stub([0.5, -0.5])
    me.on_estop(True); me.build(); me.on_estop(False)
    me._current_isaac[:] = [1.0, 1.0]
    me.on_estop(True); me.build()
    assert np.allclose(me._target_isaac, [1.0, 1.0])


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
