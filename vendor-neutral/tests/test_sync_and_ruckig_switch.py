import importlib.util, json, logging, math, random, socket, struct, sys, threading, types
from types import SimpleNamespace as NS
import numpy as np

import os
WORK = os.path.dirname(os.path.abspath(__file__))


def mod(name, **a):
    m = types.ModuleType(name); m.__dict__.update(a); sys.modules[name] = m; return m


# ---------------------------------------------------------------- stubs
class Node:
    def __init__(self, name): pass
    def destroy_node(self): pass
class _E: BEST_EFFORT = 1; TRANSIENT_LOCAL = 2
class QoSProfile:
    def __init__(self, **kw): pass
class Msg:
    def __init__(self, data=None): self.data = data
class JointState:
    def __init__(self): self.name = []; self.position = []; self.velocity = []; self.header = NS(frame_id="", stamp=None)
mod("rclpy"); mod("rclpy.executors"); mod("rclpy.node", Node=Node)
mod("rclpy.qos", QoSProfile=QoSProfile, QoSDurabilityPolicy=_E, QoSReliabilityPolicy=_E)
mod("sensor_msgs"); mod("sensor_msgs.msg", JointState=JointState)
mod("std_msgs"); mod("std_msgs.msg", Bool=Msg, Float64=Msg, Int32=Msg, String=Msg)
mod("bridge_config", load_bridge_config=lambda p: None, ConfigError=Exception, CURRENT_SCHEMA_VERSION="2.2")
mod("bridge_logger", setup_logging=lambda *a, **k: logging.getLogger("x"))
mod("safety", ESTOP_TOPIC="/plc/emergency_stop", resolve_estop=lambda v: v)

# python-snap7 util semantics (S7 is big-endian; bit 0 = LSB)
def get_bool(d, i, b): return bool((d[i] >> b) & 1)
def set_bool(d, i, b, v):
    d[i] = (d[i] | (1 << b)) if v else (d[i] & ~(1 << b) & 0xFF)
util = mod("snap7.util", get_bool=get_bool, set_bool=set_bool,
           set_int=lambda d, i, v: struct.pack_into(">h", d, i, v),
           set_dint=lambda d, i, v: struct.pack_into(">i", d, i, v),
           set_lreal=lambda d, i, v: struct.pack_into(">d", d, i, v),
           get_int=lambda d, i: struct.unpack_from(">h", d, i)[0],
           get_dint=lambda d, i: struct.unpack_from(">i", d, i)[0],
           get_lreal=lambda d, i: struct.unpack_from(">d", d, i)[0])
mod("snap7", util=util)
sys.modules["snap7"].util = util
mod("ruckig", Ruckig=lambda *a, **k: None, InputParameter=lambda *a, **k: NS(), OutputParameter=lambda *a, **k: NS(),
    Result=NS(Working=0, Finished=1), ControlInterface=NS(Position=0, Velocity=1))
mod("asyncua")
class _Variant:
    def __init__(self, value, vtype=None, is_array=False): self.value, self.vtype = value, vtype
class _DV:
    def __init__(self, v): self.v = v
sys.modules["asyncua"].ua = NS(Variant=_Variant, DataValue=_DV,
                               VariantType=NS(Boolean=1, Int32=2, Double=3, String=4))
mod("bridge_logger_ok")


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m

br = load("bridge_v46", f"{WORK}/plc_ros_bridge.py")
ok = 0
def check(cond, msg):
    global ok
    assert cond, msg
    ok += 1

# ------------------------------------------------ 1) base sync contract
class Fake(br.PlcDriver):
    def __init__(self): self.log = []; self.store = {"a": 1.0}; self._last_sock_ms = 0.0
    def connect(self): pass
    def read(self, addrs): self.log.append(("read", list(addrs))); return {a: self.store.get(a) for a in addrs}
    def write(self, values): self.log.append(("write", dict(values))); self.store.update(values); return ["bad"] if "bad" in values else []
d = Fake()
raw, failed = d.sync(["a"], {"a": 2.0, "bad": 1})
check(raw == {"a": 1.0}, "read must see the value from BEFORE the write")
check(failed == ["bad"] and d.store["a"] == 2.0, "failed list / write applied")
check([x[0] for x in d.log] == ["read", "write"], "order read -> write")
d.log.clear(); d.sync(["a"], {})
check([x[0] for x in d.log] == ["read"], "no write call when nothing to write")
check(d._last_sock_ms >= 0.0, "_last_sock_ms updated")
check(br.PlcDriver.requests_per_sync == 2 and br.TcpJsonDriver.requests_per_sync == 1, "request counts")
print("1 base sync contract OK")

# ---------------------------------- 2) tcp_json: really ONE request per sync
class Srv(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.s = socket.socket(); self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(("127.0.0.1", 0)); self.s.listen(1); self.port = self.s.getsockname()[1]
        self.tags = {"x": 1.0, "y": 2.0}; self.reqs = []
    def run(self):
        c, _ = self.s.accept(); buf = b""; seq = 0
        while True:
            data = c.recv(4096)
            if not data: break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1); req = json.loads(line); self.reqs.append(req["cmd"]); seq += 1
                if req["cmd"] == "sync":   # same order as plc_server_win.py: read, then write
                    vals = {n: self.tags.get(n) for n in req.get("read", [])}
                    w = req.get("write", {}); self.tags.update(w)
                    rep = {"values": vals, "written": {k: True for k in w}}
                elif req["cmd"] == "read":
                    rep = {"values": {n: self.tags.get(n) for n in req["tags"]}}
                else:
                    self.tags.update(req["tags"]); rep = {"written": {k: True for k in req["tags"]}}
                rep["seq"] = seq
                c.sendall((json.dumps(rep) + "\n").encode())
srv = Srv(); srv.start()
tj = br.TcpJsonDriver(NS(host="127.0.0.1", port=srv.port, timeout=2.0))
tj.set_signal_types({"x": "float", "y": "float"}); tj.connect()
raw, failed = tj.sync(["x", "y"], {"x": 5.0})
check(srv.reqs == ["sync"], f"tcp_json must use exactly one request, got {srv.reqs}")
check(raw == {"x": 1.0, "y": 2.0} and failed == [] and srv.tags["x"] == 5.0, "tcp_json read-before-write")
tj.close()
print("2 tcp_json one-request sync OK")

# ------------------------------------------------ 3) OPC UA back-to-back
class FakeUa:
    def __init__(self): self.calls = []; self.vals = {"n1": 1.5}
    def get_node(self, a): return a
    def read_values(self, nodes): self.calls.append("read"); return [self.vals[n] for n in nodes]
    def write_values(self, nodes, variants):
        self.calls.append("write"); [self.vals.__setitem__(n, v.value) for n, v in zip(nodes, variants)]
        return [NS(is_good=lambda: True) for _ in nodes]
ua = br.OpcUaDriver(NS(endpoint="x", timeout=1, user=None, password=None))
ua.client = FakeUa(); ua.set_signal_types({"n1": "float"})
raw, failed = ua.sync(["n1"], {"n1": 9.0})
check(ua.client.calls == ["read", "write"] and raw == {"n1": 1.5} and failed == [], "opcua read->write")
check(br.OpcUaDriver.requests_per_sync == 2, "opcua requests_per_sync")
print("3 opcua sync OK")

# ------------------------------------------------ 4) snap7 coalescing
class FakeS7:
    def __init__(self, dbs): self.mem = {k: bytearray(v) for k, v in dbs.items()}; self.writes = 0; self.reads = 0; self.fail_at = None
    def db_read(self, db, off, n): self.reads += 1; return bytearray(self.mem[db][off:off + n])
    def db_write(self, db, off, data):
        if self.fail_at == (db, off): raise IOError("boom")
        self.writes += 1; self.mem[db][off:off + len(data)] = data

SIZES = {"int16": 2, "int32": 4, "lreal": 8}
def mk_driver(dbs, meta):
    s = br.Snap7Driver(NS(host="h", rack=0, slot=1, timeout=1.0))
    s.client = FakeS7(dbs); s._addr_meta = meta; return s

def naive(s, values):
    for a, v in values.items(): s._write_one(s._addr_meta[a], v)

rnd = random.Random(7)
TYPE_OF = {"bool": "bool", "int16": "int", "int32": "int", "lreal": "float"}
for trial in range(300):
    meta, values = {}, {}
    occ = {1: {}, 2: {}}            # db -> {byte: "num" | set(bits)}
    for i in range(rnd.randint(1, 14)):
        db = rnd.choice([1, 2]); st = rnd.choice(["bool", "int16", "int32", "lreal"])
        off = rnd.randint(0, 40); bit = 0
        if st == "bool":
            bit = rnd.randint(0, 7); cur = occ[db].get(off)
            if cur == "num" or (isinstance(cur, set) and bit in cur): continue
            occ[db].setdefault(off, set()).add(bit)
            val = rnd.choice([True, False])
        else:
            size = SIZES[st]
            if any(k in occ[db] for k in range(off, off + size)): continue
            for k in range(off, off + size): occ[db][k] = "num"
            val = rnd.randint(-1000, 1000) if st != "lreal" else rnd.uniform(-90, 90)
        a = f"s{i}"
        meta[a] = {"db": db, "byte": off, "bit": bit, "type": TYPE_OF[st], "s7_type": st}
        values[a] = val
    if not meta: continue
    init = {k: bytearray(rnd.randrange(256) for _ in range(64)) for k in (1, 2)}
    s_new, s_old = mk_driver(init, meta), mk_driver(init, meta)
    f_new = s_new.write(values); naive(s_old, values)
    check(f_new == [], "no failures expected")
    check(s_new.client.mem == s_old.client.mem, f"memory differs in trial {trial}")
    check(s_new.client.writes <= s_old.client.writes, "coalescing must not add writes")
# adjacent act_ang array = ONE db_write
meta = {f"j{i}": {"db": 3, "byte": 8 * i, "bit": 0, "type": "float", "s7_type": "lreal"} for i in range(4)}
s = mk_driver({3: bytearray(64)}, meta); s.write({f"j{i}": float(i) for i in range(4)})
check(s.client.writes == 1, "4 contiguous LREAL -> one db_write")
# bools of one byte -> one RMW + one write
meta = {"b0": {"db": 1, "byte": 0, "bit": 0, "type": "bool", "s7_type": "bool"},
        "b1": {"db": 1, "byte": 0, "bit": 1, "type": "bool", "s7_type": "bool"},
        "w":  {"db": 1, "byte": 1, "bit": 0, "type": "int", "s7_type": "int16"}}
s = mk_driver({1: bytearray(b"\xf0\x00\x00\x00")}, meta); s.write({"b0": True, "b1": True, "w": 7})
check(s.client.reads == 1 and s.client.writes == 1 and s.client.mem[1][0] == 0xF3, "bool bits share one RMW and merge with neighbours")
# failure isolation
meta = {"p": {"db": 1, "byte": 0, "bit": 0, "type": "float", "s7_type": "lreal"},
        "q": {"db": 2, "byte": 0, "bit": 0, "type": "float", "s7_type": "lreal"}}
s = mk_driver({1: bytearray(16), 2: bytearray(16)}, meta); s.client.fail_at = (2, 0)
check(s.write({"p": 1.0, "q": 2.0}) == ["q"], "only the failing run is reported")
# overlap (misconfig) never crashes and reports success
meta = {"o1": {"db": 1, "byte": 0, "bit": 0, "type": "float", "s7_type": "lreal"},
        "o2": {"db": 1, "byte": 4, "bit": 0, "type": "int", "s7_type": "int32"}}
s = mk_driver({1: bytearray(16)}, meta)
check(s.write({"o1": 1.0, "o2": 5}) == [], "overlap tolerated")
check(s.write({"zzz": 1}) == ["zzz"], "unknown address reported failed")
print("4 snap7 coalescing OK (300 random layouts identical to per-signal writes)")

# ------------------------------------------------ 5) PLC API tag-info cache
class Inst:
    def __init__(self): self.info_calls = 0; self.mem = {"a": 1.0, "b": True}
    def GetTagInfo(self, a): self.info_calls += 1; return NS(DataType="LReal" if a == "a" else "Bool")
    def ReadDouble(self, a): return self.mem[a]
    def ReadBool(self, a): return self.mem[a]
    def WriteDouble(self, a, v): self.mem[a] = v
    def WriteBool(self, a, v): self.mem[a] = v
pa = br.PlcApiDriver(NS(instance="296")); pa.instance = Inst(); pa.set_signal_types({"a": "float", "b": "bool"})
for _ in range(5): raw, failed = pa.sync(["a", "b"], {"a": 3.0})
check(pa.instance.info_calls == 2, f"GetTagInfo should run once per tag, ran {pa.instance.info_calls}")
check(raw == {"a": 3.0, "b": True} or raw["b"] is True, "plc_api values")
print("5 plc_api cache OK")

# ------------------------------------------------ 6) bridge tick / units
src = open(f"{WORK}/plc_ros_bridge.py").read()
check("raw, failed = self.driver.sync(self._read_addrs, writes_to_send)" in src, "tick uses sync")
check('self.driver_name == "tcp_json":\n                raw' not in src, "no tcp_json special-case left")
check("math.radians(d) for d in ang_deg" not in src and "ang = ang_deg" in src, "cmd_ang is degrees again")
print("6 tick wiring OK")

# ------------------------------------------------ 7) ruckig.enabled
tw = load("twin_v41", f"{WORK}/trajectory_twin.py")
for bad in ("false", 0, None, "no", 1):
    try: tw.build_planner({"enabled": bad, "control_hz": 100}, 2); check(False, f"{bad!r} must be rejected")
    except ValueError: check(True, "")
for hz in (0, -5, 5000):
    try: tw.build_planner({"enabled": False, "control_hz": hz}, 2); check(False, "bad hz")
    except ValueError: check(True, "")
pp = tw.build_planner({"enabled": False, "control_hz": 200}, 3)
check(type(pp).__name__ == "PassthroughPlanner", "enabled:false -> PassthroughPlanner")
check(tw.build_planner({"enabled": True, "control_hz": 100}, 2).__class__.__name__ == "RuckigPlanner", "enabled:true -> RuckigPlanner")
check(tw.build_planner({"control_hz": 100}, 2).__class__.__name__ == "RuckigPlanner", "default -> RuckigPlanner")
print("7 build_planner OK")

# ---- 8) full servo_tick scenario with PassthroughPlanner
class Log:
    def __init__(self): self.ev = []
    def info(self, m, extra=None): self.ev.append(("info", m))
    def warning(self, m, extra=None): self.ev.append(("warning", m))
    def error(self, m, extra=None): self.ev.append(("error", m))
def make():
    from collections import defaultdict
    t = object.__new__(tw.TrajectoryTwin)
    t.lock = threading.Lock(); t.log = Log()
    t.all_joints = ["j1", "j2", "j3"]; t.controlled_joints = ["j1", "j2"]; t.ctrl_indices = [0, 1]; t.ctrl_dof = 2; t.all_dof = 3
    t.limits_deg = {"j1": [-90, 90], "j2": [-90, 90]}
    t._warn_cooldown = 0.0; t._stats = defaultdict(lambda: defaultdict(int)); t._warned = {}
    t._actual = np.array([0.1, 0.2, 0.3]); t._has_actual = True; t._last_js_t = 1e18
    t._enable = True; t._estop = False; t._cmd_deg = [30.0, -45.0]; t._last_received_cmd_deg = [None, None]
    t._last_applied_cmd_deg = None; t._cmd_ok = True; t._last_cmd_t = 1e18; t._state = tw.IDLE
    t._msg = JointState(); t._msg.name = list(t.all_joints); t._msg.position = [0.0] * 3; t._msg.velocity = [0.0] * 3
    t._ctrl_actual = np.zeros(2); t._last_setpoint_log = 0.0
    t._cmd_timeout_logged = t._no_feedback_logged = t._bad_cmd_logged = False
    t._cmd_tag = "p;1;2;3"; t._cmd_tag_t = 1e18
    t.published = []
    t.setpoint_pub = NS(publish=lambda m: t.published.append((list(m.position), m.header.frame_id)))
    t.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: "stamp"))
    t.planner = tw.build_planner({"enabled": False, "control_hz": 200}, 2)
    return t
import time as _t
t = make(); t._last_js_t = _t.monotonic() + 1e6; t._last_cmd_t = _t.monotonic() + 1e6
t.servo_tick()
check(t._state == tw.RUNNING, "enable + valid cmd -> RUNNING")
sp, tag = t.published[-1]
check(abs(sp[0] - math.radians(30.0)) < 1e-12 and abs(sp[1] - math.radians(-45.0)) < 1e-12, "setpoint == commanded target (no smoothing)")
check(tag.startswith("p;1;2;3"), "tag chain preserved")
t._cmd_deg = [200.0, 0.0]                      # outside limits -> brake, never publish the bad target
n = len(t.published); t.servo_tick()
check(t._state == tw.IDLE, "limit violation -> braked; passthrough stops in the same tick")
check(len(t.published) == n + 1, "exactly one hold setpoint")
sp = t.published[-1][0]
check(abs(sp[0] - 0.1) < 1e-12 and abs(sp[1] - 0.2) < 1e-12, "brake holds the MEASURED position")
check(("warning", "state_changed") in t.log.ev and ("info", "state_changed") in t.log.ev, "both transitions logged")
t.servo_tick(); check(t._state == tw.IDLE and len(t.published) == n + 1, "still-bad cmd publishes nothing")
t._cmd_deg = [10.0, 10.0]; t._estop = True; t.servo_tick()
check(t._state == tw.IDLE and len(t.published) == n + 1, "estop blocks RUNNING")
t._estop = False; t._enable = False; t.servo_tick()
check(t._state == tw.IDLE and len(t.published) == n + 1, "disable blocks RUNNING")
t._enable = True; t.servo_tick()
check(t._state == tw.RUNNING and abs(t.published[-1][0][0] - math.radians(10.0)) < 1e-12, "re-enable -> RUNNING at new target")
t._estop = True; m = len(t.published); t.servo_tick()
check(t._state == tw.IDLE and len(t.published) == m + 1 and abs(t.published[-1][0][0] - 0.1) < 1e-12, "estop while RUNNING -> hold measured position")
print("8 servo_tick with PassthroughPlanner OK")
print(f"\nALL GOOD ({ok} checks)")
