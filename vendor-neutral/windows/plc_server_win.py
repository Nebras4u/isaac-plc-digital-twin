# plc_server_win4.py
# Runs on Windows. Serves PLCSIM tags over TCP as JSON.
#
# v3.2 changes:
#   - FIXED: EPrimitiveDataType members are .NET primitive names
#     (Double, Int32, Int16, Bool), NOT Siemens names (LReal, DInt, Int).
#     _TYPE_NAMES now maps to the correct member names, so the self-test
#     passes and batch mode is actually enabled.
#   - BATCH can be toggled from outside without editing the file:
#         PowerShell:  $env:PLC_BATCH = "0"   # disable
#                      $env:PLC_BATCH = "1"   # enable (default)
#         cmd.exe:     set PLC_BATCH=0
#     Same for TIMING: $env:PLC_TIMING = "0" / set PLC_TIMING=0.
#
# Run with (PowerShell):
#     python .\plc_server_win4.py
#     # or, to disable batch in this shell:
#     $env:PLC_BATCH = "0"; python .\plc_server_win4.py

import os
import sys
import json
import time
import socket
import threading

import clr

# ---- Load PLCSIM API ----
API_DIR  = r"C:\Program Files (x86)\Common Files\Siemens\PLCSIMADV\API\6.0"
DLL_PATH = os.path.join(API_DIR, "Siemens.Simatic.Simulation.Runtime.Api.x64.dll")

if not os.path.exists(DLL_PATH):
    print(f"ERROR: DLL not found: {DLL_PATH}")
    sys.exit(1)

os.add_dll_directory(API_DIR)
clr.AddReference(DLL_PATH)

from Siemens.Simatic.Simulation.Runtime import (
    SimulationRuntimeManager,
    SimulationRuntimeException,
)

# ---- Connect ----
INSTANCE_NAME = os.environ.get("PLC_INSTANCE", "296")
instance = SimulationRuntimeManager.CreateInterface(INSTANCE_NAME)
print(f"Connected to PLCSIM: {instance.Name}, State: {instance.OperatingState}")

if str(instance.OperatingState) != "Run":
    instance.Run()
    print(f"After Run: {instance.OperatingState}")

instance.UpdateTagList()
print(f"Tag list updated. Total tags: {len(instance.TagInfos)}")


# ---- Tag map ----
DB = "MAIN_KUKA_KR210_L150_L"

BOOL  = "Bool"
LREAL = "LReal"
DINT  = "DInt"
INT   = "Int"

TAGS = {
    f"{DB}.MainRobotLeft_CmdAng[1]": LREAL,
    f"{DB}.MainRobotLeft_CmdAng[2]": LREAL,
    f"{DB}.MainRobotLeft_CmdAng[3]": LREAL,
    f"{DB}.MainRobotLeft_CmdAng[4]": LREAL,
    f"{DB}.MainRobotLeft_CmdEnable": BOOL,
    f"{DB}.MainRobotLeft_Vlcty":     LREAL,

    f"{DB}.MainRobotLeft_Emergency_Stop_Active": BOOL,
    f"{DB}.MainRobotLeft_Motion_Active":         BOOL,
    f"{DB}.MainRobotLeft_Group_Fault":           BOOL,
    f"{DB}.MainRobotLeft_Axis1_Fault":           BOOL,
    f"{DB}.MainRobotLeft_Axis2_Fault":           BOOL,
    f"{DB}.MainRobotLeft_Axis3_Fault":           BOOL,
    f"{DB}.MainRobotLeft_Axis4_Fault":           BOOL,

    f"{DB}.MainRobotLeft_ActAng[1]": LREAL,
    f"{DB}.MainRobotLeft_ActAng[2]": LREAL,
    f"{DB}.MainRobotLeft_ActAng[3]": LREAL,
    f"{DB}.MainRobotLeft_ActAng[4]": LREAL,
    f"{DB}.MainRobotLeft_CmdAck":    BOOL,

    f"{DB}.Heartbeat_Counter": DINT,
    f"{DB}.ROS_Ack_Counter":   DINT,
    f"{DB}.ROS_Alive":         BOOL,
    f"{DB}.Comm_State":        INT,
    f"{DB}.HB_Timeout_Mult":   INT,
    f"{DB}.Comm_LostCount":    DINT,
}

ARRAYS = {
    f"{DB}.MainRobotLeft_CmdAng": (f"{DB}.MainRobotLeft_CmdAng[{{i}}]", LREAL, 4),
    f"{DB}.MainRobotLeft_ActAng": (f"{DB}.MainRobotLeft_ActAng[{{i}}]", LREAL, 4),
    f"{DB}.MainRobotLeft_ActPos": (f"{DB}.MainRobotLeft_ActPos[{{i}}]", LREAL, 4),
    f"{DB}.MainRobotLeft_Pos4Maint": (f"{DB}.MainRobotLeft_Pos4Maint[{{i}}]", LREAL, 4),
    f"{DB}.MainRobotLeft_HomePos":  (f"{DB}.MainRobotLeft_HomePos[{{i}}]", LREAL, 4),
}


# ---- Optional timing diagnostics (default ON; disable with PLC_TIMING=0) ----
TIMING = os.environ.get("PLC_TIMING", "1") == "1"
_tstat = {}
_tlock = threading.Lock()


def _rec(kind, ms):
    with _tlock:
        s = _tstat.setdefault(kind, [0, 0.0, 0.0])
        s[0] += 1
        s[1] += ms
        if ms > s[2]:
            s[2] = ms


def _timing_reporter():
    while True:
        time.sleep(10.0)
        with _tlock:
            snap = {k: list(v) for k, v in _tstat.items()}
            _tstat.clear()
        if not snap:
            continue
        print("---- PLC timing (last 10 s): kind, calls, mean ms, max ms ----")
        for k in sorted(snap):
            n, tot, mx = snap[k]
            print(f"  {k:<14} {n:>6} {tot / n:>9.3f} {mx:>9.3f}")


def _read_one(name, t):
    t0 = time.perf_counter() if TIMING else 0.0
    if t == BOOL:    v = bool(instance.ReadBool(name))
    elif t == LREAL: v = float(instance.ReadDouble(name))
    elif t == DINT:  v = int(instance.ReadInt32(name))
    elif t == INT:   v = int(instance.ReadInt16(name))
    else:            v = None
    if TIMING:
        _rec(f"api_read_{t}", (time.perf_counter() - t0) * 1000.0)
    return v


def _write_one(name, t, value):
    t0 = time.perf_counter() if TIMING else 0.0
    if t == BOOL:    instance.WriteBool(name, bool(value))
    elif t == LREAL: instance.WriteDouble(name, float(value))
    elif t == DINT:  instance.WriteInt32(name, int(value))
    elif t == INT:   instance.WriteInt16(name, int(value))
    if TIMING:
        _rec(f"api_write_{t}", (time.perf_counter() - t0) * 1000.0)


# =============================================================================
# Batch (single API call) support
# =============================================================================
BATCH_ENABLED = os.environ.get("PLC_BATCH", "1") == "1"
_MAX_BATCH_FAILS = 3

_B = {"ok": False, "why": "not initialised", "fails": 0}

# EPrimitiveDataType members are .NET names, NOT Siemens names:
#   Siemens LReal  -> .NET Double
#   Siemens DInt   -> .NET Int32
#   Siemens Int    -> .NET Int16
#   Siemens Bool   -> .NET Bool
_TYPE_NAMES = {
    BOOL:  ["Bool", "Boolean"],
    LREAL: ["Double"],
    DINT:  ["Int32"],
    INT:   ["Int16"],
}
_VALUE_ATTR = {BOOL: "Bool", LREAL: "Double", DINT: "Int32", INT: "Int16"}
_PY_CAST = {BOOL: bool, LREAL: float, DINT: int, INT: int}


def _clr_members(obj):
    """Field + property names of a .NET value (falls back to dir())."""
    names = set()
    try:
        t = obj.GetType()
        for f in t.GetFields():
            names.add(str(f.Name))
        for p in t.GetProperties():
            names.add(str(p.Name))
    except Exception:
        pass
    if not names:
        names = {n for n in dir(obj) if not n.startswith("_")}
    return names


def _code_ok(code):
    c = str(code)
    return c == "OK" or c.endswith("_OK") or c.endswith(".OK")


def _init_batch():
    """Discover the managed struct layout. Returns (ok, reason)."""
    try:
        from Siemens.Simatic.Simulation.Runtime import (
            SDataValueByName, EPrimitiveDataType as EDataType,
        )
        from System import Array
    except Exception as e:
        return False, f"cannot import batch types: {e}"

    try:
        probe = SDataValueByName()
        dv = probe.DataValue
        dv_members = _clr_members(dv)
    except Exception as e:
        return False, f"cannot create SDataValueByName: {e}"

    type_attr = next((n for n in ("Type", "DataType") if n in dv_members), None)
    if type_attr is None:
        return False, f"SDataValue has no Type field; members={sorted(dv_members)}"

    needed = set(_VALUE_ATTR.values())
    nested = False
    if not needed <= dv_members:
        try:
            inner_members = _clr_members(dv.Value)
        except Exception:
            inner_members = set()
        if needed <= inner_members:
            nested = True
        else:
            return False, (f"value fields {sorted(needed)} not found; "
                           f"SDataValue members={sorted(dv_members)}")

    enum_members = [n for n in dir(EDataType) if not n.startswith("_")]
    types = {}
    for t, cands in _TYPE_NAMES.items():
        hit = next((c for c in cands if c in enum_members), None)
        if hit is None:
            return False, (f"EPrimitiveDataType has none of {cands}; "
                           f"members={enum_members}")
        types[t] = getattr(EDataType, hit)

    _B.update(S=SDataValueByName, A=Array, types=types,
              type_attr=type_attr, nested=nested)
    return True, "layout discovered"


def _get_value(dv, t):
    holder = dv.Value if _B["nested"] else dv
    return _PY_CAST[t](getattr(holder, _VALUE_ATTR[t]))


def _build_array(items, with_values):
    S, A = _B["S"], _B["A"]
    arr = A[S](len(items))
    for i, it in enumerate(items):
        name, t = it[0], it[1]
        sig = S()
        sig.Name = name
        dv = sig.DataValue
        setattr(dv, _B["type_attr"], _B["types"][t])
        if with_values:
            v = _PY_CAST[t](it[2])
            if _B["nested"]:
                inner = dv.Value
                setattr(inner, _VALUE_ATTR[t], v)
                dv.Value = inner
            else:
                setattr(dv, _VALUE_ATTR[t], v)
        sig.DataValue = dv
        arr[i] = sig
    return arr


def _batch_read(items):
    """items: [(name, type)] -> [value or 'ERR:...'] using ONE API call."""
    t0 = time.perf_counter() if TIMING else 0.0
    arr = _build_array(items, with_values=False)
    instance.ReadSignals(arr)
    out = []
    for i, (name, t) in enumerate(items):
        sig = arr[i]
        if _code_ok(sig.ErrorCode):
            out.append(_get_value(sig.DataValue, t))
        else:
            out.append(f"ERR:{sig.ErrorCode}")
    if TIMING:
        _rec("api_read_batch", (time.perf_counter() - t0) * 1000.0)
    return out


def _batch_write(items):
    """items: [(name, type, value)] -> [bool] using ONE API call."""
    t0 = time.perf_counter() if TIMING else 0.0
    arr = _build_array(items, with_values=True)
    instance.WriteSignals(arr)
    out = [_code_ok(arr[i].ErrorCode) for i in range(len(items))]
    if TIMING:
        _rec("api_write_batch", (time.perf_counter() - t0) * 1000.0)
    return out


def _batch_failed(where, err):
    _B["fails"] += 1
    print(f"[batch] {where} failed ({_B['fails']}/{_MAX_BATCH_FAILS}): {err}")
    if _B["fails"] >= _MAX_BATCH_FAILS:
        _B["ok"] = False
        _B["why"] = f"disabled after repeated failures; last: {err}"
        print("[batch] DISABLED, using per-tag calls from now on")


def _selftest_batch():
    """Batch must agree with per-tag calls. Returns (ok, reason)."""
    read_names = [
        f"{DB}.MainRobotLeft_CmdAng[1]",           # LReal -> Double
        f"{DB}.MainRobotLeft_CmdEnable",           # Bool
        f"{DB}.Heartbeat_Counter",                 # DInt -> Int32
        f"{DB}.Comm_State",                        # Int  -> Int16
    ]
    items = [(n, TAGS[n]) for n in read_names if n in TAGS]
    try:
        a = _batch_read(items)
        b = [_read_one(n, t) for n, t in items]
    except Exception as e:
        return False, f"read self-test raised: {e}"
    for (n, t), x, y in zip(items, a, b):
        if isinstance(x, str) or isinstance(y, str):
            return False, f"read self-test error on {n}: {x!r} vs {y!r}"
        if t == LREAL:
            if abs(x - y) > 1e-9:
                return False, f"read mismatch on {n}: {x} vs {y}"
        elif x != y:
            if t == DINT and abs(int(x) - int(y)) <= 2:
                continue
            return False, f"read mismatch on {n}: {x} vs {y}"

    write_names = [
        f"{DB}.MainRobotLeft_ActAng[1]",           # LReal
        f"{DB}.MainRobotLeft_CmdAck",              # Bool
        f"{DB}.ROS_Ack_Counter",                   # DInt
    ]
    w = [(n, TAGS[n]) for n in write_names if n in TAGS]
    try:
        before = [_read_one(n, t) for n, t in w]
        res = _batch_write([(n, t, v) for (n, t), v in zip(w, before)])
        after = [_read_one(n, t) for n, t in w]
    except Exception as e:
        return False, f"write self-test raised: {e}"
    if not all(res):
        return False, f"write self-test: API reported errors {res}"
    for (n, t), x, y in zip(w, before, after):
        if (abs(x - y) > 1e-9) if t == LREAL else (x != y):
            return False, f"write-back mismatch on {n}: {x} vs {y}"
    return True, "self-test passed"


def setup_batch():
    """Enable / disable batch from the OUTSIDE (env var PLC_BATCH).

    Examples:
        PowerShell :  $env:PLC_BATCH = "0"; python .\\plc_server_win4.py
        cmd.exe    :  set PLC_BATCH=0 && python plc_server_win4.py
        default    :  batch ON
    """
    print(f"[batch] env PLC_BATCH = {os.environ.get('PLC_BATCH', '<unset>')}")
    print(f"[batch] env PLC_TIMING = {os.environ.get('PLC_TIMING', '<unset>')}")
    if not BATCH_ENABLED:
        _B["why"] = "PLC_BATCH=0 (from environment)"
        print("[batch] DISABLED by environment (PLC_BATCH=0)")
        print("[batch] using per-tag calls")
        return
    ok, why = _init_batch()
    if ok:
        ok, why = _selftest_batch()
    _B["ok"], _B["why"] = ok, why
    if ok:
        print("[batch] ENABLED: one API call per read/write request "
              "(self-test passed)")
    else:
        print(f"[batch] DISABLED: {why}")
        print("[batch] falling back to per-tag calls (old behaviour)")


def read_tag(name):
    if name in ARRAYS:
        tmpl, t, n = ARRAYS[name]
        out = []
        for i in range(1, n + 1):
            try:
                out.append(_read_one(tmpl.format(i=i), t))
            except SimulationRuntimeException as e:
                out.append(f"ERR:{e}")
        return out
    if name in TAGS:
        try:
            return _read_one(name, TAGS[name])
        except SimulationRuntimeException as e:
            return f"ERR:{e}"
    return None


def write_tag(name, value):
    if name in ARRAYS:
        tmpl, t, n = ARRAYS[name]
        if not isinstance(value, (list, tuple)):
            print(f"write_tag: {name} expects a list, got {type(value).__name__}")
            return False
        if len(value) != n:
            print(f"write_tag: {name} expects {n} elements, got {len(value)}")
            return False
        ok = True
        for i, v in enumerate(value, start=1):
            try:
                _write_one(tmpl.format(i=i), t, v)
            except SimulationRuntimeException as e:
                print(f"write_tag({name}[{i}]) failed: {e}")
                ok = False
        return ok
    if name in TAGS:
        try:
            _write_one(name, TAGS[name], value)
            return True
        except SimulationRuntimeException as e:
            print(f"write_tag({name}) failed: {e}")
            return False
    return False


def _expand(name):
    """name -> ('array'|'tag', [(element_name, type)]) or None if unknown."""
    if name in ARRAYS:
        tmpl, t, n = ARRAYS[name]
        return "array", [(tmpl.format(i=i), t) for i in range(1, n + 1)]
    if name in TAGS:
        return "tag", [(name, TAGS[name])]
    return None


def read_many(names):
    """Read several tags/arrays. ONE API call in batch mode."""
    result, plan, flat = {}, [], []
    for n in names:
        e = _expand(n)
        if e is None:
            result[n] = None
            continue
        plan.append((n, e[0], len(flat), len(e[1])))
        flat.extend(e[1])

    vals = None
    if flat and _B["ok"]:
        try:
            vals = _batch_read(flat)
            _B["fails"] = 0
        except Exception as ex:
            _batch_failed("read", ex)
    if flat and vals is None:
        vals = []
        for el, t in flat:
            try:
                vals.append(_read_one(el, t))
            except SimulationRuntimeException as e:
                vals.append(f"ERR:{e}")

    for n, kind, start, cnt in plan:
        chunk = vals[start:start + cnt]
        result[n] = chunk if kind == "array" else chunk[0]
    return result


def write_many(writes):
    """Write several tags/arrays. ONE API call in batch mode."""
    result, plan, flat = {}, [], []
    for n, v in writes.items():
        e = _expand(n)
        if e is None:
            result[n] = False
            continue
        kind, elems = e
        if kind == "array":
            if not isinstance(v, (list, tuple)):
                print(f"write_many: {n} expects a list, got {type(v).__name__}")
                result[n] = False
                continue
            if len(v) != len(elems):
                print(f"write_many: {n} expects {len(elems)} elements, got {len(v)}")
                result[n] = False
                continue
            values = list(v)
        else:
            values = [v]
        plan.append((n, len(flat), len(elems)))
        flat.extend((el, t, val) for (el, t), val in zip(elems, values))

    oks = None
    if flat and _B["ok"]:
        try:
            oks = _batch_write(flat)
            _B["fails"] = 0
        except Exception as ex:
            _batch_failed("write", ex)
    if flat and oks is None:
        oks = []
        for el, t, val in flat:
            try:
                _write_one(el, t, val)
                oks.append(True)
            except SimulationRuntimeException as e:
                print(f"write_many({el}) failed: {e}")
                oks.append(False)

    for n, start, cnt in plan:
        result[n] = all(oks[start:start + cnt])
    return result


def handle_client(conn, addr):
    print(f"[+] Client connected: {addr}")
    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass
    buf = b""
    seq = 0
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    req = json.loads(line.decode("utf-8"))
                except Exception as e:
                    seq += 1
                    conn.sendall((json.dumps({
                        "error": f"bad json: {e}",
                        "seq": seq, "ts": time.time(),
                    }) + "\n").encode())
                    continue

                cmd = req.get("cmd")
                seq += 1
                reply = None
                t_cmd = time.perf_counter() if TIMING else 0.0

                if cmd == "read":
                    names = req.get("tags", list(TAGS.keys()) + list(ARRAYS.keys()))
                    result = read_many(names)
                    reply = {"values": result}

                elif cmd == "write":
                    writes = req.get("tags", {})
                    result = write_many(writes)
                    reply = {"written": result}

                elif cmd == "list":
                    reply = {
                        "tags": list(TAGS.keys()),
                        "arrays": list(ARRAYS.keys()),
                    }

                else:
                    reply = {"error": f"unknown cmd: {cmd}"}

                if TIMING:
                    _rec(f"cmd_{cmd}", (time.perf_counter() - t_cmd) * 1000.0)

                reply["seq"] = seq
                reply["ts"] = time.time()
                conn.sendall((json.dumps(reply) + "\n").encode())

    except Exception as e:
        print(f"[!] Client {addr} error: {e}")
    finally:
        conn.close()
        print(f"[-] Client disconnected: {addr}")


def main():
    host = "0.0.0.0"
    port = 5000

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(5)
    setup_batch()          # env vars are read here, BEFORE any client is served
    print(f"TCP server listening on {host}:{port}")
    if TIMING:
        print("PLC_TIMING=1 (default): timing report every 10 s")
        threading.Thread(target=_timing_reporter, daemon=True).start()
    print("Waiting for ROS 2 bridge to connect...")

    try:
        while True:
            conn, addr = srv.accept()
            threading.Thread(
                target=handle_client, args=(conn, addr), daemon=True
            ).start()
    except KeyboardInterrupt:
        print("Shutting down.")
    finally:
        srv.close()


if __name__ == "__main__":
    main()