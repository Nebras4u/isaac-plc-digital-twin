# plc_server_win.py
# Runs on Windows. Serves PLCSIM tags over TCP as JSON.
# Supports BOTH:
#   - individual tags: "DB.Var[1]", "DB.Var[2]"
#   - whole arrays:    "DB.Var"  (returns list)
#
# Every reply carries "seq" (monotonic per connection) and "ts" (unix time),
# so the bridge can reject stale replies after a reconnect.
#
# Optional timing diagnostics: set the environment variable PLC_TIMING=1.
# Every 10 s the server prints, per API call type and per command, how many
# calls were made and the mean / max time in milliseconds. Use it to find
# which PLCSIM API call is slow.

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
INSTANCE_NAME = "ros2"
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


# ---- Optional timing diagnostics (PLC_TIMING=1) ----
TIMING = os.environ.get("PLC_TIMING", "0") == "1"
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


def handle_client(conn, addr):
    print(f"[+] Client connected: {addr}")
    # Send each reply immediately instead of waiting to coalesce packets.
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
                    result = {n: read_tag(n) for n in names}
                    reply = {"values": result}

                elif cmd == "write":
                    writes = req.get("tags", {})
                    result = {n: write_tag(n, v) for n, v in writes.items()}
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
    print(f"TCP server listening on {host}:{port}")
    if TIMING:
        print("PLC_TIMING=1: timing report every 10 s")
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
