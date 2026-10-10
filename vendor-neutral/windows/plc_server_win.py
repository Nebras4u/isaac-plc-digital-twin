# plc_server_win.py
# Runs on Windows. Serves PLCSIM tags over TCP as JSON.
# Supports BOTH:
#   - individual tags: "DB.Var[1]", "DB.Var[2]"
#   - whole arrays:    "DB.Var"  (returns list)

import os
import sys
import json
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

# Type aliases
BOOL  = "Bool"
LREAL = "LReal"
DINT  = "DInt"
INT   = "Int"

TAGS = {
    # ---- Robot commands (PLC → ROS) ----
    f"{DB}.MainRobotLeft_CmdAng[1]": LREAL,
    f"{DB}.MainRobotLeft_CmdAng[2]": LREAL,
    f"{DB}.MainRobotLeft_CmdAng[3]": LREAL,
    f"{DB}.MainRobotLeft_CmdAng[4]": LREAL,
    f"{DB}.MainRobotLeft_CmdEnable": BOOL,
    f"{DB}.MainRobotLeft_Vlcty":     LREAL,

    # ---- Robot status ----
    f"{DB}.MainRobotLeft_Emergency_Stop_Active": BOOL,
    f"{DB}.MainRobotLeft_Motion_Active":         BOOL,
    f"{DB}.MainRobotLeft_Group_Fault":           BOOL,
    f"{DB}.MainRobotLeft_Axis1_Fault":           BOOL,
    f"{DB}.MainRobotLeft_Axis2_Fault":           BOOL,
    f"{DB}.MainRobotLeft_Axis3_Fault":           BOOL,
    f"{DB}.MainRobotLeft_Axis4_Fault":           BOOL,

    # ---- Robot feedback (ROS → PLC) ----
    f"{DB}.MainRobotLeft_ActAng[1]": LREAL,
    f"{DB}.MainRobotLeft_ActAng[2]": LREAL,
    f"{DB}.MainRobotLeft_ActAng[3]": LREAL,
    f"{DB}.MainRobotLeft_ActAng[4]": LREAL,
    f"{DB}.MainRobotLeft_CmdAck":    BOOL,

    # ---- Heartbeat protocol ----
    f"{DB}.Heartbeat_Counter": DINT,
    f"{DB}.ROS_Ack_Counter":   DINT,
    f"{DB}.ROS_Alive":         BOOL,
    f"{DB}.Comm_State":        INT,
    f"{DB}.HB_Timeout_Mult":   INT,
    f"{DB}.Comm_LostCount":    DINT,
}

# ---- Array definitions (whole arrays) ----
# name → (start_tag_template, element_type, count)
ARRAYS = {
    f"{DB}.MainRobotLeft_CmdAng": (f"{DB}.MainRobotLeft_CmdAng[{{i}}]", LREAL, 4),
    f"{DB}.MainRobotLeft_ActAng": (f"{DB}.MainRobotLeft_ActAng[{{i}}]", LREAL, 4),
    f"{DB}.MainRobotLeft_ActPos": (f"{DB}.MainRobotLeft_ActPos[{{i}}]", LREAL, 4),
    f"{DB}.MainRobotLeft_Pos4Maint": (f"{DB}.MainRobotLeft_Pos4Maint[{{i}}]", LREAL, 4),
    f"{DB}.MainRobotLeft_HomePos":  (f"{DB}.MainRobotLeft_HomePos[{{i}}]", LREAL, 4),
}


def _read_one(name, t):
    if t == BOOL:  return bool(instance.ReadBool(name))
    if t == LREAL: return float(instance.ReadDouble(name))
    if t == DINT:  return int(instance.ReadInt32(name))
    if t == INT:   return int(instance.ReadInt16(name))
    return None


def _write_one(name, t, value):
    if t == BOOL:  instance.WriteBool(name, bool(value))
    elif t == LREAL: instance.WriteDouble(name, float(value))
    elif t == DINT:  instance.WriteInt32(name, int(value))
    elif t == INT:   instance.WriteInt16(name, int(value))


def read_tag(name):
    # Case 1: array name
    if name in ARRAYS:
        tmpl, t, n = ARRAYS[name]
        out = []
        for i in range(1, n + 1):
            try:
                out.append(_read_one(tmpl.format(i=i), t))
            except SimulationRuntimeException as e:
                out.append(f"ERR:{e}")
        return out

    # Case 2: individual tag
    if name in TAGS:
        try:
            return _read_one(name, TAGS[name])
        except SimulationRuntimeException as e:
            return f"ERR:{e}"

    return None


def write_tag(name, value):
    # Case 1: array name + list value
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

    # Case 2: individual tag
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
    buf = b""
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
                    conn.sendall((json.dumps({"error": f"bad json: {e}"}) + "\n").encode())
                    continue

                cmd = req.get("cmd")
                if cmd == "read":
                    names = req.get("tags", list(TAGS.keys()) + list(ARRAYS.keys()))
                    result = {n: read_tag(n) for n in names}
                    conn.sendall((json.dumps({"values": result}) + "\n").encode())

                elif cmd == "write":
                    writes = req.get("tags", {})
                    result = {n: write_tag(n, v) for n, v in writes.items()}
                    conn.sendall((json.dumps({"written": result}) + "\n").encode())

                elif cmd == "list":
                    conn.sendall((json.dumps({
                        "tags": list(TAGS.keys()),
                        "arrays": list(ARRAYS.keys()),
                    }) + "\n").encode())

                else:
                    conn.sendall((json.dumps({"error": f"unknown cmd: {cmd}"}) + "\n").encode())
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
    print("Waiting for ROS 2 bridge to connect...")

    try:
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        print("Shutting down.")
    finally:
        srv.close()


if __name__ == "__main__":
    main()