# plc_server_win.py
# Runs on Windows. Serves PLCSIM tags over TCP as JSON.

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
INSTANCE_NAME = "ros2_"
instance = SimulationRuntimeManager.CreateInterface(INSTANCE_NAME)
print(f"Connected to PLCSIM: {instance.Name}, State: {instance.OperatingState}")

if str(instance.OperatingState) != "Run":
    instance.Run()
    print(f"After Run: {instance.OperatingState}")

instance.UpdateTagList()
print(f"Tag list updated. Total tags: {len(instance.TagInfos)}")


# ---- Tag map ----
DB = "MAIN_KUKA_KR210_L150_L"

TAGS = {
    # ---- Robot commands (PLC → ROS) ----
    f"{DB}.MainRobotLeft_CmdAng[1]": "LReal",
    f"{DB}.MainRobotLeft_CmdAng[2]": "LReal",
    f"{DB}.MainRobotLeft_CmdAng[3]": "LReal",
    f"{DB}.MainRobotLeft_CmdAng[4]": "LReal",
    f"{DB}.MainRobotLeft_CmdEnable": "Bool",
    f"{DB}.MainRobotLeft_Vlcty":     "LReal",

    # ---- Robot status ----
    f"{DB}.MainRobotLeft_Emergency_Stop_Active": "Bool",
    f"{DB}.MainRobotLeft_Motion_Active":         "Bool",
    f"{DB}.MainRobotLeft_Group_Fault":           "Bool",
    f"{DB}.MainRobotLeft_Axis1_Fault":           "Bool",
    f"{DB}.MainRobotLeft_Axis2_Fault":           "Bool",
    f"{DB}.MainRobotLeft_Axis3_Fault":           "Bool",
    f"{DB}.MainRobotLeft_Axis4_Fault":           "Bool",

    # ---- Robot feedback (ROS → PLC) ----
    f"{DB}.MainRobotLeft_ActAng[1]": "LReal",
    f"{DB}.MainRobotLeft_ActAng[2]": "LReal",
    f"{DB}.MainRobotLeft_ActAng[3]": "LReal",
    f"{DB}.MainRobotLeft_ActAng[4]": "LReal",
    f"{DB}.MainRobotLeft_CmdAck":    "Bool",

    # ---- Heartbeat protocol ----
    f"{DB}.Heartbeat_Counter": "DInt",
    f"{DB}.ROS_Ack_Counter":   "DInt",
    f"{DB}.ROS_Alive":         "Bool",
    f"{DB}.Comm_State":        "Int",
    f"{DB}.HB_Timeout_Mult":   "Int",
    f"{DB}.Comm_LostCount":    "DInt",
}


def read_tag(name):
    t = TAGS[name]
    try:
        if t == "Bool":
            return bool(instance.ReadBool(name))
        if t == "LReal":
            return float(instance.ReadDouble(name))
        if t == "DInt":
            return int(instance.ReadInt32(name))
        if t == "Int":
            return int(instance.ReadInt16(name))
    except SimulationRuntimeException as e:
        return f"ERR:{e}"
    return None


def write_tag(name, value):
    t = TAGS[name]
    try:
        if t == "Bool":
            instance.WriteBool(name, bool(value))
        elif t == "LReal":
            instance.WriteDouble(name, float(value))
        elif t == "DInt":
            instance.WriteInt32(name, int(value))
        elif t == "Int":
            instance.WriteInt16(name, int(value))
        return True
    except SimulationRuntimeException as e:
        print(f"write_tag({name}) failed: {e}")
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
                    names = req.get("tags", list(TAGS.keys()))
                    result = {n: read_tag(n) for n in names if n in TAGS}
                    conn.sendall((json.dumps({"values": result}) + "\n").encode())

                elif cmd == "write":
                    writes = req.get("tags", {})
                    result = {n: write_tag(n, v) for n, v in writes.items() if n in TAGS}
                    conn.sendall((json.dumps({"written": result}) + "\n").encode())

                elif cmd == "list":
                    conn.sendall((json.dumps({"tags": list(TAGS.keys())}) + "\n").encode())

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
