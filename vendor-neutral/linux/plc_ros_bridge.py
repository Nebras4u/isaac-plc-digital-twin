#!/usr/bin/env python3
"""PLC <-> ROS 2 bridge (vendor neutral).

Three layers:
  1. Driver  : how to talk to the PLC (tcp_json, opcua, plc_api).
  2. Mapping : plc_ros_bridge.yaml (logical name -> address/type, topics).
  3. Logic   : heartbeat, deg/rad conversion, /joint_states feedback.

Driver contract:
  connect()
  read(addrs)   -> {addr: value or None}
  write(values) -> list of addrs that failed   (values = {addr: value})
  close()

Run:
  python3 plc_ros_bridge.py --ros-args -p config:=plc_ros_bridge.yaml
"""
import abc
import json
import math
import socket
import threading
import time

import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32, String

MSG_TYPES = {"bool": Bool, "int": Int32, "float": Float64, "string": String}
CAST = {"bool": bool, "int": int, "float": float, "string": str}


# ==========================================================================
# 1. Drivers
# ==========================================================================
class PlcDriver(abc.ABC):
    @abc.abstractmethod
    def connect(self): ...

    @abc.abstractmethod
    def read(self, addrs): ...

    @abc.abstractmethod
    def write(self, values): ...

    def close(self):
        pass


class TcpJsonDriver(PlcDriver):
    """Talks to plc_server_win.py (read/write over newline-delimited JSON)."""

    def __init__(self, cfg):
        self.host = cfg["host"]
        self.port = cfg.get("port", 5000)
        self.timeout = cfg.get("timeout", 1.0)
        self.sock = None
        self.buf = b""

    def connect(self):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        self.buf = b""

    def _request(self, req):
        self.sock.sendall((json.dumps(req) + "\n").encode())
        while b"\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("server closed the connection")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8"))

    def read(self, addrs):
        resp = self._request({"cmd": "read", "tags": list(addrs)})
        if "values" not in resp:
            raise RuntimeError(f"bad read reply: {resp}")
        out = {}
        for a in addrs:
            v = resp["values"].get(a)
            out[a] = None if isinstance(v, str) and v.startswith("ERR:") else v
        return out

    def write(self, values):
        resp = self._request({"cmd": "write", "tags": values})
        if "written" not in resp:
            raise RuntimeError(f"bad write reply: {resp}")
        return [a for a in values if not resp["written"].get(a, False)]

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        self.sock = None


class OpcUaDriver(PlcDriver):
    """OPC UA client (pip install asyncua). For any vendor with an OPC UA server.
    addr = node id, e.g. 'ns=3;s="DB"."Var"'."""

    def __init__(self, cfg):
        self.endpoint = cfg["endpoint"]
        self.timeout = cfg.get("timeout", 3.0)
        self.user = cfg.get("user")
        self.password = cfg.get("password")
        self.client = None
        self.nodes = {}
        self.vtypes = {}

    def connect(self):
        from asyncua.sync import Client

        self.close()
        self.client = Client(self.endpoint, timeout=self.timeout)
        if self.user:
            self.client.set_user(self.user)
            self.client.set_password(self.password)
        self.client.connect()
        self.nodes.clear()
        self.vtypes.clear()

    def _node(self, addr):
        if addr not in self.nodes:
            self.nodes[addr] = self.client.get_node(addr)
        return self.nodes[addr]

    def read(self, addrs):
        from asyncua import ua

        out = {}
        for a in addrs:
            try:
                out[a] = self._node(a).read_value()
            except ua.UaStatusCodeError:
                out[a] = None
        return out

    def write(self, values):
        from asyncua import ua

        failed = []
        for a, v in values.items():
            try:
                node = self._node(a)
                if a not in self.vtypes:
                    self.vtypes[a] = node.read_data_type_as_variant_type()
                node.write_value(ua.DataValue(ua.Variant(v, self.vtypes[a])))
            except ua.UaStatusCodeError:
                failed.append(a)
        return failed

    def close(self):
        try:
            if self.client:
                self.client.disconnect()
        except Exception:
            pass
        self.client = None


class PlcApiDriver(PlcDriver):
    """Direct PLCSIM Advanced API (Windows only). Fastest.
    Requires: pythonnet + Siemens.Simatic.Simulation.Runtime.Api.x64.dll
    addr format: symbolic tag name (e.g. 'MAIN_DB.Tag[1]')."""

    def __init__(self, cfg):
        self.instance_name = cfg.get("instance", "ros2_")
        self.instance = None

    def connect(self):
        import clr
        import os

        api_dir = r"C:\Program Files (x86)\Common Files\Siemens\PLCSIMADV\API\6.0"
        dll = os.path.join(api_dir, "Siemens.Simatic.Simulation.Runtime.Api.x64.dll")
        os.add_dll_directory(api_dir)
        clr.AddReference(dll)

        from Siemens.Simatic.Simulation.Runtime import SimulationRuntimeManager
        self.instance = SimulationRuntimeManager.CreateInterface(self.instance_name)
        if str(self.instance.OperatingState) != "Run":
            self.instance.Run()
        self.instance.UpdateTagList()

    def _read_one(self, addr):
        info = self.instance.GetTagInfo(addr)
        t = str(info.DataType)
        if "Bool" in t:
            return bool(self.instance.ReadBool(addr))
        if "LReal" in t:
            return float(self.instance.ReadDouble(addr))
        if "Real" in t:
            return float(self.instance.ReadFloat(addr))
        if "DInt" in t:
            return int(self.instance.ReadInt32(addr))
        if "Int" in t:
            return int(self.instance.ReadInt16(addr))
        return None

    def _write_one(self, addr, value):
        info = self.instance.GetTagInfo(addr)
        t = str(info.DataType)
        if "Bool" in t:
            self.instance.WriteBool(addr, bool(value))
        elif "LReal" in t:
            self.instance.WriteDouble(addr, float(value))
        elif "Real" in t:
            self.instance.WriteFloat(addr, float(value))
        elif "DInt" in t:
            self.instance.WriteInt32(addr, int(value))
        elif "Int" in t:
            self.instance.WriteInt16(addr, int(value))

    def read(self, addrs):
        out = {}
        for a in addrs:
            try:
                # try whole array (base name)
                v = self._read_one(a)
                out[a] = v
            except Exception:
                # fallback: try array elements a[1], a[2], ...
                try:
                    elems = []
                    for i in range(1, 17):
                        try:
                            elems.append(self._read_one(f"{a}[{i}]"))
                        except Exception:
                            break
                    out[a] = elems if elems else None
                except Exception:
                    out[a] = None
        return out

    def write(self, values):
        failed = []
        for a, v in values.items():
            try:
                if isinstance(v, (list, tuple)):
                    for i, x in enumerate(v, start=1):
                        self._write_one(f"{a}[{i}]", x)
                else:
                    self._write_one(a, v)
            except Exception:
                failed.append(a)
        return failed

    def close(self):
        try:
            self.instance = None
        except Exception:
            pass


DRIVERS = {
    "tcp_json": TcpJsonDriver,
    "opcua":    OpcUaDriver,
    "plc_api":  PlcApiDriver,
}
# Add yours: DRIVERS["modbus"] = ModbusDriver


# ==========================================================================
# 2 + 3. Mapping (from YAML) and logic
# ==========================================================================
class PlcRosBridge(Node):
    def __init__(self):
        super().__init__("plc_ros_bridge")
        self.declare_parameter("config", "plc_ros_bridge.yaml")
        with open(self.get_parameter("config").value) as f:
            cfg = yaml.safe_load(f)

        self.cfg = cfg
        driver_name = cfg["plc"]["driver"]
        self.driver_name = driver_name
        self.driver = DRIVERS[driver_name](cfg["plc"])

        # ---- Build address prefix per protocol ----
        ns = cfg.get("vars", {}).get("ns", "")
        db = cfg.get("vars", {}).get("db", "")

        def make_addr(raw):
            # raw may already be a full address (contains "." or starts with "ns=")
            if raw.startswith("ns=") or ("." in raw):
                return raw
            if driver_name == "opcua":
                return f'{ns}."{raw}"'
            else:  # tcp_json, plc_api
                return f'{db}.{raw}'

        # ---- signals: logical name -> addr/type/dir ----
        self.sig = {}
        for name, s in cfg["signals"].items():
            raw_addr = s["addr"]
            self.sig[name] = {
                "addr": make_addr(raw_addr),
                "type": s.get("type", "float"),
                "dir":  s.get("dir", "read"),
            }
        self.read_names = [n for n, s in self.sig.items() if s["dir"] == "read"]

        # ---- robot / heartbeat sections ----
        robot = cfg.get("robot", {})
        self.joints  = robot.get("joints", [])
        self.cmd_ang = robot.get("cmd_ang", [])
        self.act_ang = robot.get("act_ang", [])
        self.cmd_ack = robot.get("cmd_ack")
        hb = cfg.get("heartbeat", {})
        self.hb_counter = hb.get("counter")
        self.hb_ack     = hb.get("ack")

        referenced = (
            self.cmd_ang + self.act_ang
            + [x for x in (self.cmd_ack, self.hb_counter, self.hb_ack) if x]
            + [p["signal"] for p in cfg.get("publish", [])]
        )
        for n in referenced:
            if n not in self.sig:
                raise ValueError(f"signal '{n}' is used in the config but not defined")

        # ---- publishers ----
        self.pubs = [
            (p, self.create_publisher(MSG_TYPES[p["type"]], p["topic"], 10))
            for p in cfg.get("publish", [])
        ]
        self.pub_conn = self.create_publisher(Bool, "/plc/connected", 10)
        self.pub_estop = self.create_publisher(Bool, "/plc/emergency_stop", 10)
        self.pub_cmd_ang = None
        if self.cmd_ang:
            self.pub_cmd_ang = self.create_publisher(
                JointState, robot.get("cmd_ang_topic", "/plc/cmd_ang"), 10
            )

        # ---- joint_states feedback ----
        self.joint_idx = {j: i for i, j in enumerate(self.joints)}
        self._js_lock = threading.Lock()
        self._js_rad = [0.0] * len(self.joints)
        self._js_ok = False
        if self.act_ang:
            self.create_subscription(
                JointState, robot.get("joint_states_topic", "/joint_states"),
                self.on_joint_states, 10,
            )

        # ---- state ----
        self.connected = False
        self._last_try = 0.0
        self._last_ack = None
        self._warned = set()

        self.create_timer(1.0 / cfg.get("poll_hz", 20.0), self.tick)
        self.get_logger().info(f"plc_ros_bridge started (driver={driver_name})")

    # ----------------------------------------------------------------------
    def on_joint_states(self, msg):
        with self._js_lock:
            for name, pos in zip(msg.name, msg.position):
                i = self.joint_idx.get(name)
                if i is not None:
                    self._js_rad[i] = float(pos)
            self._js_ok = True

    def _set_connected(self, state):
        if state != self.connected:
            self.get_logger().info("PLC connected" if state else "PLC disconnected")
        self.connected = state
        self.pub_conn.publish(Bool(data=state))
        if not state:
            # safety: estop when PLC link drops
            self.pub_estop.publish(Bool(data=True))
            self.get_logger().warn("PLC lost — estop published")

    def _try_connect(self):
        if time.time() - self._last_try < 2.0:
            return False
        self._last_try = time.time()
        try:
            self.driver.connect()
        except Exception as e:
            self.get_logger().warn(f"PLC connect failed: {e}")
            self._set_connected(False)
            return False
        self._last_ack = None
        self._set_connected(True)
        return True

    def _get(self, name, raw):
        """Read one signal from the raw reply; None if missing or wrong type."""
        s = self.sig[name]
        v = raw.get(s["addr"])
        try:
            if v is None:
                raise ValueError("no value")

            # Arrays (list of numbers)
            if isinstance(v, (list, tuple)):
                return [float(x) for x in v]

            # String values (from TCP server or OPC UA strings)
            if isinstance(v, str):
                if v.startswith("ERR:"):
                    raise ValueError(v)
                if s["type"] == "float":
                    return float(v)
                if s["type"] == "int":
                    return int(v)
                if s["type"] == "bool":
                    return v.lower() in ("true", "1", "yes")
                return v

            # Normal scalar
            return CAST[s["type"]](v)
        except (TypeError, ValueError) as e:
            if name not in self._warned:
                self._warned.add(name)
                self.get_logger().warn(f"signal '{name}' ({s['addr']}) = {v!r} -> {e}")
            return None

    # ----------------------------------------------------------------------
    def tick(self):
        if not self.connected and not self._try_connect():
            return
        try:
            raw = self.driver.read([self.sig[n]["addr"] for n in self.read_names])
            vals = {n: self._get(n, raw) for n in self.read_names}
            self._publish(vals)

            writes = {}
            pending_ack = None

            # heartbeat: echo PLC counter back
            if self.hb_counter and self.hb_ack:
                hb = vals.get(self.hb_counter)
                if hb is not None and hb != self._last_ack:
                    writes[self.sig[self.hb_ack]["addr"]] = int(hb)
                    pending_ack = int(hb)

            # feedback: /joint_states (rad) -> PLC (deg)
            with self._js_lock:
                if self.act_ang and self._js_ok:
                    if len(self.act_ang) == 1:
                        # whole array write
                        sig = self.act_ang[0]
                        addr = self.sig[sig]["addr"]
                        writes[addr] = [math.degrees(float(x)) for x in self._js_rad]
                    else:
                        # one tag per joint
                        for sig, rad in zip(self.act_ang, self._js_rad):
                            writes[self.sig[sig]["addr"]] = math.degrees(float(rad))
                    if self.cmd_ack:
                        writes[self.sig[self.cmd_ack]["addr"]] = True

            if writes:
                failed = self.driver.write(writes)
                for a in failed:
                    self.get_logger().warn(f"write failed: {a}")
                if pending_ack is not None and self.sig[self.hb_ack]["addr"] not in failed:
                    self._last_ack = pending_ack
        except Exception as e:
            self.get_logger().error(f"PLC link error: {e}")
            self.driver.close()
            self._set_connected(False)

    def _publish(self, vals):
        # a signal with no value is NOT published (no fake zeros)
        for p, pub in self.pubs:
            v = vals.get(p["signal"])
            if v is not None:
                pub.publish(MSG_TYPES[p["type"]](data=CAST[p["type"]](v)))

        if self.pub_cmd_ang:
            ang = []
            for n in self.cmd_ang:
                v = vals.get(n)
                if v is None:
                    return
                if isinstance(v, (list, tuple)):
                    ang.extend(v)
                else:
                    ang.append(v)
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(self.joints)
            js.position = ang  # degrees (as in the old bridge)
            self.pub_cmd_ang.publish(js)

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PlcRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()