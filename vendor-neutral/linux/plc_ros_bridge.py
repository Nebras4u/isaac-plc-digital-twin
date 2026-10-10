#!/usr/bin/env python3
"""PLC <-> ROS 2 bridge (vendor neutral).

Three layers:
  1. Driver  : how to talk to the PLC (tcp_json, opcua). Add yours to DRIVERS.
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
    """Talks to plc_server_win.py (read / write over newline-delimited JSON)."""

    def __init__(self, cfg):
        self.host = cfg["host"]
        self.port = cfg.get("port", 5000)
        self.timeout = cfg.get("timeout", 1.0)  # always finite: never hangs
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
            # old servers returned "ERR:..." strings on failure
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
            except ua.UaStatusCodeError:  # bad node id etc. (not a link failure)
                out[a] = None
        return out

    def write(self, values):
        from asyncua import ua

        failed = []
        for a, v in values.items():
            try:
                node = self._node(a)
                if a not in self.vtypes:  # PLC types must match exactly
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


DRIVERS = {"tcp_json": TcpJsonDriver, "opcua": OpcUaDriver}
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

        self.driver = DRIVERS[cfg["plc"]["driver"]](cfg["plc"])

        # ---- signals: logical name -> addr/type/dir ----
        self.sig = {}
        for name, s in cfg["signals"].items():
            addr = s["addr"]
            for k, v in cfg.get("vars", {}).items():
                addr = addr.replace("{" + k + "}", str(v))
            self.sig[name] = {
                "addr": addr,
                "type": s.get("type", "float"),
                "dir": s.get("dir", "read"),
            }
        self.read_names = [n for n, s in self.sig.items() if s["dir"] == "read"]

        # ---- robot / heartbeat sections ----
        robot = cfg.get("robot", {})
        self.joints = robot.get("joints", [])
        self.cmd_ang = robot.get("cmd_ang", [])
        self.act_ang = robot.get("act_ang", [])
        self.cmd_ack = robot.get("cmd_ack")
        hb = cfg.get("heartbeat", {})
        self.hb_counter = hb.get("counter")
        self.hb_ack = hb.get("ack")

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
        self.get_logger().info("plc_ros_bridge started")

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
        self._last_ack = None  # force an ack of the current counter
        self._set_connected(True)
        return True

    def _get(self, name, raw):
        """Read one signal from the raw reply; None if missing or wrong type."""
        s = self.sig[name]
        v = raw.get(s["addr"])
        try:
            if v is None:
                raise ValueError("no value")
            return CAST[s["type"]](v)
        except (TypeError, ValueError):
            if name not in self._warned:
                self._warned.add(name)
                self.get_logger().warn(f"signal '{name}' ({s['addr']}) returned no usable value")
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

            # heartbeat: echo the PLC counter back
            if self.hb_counter and self.hb_ack:
                hb = vals.get(self.hb_counter)
                if hb is not None and hb != self._last_ack:
                    writes[self.sig[self.hb_ack]["addr"]] = int(hb)
                    pending_ack = int(hb)

            # feedback: /joint_states (rad) -> PLC (deg)
            with self._js_lock:
                if self.act_ang and self._js_ok:
                    for sig, rad in zip(self.act_ang, self._js_rad):
                        writes[self.sig[sig]["addr"]] = math.degrees(rad)
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
        # a signal with no value is NOT published (no fake zeros / fake False)
        for p, pub in self.pubs:
            v = vals.get(p["signal"])
            if v is not None:
                pub.publish(MSG_TYPES[p["type"]](data=CAST[p["type"]](v)))

        if self.pub_cmd_ang:
            ang = [vals.get(n) for n in self.cmd_ang]
            if all(a is not None for a in ang):
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
