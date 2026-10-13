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

Tested with asyncua 2.1.0 (read_values / write_values API).

Address handling (OPC UA):
  - If a signal's addr starts with "ns=", it is treated as a complete
    OPC UA Node ID and used as-is. This makes the bridge vendor-neutral.
  - Otherwise, the address is built from the YAML prefix using
    vars.opcua_addr_template (default: '{ns}."{raw}"' for Siemens).

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

from bridge_logger import setup_logging

try:
    from diagnostic_updater import Updater, FunctionDiagnosticTask
    from diagnostic_msgs.msg import DiagnosticStatus
    _HAS_DIAG = True
except Exception:
    _HAS_DIAG = False


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


# --------------------------------------------------------------------------
# tcp_json driver
# --------------------------------------------------------------------------
class TcpJsonDriver(PlcDriver):
    """Talks to plc_server_win.py over newline-delimited JSON.

    Rejects stale replies (seq <= last seen) after reconnect.
    """

    def __init__(self, cfg):
        self.host = cfg["host"]
        self.port = cfg.get("port", 5000)
        self.timeout = cfg.get("timeout", 1.0)
        self.sock = None
        self.buf = b""
        self._last_seq = 0

    def connect(self):
        self.close()
        self.sock = socket.create_connection(
            (self.host, self.port), timeout=self.timeout
        )
        self.sock.settimeout(self.timeout)
        self.buf = b""
        self._last_seq = 0

    def _request(self, req):
        self.sock.sendall((json.dumps(req) + "\n").encode())
        while b"\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("server closed the connection")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        resp = json.loads(line.decode("utf-8"))

        seq = resp.get("seq")
        if seq is not None:
            try:
                seq = int(seq)
            except (TypeError, ValueError):
                seq = None
        if seq is not None:
            if seq <= self._last_seq:
                raise RuntimeError(
                    f"stale reply: seq={seq}, last={self._last_seq}"
                )
            self._last_seq = seq
        return resp

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


# --------------------------------------------------------------------------
# OPC UA driver (asyncua 2.x — uses read_values / write_values)
# --------------------------------------------------------------------------
class OpcUaDriver(PlcDriver):
    """OPC UA client for asyncua 2.x (tested on 2.1.0).

    Uses client.read_values / client.write_values, which are batched:
    one ReadRequest / WriteRequest per call, regardless of node count.

    Reconnect strategy
    ------------------
    asyncua's sync wrapper caches the asyncio event loop per thread.
    A hard disconnect (server stop) leaves that loop in a poisoned
    state: the next ``Client(...).connect()`` re-uses the dead loop
    and either raises immediately or hangs forever.

    To recover, we fully tear down the underlying sync client on
    ``close()`` (calling ``disconnect()``, dropping the cached loop
    if the API exposes a reset) and rebuild a fresh ``Client`` on
    every ``connect()``. All driver methods are guarded by a lock so
    a reconnect from the rclpy timer thread can't race a read/write.
    """

    _YAML_TO_UA = None

    def __init__(self, cfg):
        self.endpoint = cfg["endpoint"]
        self.timeout = cfg.get("timeout", 5.0)
        self.user = cfg.get("user")
        self.password = cfg.get("password")
        self.client = None
        self.nodes = {}
        self.types = {}
        self._lock = threading.RLock()

    def set_signal_types(self, types):
        self.types = dict(types)

    # -- connect / close --------------------------------------------------
    def connect(self):
        from asyncua.sync import Client
        with self._lock:
            self._close_locked()
            self.client = Client(self.endpoint, timeout=self.timeout)
            if self.user:
                self.client.set_user(self.user)
                self.client.set_password(self.password)
            try:
                self.client.connect()
            except Exception:
                try:
                    self.client.disconnect()
                except Exception:
                    pass
                self.client = None
                raise
            self.nodes.clear()

    def _close_locked(self):
        if self.client is not None:
            try:
                self.client.disconnect()
            except Exception:
                pass
            try:
                from asyncua.sync import SyncClient
                for attr in ("_reset_loop", "reset_loop"):
                    fn = getattr(SyncClient, attr, None)
                    if callable(fn):
                        try:
                            fn()
                        except Exception:
                            pass
                        break
            except Exception:
                pass
            self.client = None
        self.nodes.clear()

    def close(self):
        with self._lock:
            self._close_locked()

    # -- node cache -------------------------------------------------------
    def _node(self, addr):
        if addr not in self.nodes:
            self.nodes[addr] = self.client.get_node(addr)
        return self.nodes[addr]

    # -- read -------------------------------------------------------------
    @classmethod
    def _yaml_to_ua(cls):
        if cls._YAML_TO_UA is None:
            from asyncua import ua
            cls._YAML_TO_UA = {
                "bool":   ua.VariantType.Boolean,
                "int":    ua.VariantType.Int32,
                "float":  ua.VariantType.Double,
                "string": ua.VariantType.String,
            }
        return cls._YAML_TO_UA

    def read(self, addrs):
        addrs = list(addrs)
        if not addrs:
            return {}
        with self._lock:
            if self.client is None:
                raise ConnectionError("OPC UA client is not connected")
            nodes = [self._node(a) for a in addrs]
            try:
                values = self.client.read_values(nodes)
            except Exception:
                out = {}
                for a in addrs:
                    try:
                        out[a] = self._node(a).read_value()
                    except Exception:
                        out[a] = None
                return out

            out = {}
            for a, v in zip(addrs, values):
                try:
                    if hasattr(v, "tolist"):
                        v = v.tolist()
                    out[a] = v
                except Exception:
                    out[a] = None
            return out

    # -- write ------------------------------------------------------------
    def _make_variant(self, addr, value):
        from asyncua import ua
        ua_type = self._yaml_to_ua()[self.types.get(addr, "float")]
        if isinstance(value, (list, tuple)):
            return ua.Variant(list(value), ua_type, is_array=True)
        return ua.Variant(value, ua_type)

    def write(self, values):
        from asyncua import ua
        if not values:
            return []
        with self._lock:
            if self.client is None:
                raise ConnectionError("OPC UA client is not connected")
            addrs = list(values.keys())
            variants = []
            ok_addrs = []
            failed = []
            for a in addrs:
                try:
                    variants.append(self._make_variant(a, values[a]))
                    ok_addrs.append(a)
                except Exception:
                    failed.append(a)

            if variants:
                try:
                    statuses = self.client.write_values(
                        [self._node(a) for a in ok_addrs], variants
                    )
                    for a, st in zip(ok_addrs, statuses):
                        if hasattr(st, "is_good") and not st.is_good():
                            failed.append(a)
                except Exception:
                    for a in ok_addrs:
                        try:
                            node = self._node(a)
                            node.write_value(
                                ua.DataValue(self._make_variant(a, values[a]))
                            )
                        except Exception:
                            failed.append(a)

            return failed


# --------------------------------------------------------------------------
# plc_api driver (Windows only, PLCSIM Advanced direct)
# --------------------------------------------------------------------------
class PlcApiDriver(PlcDriver):
    """Direct PLCSIM Advanced API (Windows only). For lab/simulation only."""

    def __init__(self, cfg):
        self.instance_name = cfg.get("instance", "ros2_")
        self.instance = None

    def connect(self):
        import clr
        import os

        api_dir = r"C:\Program Files (x86)\Common Files\Siemens\PLCSIMADV\API\6.0"
        dll = os.path.join(
            api_dir, "Siemens.Simatic.Simulation.Runtime.Api.x64.dll"
        )
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
                out[a] = self._read_one(a)
            except Exception:
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


# ==========================================================================
# 2 + 3. Mapping (from YAML) and logic
# ==========================================================================
class PlcRosBridge(Node):
    def __init__(self):
        super().__init__("plc_ros_bridge")
        self.declare_parameter("config", "plc_ros_bridge.yaml")
        with open(self.get_parameter("config").value) as f:
            cfg = yaml.safe_load(f)

        self._validate_config(cfg)
        self.cfg = cfg

        # ---- structured logging ----
        self.log = setup_logging(
            cfg, logger_name="plc_ros_bridge", ros_logger=self.get_logger()
        )

        driver_name = cfg["plc"]["driver"]
        self.driver_name = driver_name
        self.driver = DRIVERS[driver_name](cfg["plc"])

        # ---- address prefix / template per protocol ----
        vars_cfg = cfg.get("vars", {})
        ns = vars_cfg.get("ns", "")
        db = vars_cfg.get("db", "")
        opcua_tpl = vars_cfg.get("opcua_addr_template", '{ns}."{raw}"')

        def make_addr(raw):
            if raw.startswith("ns="):
                return raw
            if driver_name == "opcua":
                return opcua_tpl.format(ns=ns, raw=raw)
            else:
                return f'{db}.{raw}'

        # ---- signals ----
        self.sig = {}
        for name, s in cfg["signals"].items():
            raw_addr = s["addr"]
            self.sig[name] = {
                "addr": make_addr(raw_addr),
                "type": s.get("type", "float"),
                "dir":  s.get("dir", "read"),
            }
        self.read_names = [n for n, s in self.sig.items() if s["dir"] == "read"]

        if hasattr(self.driver, "set_signal_types"):
            self.driver.set_signal_types({
                s["addr"]: s["type"] for s in self.sig.values()
            })

        # ---- robot / heartbeat ----
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
                raise ValueError(
                    f"signal '{n}' is used in the config but not defined"
                )

        # ---- publishers ----
        self.pubs = [
            (p, self.create_publisher(MSG_TYPES[p["type"]], p["topic"], 10))
            for p in cfg.get("publish", [])
        ]
        self.pub_conn  = self.create_publisher(Bool, "/plc/connected", 10)
        self.pub_estop = self.create_publisher(Bool, "/plc/emergency_stop", 10)
        self._estop_from_plc = any(
            p["topic"] == "/plc/emergency_stop" and p["type"] == "bool"
            for p in cfg.get("publish", [])
        )
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
        self._backoff = 1.0
        self._backoff_min = 1.0
        self._backoff_max = cfg.get("backoff_max", 30.0)
        self._last_ack = None
        self._warned = {}
        self._warn_cooldown = cfg.get("warn_cooldown", 60.0)

        # ---- watchdog ----
        self._watchdog_timeout = cfg.get("watchdog_timeout", 2.0)
        self._last_tick_ok = time.time()

        # ---- diagnostics ----
        self._diag_updater = None
        if _HAS_DIAG:
            self._diag_updater = Updater(self)
            self._diag_updater.setHardwareID("plc_ros_bridge")
            self._diag_updater.add(
                FunctionDiagnosticTask("PLC link", self._diag)
            )
            self.create_timer(1.0, self._diag_updater.update)

        # ---- timers ----
        poll_hz = cfg.get("poll_hz", 20.0)
        self.create_timer(1.0 / poll_hz, self.tick)
        self.create_timer(0.5, self._check_watchdog)

        self.log.info(
            "bridge_started",
            extra={"driver": driver_name, "poll_hz": poll_hz},
        )

    # ------------------------------------------------------------------
    def _validate_config(self, cfg):
        if "plc" not in cfg or "driver" not in cfg["plc"]:
            raise ValueError("config: 'plc.driver' is required")
        if cfg["plc"]["driver"] not in DRIVERS:
            raise ValueError(f"unknown driver: {cfg['plc']['driver']}")
        if "signals" not in cfg or not isinstance(cfg["signals"], dict):
            raise ValueError("config: 'signals' must be a non-empty mapping")
        for name, s in cfg["signals"].items():
            if "addr" not in s:
                raise ValueError(f"signal '{name}' missing 'addr'")
            t = s.get("type", "float")
            if t not in CAST:
                raise ValueError(
                    f"signal '{name}' has invalid type '{t}' "
                    f"(allowed: {list(CAST.keys())})"
                )
            d = s.get("dir", "read")
            if d not in ("read", "write"):
                raise ValueError(f"signal '{name}' has invalid dir '{d}'")

    # ------------------------------------------------------------------
    def on_joint_states(self, msg):
        with self._js_lock:
            for name, pos in zip(msg.name, msg.position):
                i = self.joint_idx.get(name)
                if i is not None:
                    self._js_rad[i] = float(pos)
            self._js_ok = True

    def _set_connected(self, state):
        if state == self.connected:
            return
        if state:
            self._last_tick_ok = time.time()
            self.log.info(
                "plc_connected",
                extra={"driver": self.driver_name, "backoff_s": self._backoff},
            )
            if self._estop_from_plc:
                # The real PLC estop value is published on the next tick;
                # do not declare "cleared" before reading it.
                self.log.info("plc_reconnected_estop_from_plc")
            else:
                # No PLC estop signal configured: fall back to clearing.
                self.pub_estop.publish(Bool(data=False))
                self.log.info("plc_reconnected_estop_cleared")
        else:
            self.log.warning(
                "plc_disconnected",
                extra={"driver": self.driver_name},
            )
            self.pub_estop.publish(Bool(data=True))
            self.log.warning("plc_lost_estop_published")
        self.connected = state
        self.pub_conn.publish(Bool(data=state))

    def _try_connect(self):
        now = time.time()
        if now - self._last_try < self._backoff:
            return False
        self._last_try = now
        try:
            self.driver.connect()
        except Exception as e:
            self.log.warning(
                "plc_connect_failed",
                extra={"error": str(e), "next_retry_in_s": self._backoff},
            )
            self._set_connected(False)
            self._backoff = min(self._backoff * 2.0, self._backoff_max)
            return False
        self._backoff = self._backoff_min
        self._last_ack = None
        self._set_connected(True)
        return True

    def _get(self, name, raw):
        s = self.sig[name]
        v = raw.get(s["addr"])
        try:
            if v is None:
                raise ValueError("no value")
            if isinstance(v, (list, tuple)):
                return [float(x) for x in v]
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
            return CAST[s["type"]](v)
        except (TypeError, ValueError) as e:
            now = time.time()
            last = self._warned.get(name, 0.0)
            if now - last > self._warn_cooldown:
                self._warned[name] = now
                self.log.warning(
                    "signal_bad_value",
                    extra={
                        "signal": name,
                        "addr": s["addr"],
                        "value": repr(v),
                        "error": str(e),
                    },
                )
            return None

    # ------------------------------------------------------------------
    def tick(self):
        if not self.connected and not self._try_connect():
            return
        tick_id = int(time.time() * 1000) % 1000000
        t0 = time.time()
        try:
            raw = self.driver.read(
                [self.sig[n]["addr"] for n in self.read_names]
            )
            vals = {n: self._get(n, raw) for n in self.read_names}
            read_ms = (time.time() - t0) * 1000.0
            if read_ms > 200.0:
                self.log.warning(
                    "slow_read",
                    extra={"tick_id": tick_id, "read_ms": round(read_ms, 1)},
                )

            self._last_tick_ok = time.time()
            self._publish(vals)

            writes = {}
            pending_ack = None

            if self.hb_counter and self.hb_ack:
                hb = vals.get(self.hb_counter)
                if hb is not None and hb != self._last_ack:
                    writes[self.sig[self.hb_ack]["addr"]] = int(hb)
                    pending_ack = int(hb)

            with self._js_lock:
                if self.act_ang and self._js_ok:
                    if len(self.act_ang) == 1:
                        sig = self.act_ang[0]
                        addr = self.sig[sig]["addr"]
                        writes[addr] = [
                            math.degrees(float(x)) for x in self._js_rad
                        ]
                    else:
                        for sig, rad in zip(self.act_ang, self._js_rad):
                            writes[self.sig[sig]["addr"]] = math.degrees(
                                float(rad)
                            )
                    if self.cmd_ack:
                        writes[self.sig[self.cmd_ack]["addr"]] = True

            if writes:
                failed = self.driver.write(writes)
                for a in failed:
                    self.log.warning(
                        "write_failed",
                        extra={"tick_id": tick_id, "addr": a},
                    )
                if (
                    pending_ack is not None
                    and self.sig[self.hb_ack]["addr"] not in failed
                ):
                    self._last_ack = pending_ack

            self.log.debug(
                "tick_ok",
                extra={
                    "tick_id": tick_id,
                    "read_ms": round(read_ms, 2),
                    "n_read": len(self.read_names),
                    "n_write": len(writes),
                },
            )

        except Exception as e:
            self.log.error(
                "plc_link_error",
                extra={"tick_id": tick_id, "error": str(e),
                       "driver": self.driver_name},
            )
            self.driver.close()
            self._set_connected(False)
            self._last_try = 0.0

    def _publish(self, vals):
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
            js.position = ang
            self.pub_cmd_ang.publish(js)

    # ------------------------------------------------------------------
    def _check_watchdog(self):
        now = time.time()
        if self.connected and now - self._last_tick_ok > self._watchdog_timeout:
            stalled = round(now - self._last_tick_ok, 2)
            self.log.error("watchdog_tick_stalled",
                           extra={"stalled_s": stalled})
            self.pub_estop.publish(Bool(data=True))

    # ------------------------------------------------------------------
    def _diag(self, stat):
        if not _HAS_DIAG:
            return stat
        if self.connected:
            stat.summary(DiagnosticStatus.OK, "connected")
        else:
            stat.summary(DiagnosticStatus.ERROR, "disconnected")
        stat.add("driver", self.driver_name)
        stat.add("backoff_s", f"{self._backoff:.1f}")
        stat.add("last_tick_age_s", f"{time.time() - self._last_tick_ok:.2f}")
        return stat

    def destroy_node(self):
        try:
            self.driver.close()
        except Exception:
            pass
        try:
            self.log.info("bridge_shutdown")
        except Exception:
            pass
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