#!/usr/bin/env python3
"""PLC <-> ROS 2 bridge (vendor neutral)."""
import abc
import csv
import json
import math
import socket
import threading
import time
from datetime import datetime, timezone

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32, String

from bridge_config import load_bridge_config, ConfigError, CURRENT_SCHEMA_VERSION
from bridge_logger import setup_logging
from safety import ESTOP_TOPIC, resolve_estop


class _LatencyProbe:
    COLS = ["origin_ns", "read_start_ns", "read_done_ns", "cmd_pub_ns",
            "sp_pub_ns", "js_pub_ns", "js_recv_ns", "write_done_ns"]

    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(self.COLS)
        self._f.flush()
        self._last = None

    def record(self, tag, js_recv_ns, write_done_ns):
        parts = tag.split(";")
        if len(parts) != 7 or parts[0] != "p":
            return False
        try:
            nums = [int(x) for x in parts[1:]]
        except ValueError:
            return False
        if nums[0] == self._last:
            return False
        self._last = nums[0]
        self._w.writerow(nums + [js_recv_ns, write_done_ns])
        self._f.flush()
        return True

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass


try:
    from diagnostic_updater import Updater, FunctionDiagnosticTask
    from diagnostic_msgs.msg import DiagnosticStatus
    _HAS_DIAG = True
except Exception:
    _HAS_DIAG = False


MSG_TYPES = {"bool": Bool, "int": Int32, "float": Float64, "string": String}
CAST = {"bool": bool, "int": int, "float": float, "string": str}

JS_STALE_AGE_S = 0.5

PUB_STREAM_QOS = QoSProfile(depth=1)
SUB_STREAM_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)


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
    def __init__(self, cfg):
        self.host = cfg.host
        self.port = cfg.port if cfg.port is not None else 5000
        self.timeout = cfg.timeout
        self.sock = None
        self.buf = b""
        self._last_seq = 0
        self._last_sock_ms = 0.0
        self._last_parse_ms = 0.0

    def connect(self):
        self.close()
        self.sock = socket.create_connection(
            (self.host, self.port), timeout=self.timeout
        )
        self.sock.settimeout(self.timeout)
        try:
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass
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
                raise RuntimeError(f"stale reply: seq={seq}, last={self._last_seq}")
            self._last_seq = seq
        return resp

    def read(self, addrs):
        t0 = time.perf_counter()
        resp = self._request({"cmd": "read", "tags": list(addrs)})
        t1 = time.perf_counter()
        if "values" not in resp:
            raise RuntimeError(f"bad read reply: {resp}")
        out = {}
        for a in addrs:
            v = resp["values"].get(a)
            out[a] = None if isinstance(v, str) and v.startswith("ERR:") else v
        t2 = time.perf_counter()
        self._last_sock_ms = (t1 - t0) * 1000.0
        self._last_parse_ms = (t2 - t1) * 1000.0
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
    _YAML_TO_UA = None

    def __init__(self, cfg):
        self.endpoint = cfg.endpoint
        self.timeout = cfg.timeout
        self.user = cfg.user
        self.password = cfg.password
        self.client = None
        self.nodes = {}
        self.types = {}
        self._lock = threading.RLock()
        self._last_sock_ms = 0.0
        self._last_parse_ms = 0.0

    def set_signal_types(self, types):
        self.types = dict(types)

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

    def _node(self, addr):
        if addr not in self.nodes:
            self.nodes[addr] = self.client.get_node(addr)
        return self.nodes[addr]

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
        t0 = time.perf_counter()
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
                t1 = time.perf_counter()
                self._last_sock_ms = (t1 - t0) * 1000.0
                self._last_parse_ms = 0.0
                return out

            out = {}
            for a, v in zip(addrs, values):
                try:
                    if hasattr(v, "tolist"):
                        v = v.tolist()
                    out[a] = v
                except Exception:
                    out[a] = None
        t1 = time.perf_counter()
        self._last_sock_ms = (t1 - t0) * 1000.0
        self._last_parse_ms = 0.0
        return out

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


class PlcApiDriver(PlcDriver):
    def __init__(self, cfg):
        self.instance_name = cfg.instance if cfg.instance else "ros2_"
        self.instance = None
        self._last_sock_ms = 0.0
        self._last_parse_ms = 0.0

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
        t0 = time.perf_counter()
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
        t1 = time.perf_counter()
        self._last_sock_ms = (t1 - t0) * 1000.0
        self._last_parse_ms = 0.0
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


class PlcRosBridge(Node):
    def __init__(self):
        super().__init__("plc_ros_bridge")
        self.declare_parameter("config", "plc_ros_bridge.yaml")

        cfg_path = self.get_parameter("config").value
        try:
            self.cfg, self._cfg_hash = load_bridge_config(cfg_path)
        except ConfigError as e:
            self.get_logger().fatal(f"config validation failed: {e}")
            raise

        log_cfg = {"logging": self.cfg.logging} if self.cfg.logging else {}
        self.log = setup_logging(
            log_cfg, logger_name="plc_ros_bridge", ros_logger=self.get_logger()
        )

        self.log.info(
            "config_loaded",
            extra={
                "path": str(cfg_path),
                "hash": self._cfg_hash,
                "schema_version": self.cfg.schema_version,
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "node": self.get_name(),
            },
        )

        if not self.cfg.estop_forwarding_required:
            self.log.critical(
                "estop_forwarding_disabled",
                extra={"note": "PLC E-stop is NOT forwarded to ROS; "
                                 "simulation/test use only"},
            )

        driver_name = self.cfg.plc.driver
        self.driver_name = driver_name
        self.driver = DRIVERS[driver_name](self.cfg.plc)

        vars_cfg = self.cfg.vars
        ns = vars_cfg.ns
        db = vars_cfg.db
        opcua_tpl = vars_cfg.opcua_addr_template

        def make_addr(raw):
            if raw.startswith("ns="):
                return raw
            if driver_name == "opcua":
                return opcua_tpl.format(ns=ns, raw=raw)
            if not db:
                raise ValueError(
                    f"vars.db is empty; cannot build address for signal "
                    f"'{raw}'. Set vars.db in the YAML."
                )
            return f'{db}.{raw}'

        self.sig = {}
        for name, s in self.cfg.signals.items():
            self.sig[name] = {
                "addr": make_addr(s.addr),
                "type": s.type,
                "dir":  s.dir,
            }
        self.read_names = [n for n, s in self.sig.items() if s["dir"] == "read"]

        if hasattr(self.driver, "set_signal_types"):
            self.driver.set_signal_types({
                s["addr"]: s["type"] for s in self.sig.values()
            })

        robot = self.cfg.robot
        self.joints  = robot.joints
        self.cmd_ang = robot.cmd_ang
        self.act_ang = robot.act_ang
        self.cmd_ack = robot.cmd_ack
        self.feedback_valid = robot.feedback_valid
        hb = self.cfg.heartbeat
        self.hb_counter = hb.counter
        self.hb_ack     = hb.ack

        state_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pubs = [
            (p, self.create_publisher(MSG_TYPES[p.type], p.topic, state_qos))
            for p in self.cfg.publish
        ]

        self.pub_estop = None
        for p, pub in self.pubs:
            if p.topic == "/plc/emergency_stop":
                self.pub_estop = pub
                break
        if self.pub_estop is None:
            self.pub_estop = self.create_publisher(
                Bool, "/plc/emergency_stop", state_qos)

        self.pub_conn  = self.create_publisher(
            Bool, "/plc/connected", state_qos)

        self._estop_from_plc = any(
            p.topic == "/plc/emergency_stop" and p.type == "bool"
            for p in self.cfg.publish
        )
        self.pub_cmd_ang = None
        if self.cmd_ang:
            self.pub_cmd_ang = self.create_publisher(
                JointState, robot.cmd_ang_topic, PUB_STREAM_QOS
            )

        self._probe = (
            _LatencyProbe(self.cfg.latency_probe_file)
            if self.cfg.latency_probe else None
        )
        self._js_tag = ""
        self._js_recv_ns = 0

        # ← FIX: يُحدَّث في نهاية كل tick()، ليصبح origin_ns للدورة التالية
        self._last_tick_end_ns = None

        self.joint_idx = {j: i for i, j in enumerate(self.joints)}
        self._js_lock = threading.Lock()
        self._js_rad = [0.0] * len(self.joints)
        self._js_ok = False
        self._last_js_t = 0.0
        if self.act_ang:
            self.create_subscription(
                JointState, robot.joint_states_topic,
                self.on_joint_states, SUB_STREAM_QOS,
            )

        self.connected = False
        self._last_try = 0.0
        self._backoff = 1.0
        self._backoff_min = 1.0
        self._backoff_max = self.cfg.backoff_max
        self._last_ack = None
        self._warned = {}
        self._warn_cooldown = self.cfg.warn_cooldown

        self._watchdog_timeout = self.cfg.watchdog_timeout
        self._last_tick_ok = time.monotonic()
        self._watchdog_fired = False
        self._watchdog_stop = threading.Event()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True, name="plc_watchdog"
        )
        self._watchdog_thread.start()

        self._last_pub = {}
        self._last_cmd_ang = None

        self._tick_count = 0
        self._slow_count = 0
        self._slow_sock_sum_ms = 0.0
        self._slow_gap_sum_ms = 0.0
        self._stats_every = max(1, int(self.cfg.poll_hz * 10))
        self._stats_t0 = time.monotonic()

        self._diag_updater = None
        if _HAS_DIAG:
            self._diag_updater = Updater(self)
            self._diag_updater.setHardwareID("plc_ros_bridge")
            self._diag_updater.add(
                FunctionDiagnosticTask("PLC link", self._diag)
            )
            self.create_timer(1.0, self._diag_updater.update)

        poll_hz = self.cfg.poll_hz
        self._poll_period = 1.0 / poll_hz
        self._io_stop = threading.Event()
        self._io_thread = threading.Thread(
            target=self._io_loop, daemon=True, name="plc_io"
        )
        self._io_thread.start()

        self.log.info(
            "bridge_started",
            extra={"driver": driver_name, "poll_hz": poll_hz},
        )

    def on_joint_states(self, msg):
        with self._js_lock:
            if self._probe is not None:
                self._js_tag = msg.header.frame_id
                self._js_recv_ns = time.monotonic_ns()
            for name, pos in zip(msg.name, msg.position):
                i = self.joint_idx.get(name)
                if i is not None:
                    self._js_rad[i] = float(pos)
            self._last_js_t = time.monotonic()
            self._js_ok = True

    def _set_connected(self, state):
        if state == self.connected:
            return
        if state:
            self._last_tick_ok = time.monotonic()
            self._last_pub.clear()
            self._last_cmd_ang = None
            self.log.info(
                "plc_connected",
                extra={"driver": self.driver_name, "backoff_s": self._backoff},
            )
            if self._estop_from_plc:
                self.log.info("plc_reconnected_estop_from_plc")
            else:
                self.pub_estop.publish(Bool(data=False))
                self.log.info("plc_reconnected_estop_cleared")
        else:
            self.log.warning(
                "plc_disconnected",
                extra={"driver": self.driver_name},
            )
            self.pub_estop.publish(Bool(data=True))
            self._last_pub["/plc/emergency_stop"] = True
            self.log.warning("plc_lost_estop_published")
        self.connected = state
        self.pub_conn.publish(Bool(data=state))

    def _try_connect(self):
        now = time.monotonic()
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
            now = time.monotonic()
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

    def tick(self):
        if not self.connected and not self._try_connect():
            return
        tick_id = int(time.monotonic() * 1000) % 1000000
        t_rs = time.monotonic_ns()
        try:
            raw = self.driver.read(
                [self.sig[n]["addr"] for n in self.read_names]
            )
            t_rd = time.monotonic_ns()
            vals = {n: self._get(n, raw) for n in self.read_names}
            read_ms = (t_rd - t_rs) / 1e6
            sock_ms = getattr(self.driver, "_last_sock_ms", 0.0)
            gap_ms = max(0.0, read_ms - sock_ms)

            if read_ms > 200.0:
                self._slow_count += 1
                self._slow_sock_sum_ms += sock_ms
                self._slow_gap_sum_ms += gap_ms
                self.log.warning(
                    "slow_read",
                    extra={
                        "tick_id": tick_id,
                        "read_ms": round(read_ms, 1),
                        "sock_ms": round(sock_ms, 2),
                        "gap_ms": round(gap_ms, 1),
                    },
                )

            self._last_tick_ok = time.monotonic()
            if self._watchdog_fired:
                self._last_pub.clear()
                self._watchdog_fired = False

            self._publish(vals, t_rs, t_rd)

            writes = {}
            pending_ack = None
            probe_tag, probe_recv, act_written = "", 0, False

            if self.hb_counter and self.hb_ack:
                hb = vals.get(self.hb_counter)
                if hb is not None and hb != self._last_ack:
                    writes[self.sig[self.hb_ack]["addr"]] = int(hb)
                    pending_ack = int(hb)

            with self._js_lock:
                js_fresh = (
                    self._js_ok
                    and self._last_js_t > 0
                    and (time.monotonic() - self._last_js_t) <= JS_STALE_AGE_S
                )
                if self.act_ang and js_fresh:
                    act_written = True
                    probe_tag, probe_recv = self._js_tag, self._js_recv_ns
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
                    if self.feedback_valid:
                        writes[self.sig[self.feedback_valid]["addr"]] = True
                elif self.act_ang and not js_fresh:
                    if self.cmd_ack:
                        writes[self.sig[self.cmd_ack]["addr"]] = False
                    if self.feedback_valid:
                        writes[self.sig[self.feedback_valid]["addr"]] = False

            if writes:
                failed = self.driver.write(writes)
                if (self._probe is not None and act_written and probe_tag
                        and not failed):
                    self._probe.record(
                        probe_tag, probe_recv, time.monotonic_ns())
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

            self._tick_count += 1
            if self._tick_count % self._stats_every == 0:
                avg_sock = (self._slow_sock_sum_ms / self._slow_count
                            if self._slow_count else 0.0)
                avg_gap = (self._slow_gap_sum_ms / self._slow_count
                           if self._slow_count else 0.0)
                self.log.info(
                    "tick_stats",
                    extra={
                        "ticks": self._tick_count,
                        "rate_hz": self._measured_rate_hz(),
                        "slow": self._slow_count,
                        "slow_pct": round(
                            100.0 * self._slow_count / self._tick_count, 2),
                        "avg_slow_sock_ms": round(avg_sock, 2),
                        "avg_slow_gap_ms": round(avg_gap, 1),
                    },
                )
                self._slow_count = 0
                self._slow_sock_sum_ms = 0.0
                self._slow_gap_sum_ms = 0.0

            # ← FIX: يُسجَّل في نهاية الدورة، ليصبح origin_ns للدورة التالية
            self._last_tick_end_ns = time.monotonic_ns()

        except Exception as e:
            self.log.error(
                "plc_link_error",
                extra={"tick_id": tick_id, "error": str(e),
                       "driver": self.driver_name},
            )
            self.driver.close()
            self._set_connected(False)
            self._last_try = 0.0

    def _warn_estop_unreadable(self, signal):
        now = time.monotonic()
        if now - self._warned.get("estop_unreadable", 0.0) > self._warn_cooldown:
            self._warned["estop_unreadable"] = now
            self.log.error(
                "estop_unreadable_failsafe_active",
                extra={"signal": signal},
            )

    def _publish(self, vals, t_rs=None, t_rd=None):
        for p, pub in self.pubs:
            v = vals.get(p.signal)
            if p.topic == ESTOP_TOPIC:
                if v is None:
                    self._warn_estop_unreadable(p.signal)
                v = resolve_estop(v)
            if v is None:
                continue
            cast_v = CAST[p.type](v)
            key = p.topic
            if self._last_pub.get(key) == cast_v:
                continue
            self._last_pub[key] = cast_v
            pub.publish(MSG_TYPES[p.type](data=cast_v))

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
            self._last_cmd_ang = list(ang)

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(self.joints)
            js.position = ang
            if self._probe is not None and t_rs is not None:
                # ← FIX: origin_ns = نهاية الدورة السابقة (queue delay الحقيقي)
                origin_ns = self._last_tick_end_ns if self._last_tick_end_ns is not None else t_rs
                js.header.frame_id = (
                    f"p;{origin_ns};{t_rs};{t_rd};{time.monotonic_ns()}")
            self.pub_cmd_ang.publish(js)

    def _measured_rate_hz(self):
        now = time.monotonic()
        dt = now - self._stats_t0
        self._stats_t0 = now
        return round(self._stats_every / dt, 1) if dt > 0 else 0.0

    def _io_loop(self):
        next_t = time.monotonic()
        while not self._io_stop.is_set():
            try:
                self.tick()
            except Exception as e:
                try:
                    self.log.error("io_loop_error", extra={"error": str(e)})
                except Exception:
                    pass
            next_t += self._poll_period
            delay = next_t - time.monotonic()
            if delay > 0:
                self._io_stop.wait(delay)
            else:
                next_t = time.monotonic()

    def _watchdog_loop(self):
        while not self._watchdog_stop.wait(0.5):
            try:
                self._check_watchdog()
            except Exception as e:
                try:
                    self.log.error("watchdog_error", extra={"error": str(e)})
                except Exception:
                    pass

    def _check_watchdog(self):
        now = time.monotonic()
        if self.connected and now - self._last_tick_ok > self._watchdog_timeout:
            stalled = round(now - self._last_tick_ok, 2)
            self.log.error("watchdog_tick_stalled",
                           extra={"stalled_s": stalled})
            self.pub_estop.publish(Bool(data=True))
            self._last_pub["/plc/emergency_stop"] = True
            self._watchdog_fired = True

    def _diag(self, stat):
        if not _HAS_DIAG:
            return stat
        if self.connected:
            stat.summary(DiagnosticStatus.OK, "connected")
        else:
            stat.summary(DiagnosticStatus.ERROR, "disconnected")
        stat.add("driver", self.driver_name)
        stat.add("backoff_s", f"{self._backoff:.1f}")
        stat.add("last_tick_age_s",
                 f"{time.monotonic() - self._last_tick_ok:.2f}")
        stat.add("config_hash", self._cfg_hash[:12] + "…")
        stat.add("schema_version", self.cfg.schema_version)
        return stat

    def destroy_node(self):
        self._io_stop.set()
        try:
            self._io_thread.join(timeout=2.0)
        except Exception:
            pass
        self._watchdog_stop.set()
        try:
            self._watchdog_thread.join(timeout=1.0)
        except Exception:
            pass
        if getattr(self, "_probe", None) is not None:
            self._probe.close()
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
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
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