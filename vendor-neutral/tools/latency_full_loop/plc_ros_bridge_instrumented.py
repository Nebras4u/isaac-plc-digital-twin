#!/usr/bin/env python3
"""
plc_ros_bridge_instrumented.py
==============================
PLC <-> ROS 2 bridge مع قياس داخلي كامل.

يضيف فوق plc_ros_bridge.py الأصلي:
  - قياس زمن OPC UA read  → /plc/read_latency_ms
  - قياس زمن OPC UA write → /plc/write_latency_ms
  - قياس زمن الدورة الكاملة → /plc/tick_period_ms
  - قياس زمن publish لكل tick → /plc/publish_latency_ms
  - نشر /plc/cmd_ang مع timestamp للقراءة في header.frame_id
    (بصيغة "mono:<seconds>" لاستخدامه في latency_full_loop.py)
  - تقرير داخلي كل 10 ثواني على stdout
  - حفظ إحصاءات كل 60 ثانية في plc_bridge_internal.csv

Run:
  python3 plc_ros_bridge_instrumented.py --ros-args -p config:=plc_ros_bridge.yaml
"""
import abc
import csv
import json
import math
import socket
import threading
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32, String


# ==========================================================================
# Message type maps (نفس الأصلية)
# ==========================================================================
MSG_TYPES = {"bool": Bool, "int": Int32, "float": Float64, "string": String}
CAST = {"bool": bool, "int": int, "float": float, "string": str}


# ==========================================================================
# 1. Drivers (نفس الأصلية — بدون تغيير)
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
        self.sock = socket.create_connection(
            (self.host, self.port), timeout=self.timeout
        )
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
    """OPC UA client (pip install asyncua).
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
    """Direct PLCSIM Advanced API (Windows only). Fastest."""

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
        if "Bool" in t:  return bool(self.instance.ReadBool(addr))
        if "LReal" in t: return float(self.instance.ReadDouble(addr))
        if "Real" in t:  return float(self.instance.ReadFloat(addr))
        if "DInt" in t:  return int(self.instance.ReadInt32(addr))
        if "Int" in t:   return int(self.instance.ReadInt16(addr))
        return None

    def _write_one(self, addr, value):
        info = self.instance.GetTagInfo(addr)
        t = str(info.DataType)
        if "Bool" in t:   self.instance.WriteBool(addr, bool(value))
        elif "LReal" in t: self.instance.WriteDouble(addr, float(value))
        elif "Real" in t:  self.instance.WriteFloat(addr, float(value))
        elif "DInt" in t:  self.instance.WriteInt32(addr, int(value))
        elif "Int" in t:   self.instance.WriteInt16(addr, int(value))

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
        self.instance = None


DRIVERS = {
    "tcp_json": TcpJsonDriver,
    "opcua":    OpcUaDriver,
    "plc_api":  PlcApiDriver,
}


# ==========================================================================
# 2. Bridge with instrumentation
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

        # ---- Address prefix ----
        ns = cfg.get("vars", {}).get("ns", "")
        db = cfg.get("vars", {}).get("db", "")

        def make_addr(raw):
            if raw.startswith("ns=") or ("." in raw):
                return raw
            if driver_name == "opcua":
                return f'{ns}."{raw}"'
            return f'{db}.{raw}'

        # ---- Signals ----
        self.sig = {}
        for name, s in cfg["signals"].items():
            raw_addr = s["addr"]
            self.sig[name] = {
                "addr": make_addr(raw_addr),
                "type": s.get("type", "float"),
                "dir":  s.get("dir", "read"),
            }
        self.read_names = [n for n, s in self.sig.items() if s["dir"] == "read"]

        # ---- Robot / heartbeat ----
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

        # ---- Publishers (أصلية) ----
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

        # ---- ⚡ Publishers تشخيصية جديدة ----
        self.pub_read_ms    = self.create_publisher(
            Float64, "/plc/read_latency_ms", 10)
        self.pub_write_ms   = self.create_publisher(
            Float64, "/plc/write_latency_ms", 10)
        self.pub_tick_ms    = self.create_publisher(
            Float64, "/plc/tick_period_ms", 10)
        self.pub_publish_ms = self.create_publisher(
            Float64, "/plc/publish_latency_ms", 10)

        # ---- Joint states feedback (أصلية) ----
        self.joint_idx = {j: i for i, j in enumerate(self.joints)}
        self._js_lock = threading.Lock()
        self._js_rad = [0.0] * len(self.joints)
        self._js_ok = False
        if self.act_ang:
            self.create_subscription(
                JointState, robot.get("joint_states_topic", "/joint_states"),
                self.on_joint_states, 10,
            )

        # ---- State ----
        self.connected = False
        self._last_try = 0.0
        self._last_ack = None
        self._warned = set()

        # ---- ⚡ إحصاءات داخلية (نوافذ متحركة) ----
        self._read_times    = []
        self._write_times   = []
        self._tick_times    = []
        self._publish_times = []
        self._last_tick_t   = 0.0
        self._stats_start   = time.monotonic()

        # ---- ⚡ CSV داخلي ----
        self._csv_file = open("plc_bridge_internal.csv", "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "elapsed_s", "n_read", "read_mean_ms", "read_p95_ms", "read_max_ms",
            "n_write", "write_mean_ms", "write_p95_ms", "write_max_ms",
            "n_tick", "tick_mean_ms", "tick_p95_ms", "tick_max_ms",
            "n_pub",  "publish_mean_ms", "publish_p95_ms",
        ])
        self._csv_file.flush()

        # ---- Timers ----
        poll_hz = cfg.get("poll_hz", 20.0)
        self.create_timer(1.0 / poll_hz, self.tick)
        self.create_timer(10.0, self._report_internal_stats)
        self.create_timer(60.0, self._dump_stats_to_csv)

        self.get_logger().info(
            f"plc_ros_bridge (INSTRUMENTED) started | "
            f"driver={driver_name} | poll={poll_hz}Hz"
        )

    # ======================================================================
    # on_joint_states
    # ======================================================================
    def on_joint_states(self, msg):
        with self._js_lock:
            for name, pos in zip(msg.name, msg.position):
                i = self.joint_idx.get(name)
                if i is not None:
                    self._js_rad[i] = float(pos)
            self._js_ok = True

    # ======================================================================
    # Connection helpers
    # ======================================================================
    def _set_connected(self, state):
        if state != self.connected:
            self.get_logger().info(
                "PLC connected" if state else "PLC disconnected"
            )
        self.connected = state
        self.pub_conn.publish(Bool(data=state))
        if not state:
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

    # ======================================================================
    # _get
    # ======================================================================
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

            # array_float يُعالج أعلاه، لكن لو جاء scalar:
            if s["type"] == "array_float":
                return [float(v)]

            return CAST[s["type"]](v)
        except (TypeError, ValueError, KeyError) as e:
            if name not in self._warned:
                self._warned.add(name)
                self.get_logger().warn(
                    f"signal '{name}' ({s['addr']}) = {v!r} -> {e}"
                )
            return None

    # ======================================================================
    # tick (القلب النابض مع القياس)
    # ======================================================================
    def tick(self):
        t_tick_start = time.monotonic()

        # ---- قياس زمن الدورة ----
        if self._last_tick_t > 0:
            period_ms = (t_tick_start - self._last_tick_t) * 1000.0
            self._tick_times.append(period_ms)
            self.pub_tick_ms.publish(Float64(data=period_ms))
        self._last_tick_t = t_tick_start

        if not self.connected and not self._try_connect():
            return

        try:
            # ==============================================================
            # 1) OPC UA READ
            # ==============================================================
            t_read_start = time.monotonic()
            raw = self.driver.read(
                [self.sig[n]["addr"] for n in self.read_names]
            )
            t_read_done = time.monotonic()
            read_ms = (t_read_done - t_read_start) * 1000.0
            self._read_times.append(read_ms)
            self.pub_read_ms.publish(Float64(data=read_ms))

            vals = {n: self._get(n, raw) for n in self.read_names}

            # ==============================================================
            # 2) PUBLISH (مع قياس)
            # ==============================================================
            t_pub_start = time.monotonic()
            self._publish(vals, cmd_stamp_mono=t_read_done)
            t_pub_done = time.monotonic()
            pub_ms = (t_pub_done - t_pub_start) * 1000.0
            self._publish_times.append(pub_ms)
            self.pub_publish_ms.publish(Float64(data=pub_ms))

            # ==============================================================
            # 3) تجهيز الكتابات
            # ==============================================================
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

            # ==============================================================
            # 4) OPC UA WRITE
            # ==============================================================
            if writes:
                t_write_start = time.monotonic()
                failed = self.driver.write(writes)
                t_write_done = time.monotonic()
                write_ms = (t_write_done - t_write_start) * 1000.0
                self._write_times.append(write_ms)
                self.pub_write_ms.publish(Float64(data=write_ms))

                for a in failed:
                    self.get_logger().warn(f"write failed: {a}")
                if pending_ack is not None and \
                   self.sig[self.hb_ack]["addr"] not in failed:
                    self._last_ack = pending_ack

        except Exception as e:
            self.get_logger().error(f"PLC link error: {e}")
            self.driver.close()
            self._set_connected(False)

    # ======================================================================
    # _publish
    # ======================================================================
    def _publish(self, vals, cmd_stamp_mono=None):
        # إشارات مفردة
        for p, pub in self.pubs:
            v = vals.get(p["signal"])
            if v is not None:
                pub.publish(MSG_TYPES[p["type"]](data=CAST[p["type"]](v)))

        # /plc/cmd_ang
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
            # ⚡ نحمل monotonic time في frame_id (صيغة "mono:<sec>")
            if cmd_stamp_mono is not None:
                js.header.frame_id = f"mono:{cmd_stamp_mono:.6f}"
            self.pub_cmd_ang.publish(js)

    # ======================================================================
    # تقرير داخلي كل 10 ثواني
    # ======================================================================
    def _report_internal_stats(self):
        if not self._read_times:
            return

        r = np.array(self._read_times)
        w = np.array(self._write_times) if self._write_times else np.array([0.0])
        t = np.array(self._tick_times)  if self._tick_times  else np.array([0.0])
        p = np.array(self._publish_times) if self._publish_times else np.array([0.0])

        self.get_logger().info(
            f"[STATS] read  n={len(r):4d} "
            f"mean={r.mean():5.2f} p95={np.percentile(r, 95):5.2f} "
            f"max={r.max():5.2f} ms"
        )
        if self._write_times:
            self.get_logger().info(
                f"[STATS] write n={len(w):4d} "
                f"mean={w.mean():5.2f} p95={np.percentile(w, 95):5.2f} "
                f"max={w.max():5.2f} ms"
            )
        if self._tick_times:
            self.get_logger().info(
                f"[STATS] tick  n={len(t):4d} "
                f"mean={t.mean():5.2f} p95={np.percentile(t, 95):5.2f} "
                f"max={t.max():5.2f} ms"
            )
        if self._publish_times:
            self.get_logger().info(
                f"[STATS] pub   n={len(p):4d} "
                f"mean={p.mean():5.2f} p95={np.percentile(p, 95):5.2f} "
                f"max={p.max():5.2f} ms"
            )

        # ---- امسح النوافذ ----
        self._read_times.clear()
        self._write_times.clear()
        self._tick_times.clear()
        self._publish_times.clear()

    # ======================================================================
    # حفظ إحصاءات كل 60 ثانية في CSV
    # ======================================================================
    def _dump_stats_to_csv(self):
        if not self._read_times and not self._write_times:
            return

        elapsed = time.monotonic() - self._stats_start

        def stats(arr):
            if not arr:
                return (0, 0.0, 0.0, 0.0)
            a = np.array(arr)
            return (len(a), float(a.mean()),
                    float(np.percentile(a, 95)), float(a.max()))

        r = stats(self._read_times)
        w = stats(self._write_times)
        t = stats(self._tick_times)
        p = stats(self._publish_times)

        self._csv_writer.writerow([
            f"{elapsed:.1f}",
            r[0], f"{r[1]:.2f}", f"{r[2]:.2f}", f"{r[3]:.2f}",
            w[0], f"{w[1]:.2f}", f"{w[2]:.2f}", f"{w[3]:.2f}",
            t[0], f"{t[1]:.2f}", f"{t[2]:.2f}", f"{t[3]:.2f}",
            p[0], f"{p[1]:.2f}", f"{p[2]:.2f}",
        ])
        self._csv_file.flush()

    # ======================================================================
    def destroy_node(self):
        try:
            self._csv_file.close()
        except Exception:
            pass
        self.driver.close()
        super().destroy_node()


# ==========================================================================
# main
# ==========================================================================
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