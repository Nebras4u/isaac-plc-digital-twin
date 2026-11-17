import threading
import time
from collections import defaultdict
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from bridge_config import load_bridge_config, ConfigError
from bridge_logger import setup_logging

from plc_bridge.constants import (
    MSG_TYPES, PUB_STREAM_QOS, SUB_STREAM_QOS, _HAS_DIAG, Updater,
    FunctionDiagnosticTask,
)
from plc_bridge.drivers import DRIVERS
from plc_bridge.latency import _LatencyProbe
from plc_bridge.lifecycle import PlcRosBridgeLifecycle
from plc_bridge.publish import PlcRosBridgePublish
from plc_bridge.tick import PlcRosBridgeTick
from plc_bridge.validate import PlcRosBridgeValidate


class PlcRosBridge(
    PlcRosBridgeValidate,
    PlcRosBridgeLifecycle,
    PlcRosBridgePublish,
    PlcRosBridgeTick,
    Node,
):
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
            log_cfg, logger_name="plc_ros_bridge", ros_logger=self.get_logger())

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
            if raw.startswith("GVL.") or raw.startswith("MAIN."):
                return raw
            if raw.startswith("DB") and "." in raw:
                return raw
            if driver_name == "opcua":
                return opcua_tpl.format(ns=ns, raw=raw)
            if driver_name == "snap7":
                return raw
            if not db:
                raise ValueError(
                    f"vars.db is empty; cannot build address for signal "
                    f"'{raw}'. Set vars.db in the YAML.")
            return f'{db}.{raw}'

        self.strict_types = bool(getattr(self.cfg, "strict_types", True))

        self.sig = {}
        for name, s in self.cfg.signals.items():
            self.sig[name] = {
                "addr": make_addr(s.addr),
                "type": s.type,
                "dir":  s.dir,
                "length": getattr(s, "length", 1),
                "min":  getattr(s, "min", None),
                "max":  getattr(s, "max", None),
                "strict": getattr(s, "strict", None),
                "reject_non_finite": bool(getattr(s, "reject_non_finite", True)),
                "allow_int_for_float": bool(getattr(s, "allow_int_for_float", True)),
                "db_num":      getattr(s, "db_num", None),
                "byte_offset": getattr(s, "byte_offset", None),
                "bit_offset":  getattr(s, "bit_offset", 0) or 0,
                "s7_type":     getattr(s, "s7_type", None),
            }
        self.read_names = [n for n, s in self.sig.items() if s["dir"] == "read"]
        self._read_addrs = [self.sig[n]["addr"] for n in self.read_names]

        self.driver.set_signal_types(
            {s["addr"]: s["type"] for s in self.sig.values()})
        self.driver.set_signal_lengths(
            {s["addr"]: s["length"] for s in self.sig.values()})

        if driver_name == "snap7":
            meta = {}
            for name, s in self.sig.items():
                if s["db_num"] is None or s["byte_offset"] is None:
                    raise ValueError(
                        f"snap7 signal '{name}' needs db_num and byte_offset "
                        f"(add them in the YAML under signals.{name})")
                meta[s["addr"]] = {
                    "db": s["db_num"],
                    "byte": s["byte_offset"],
                    "bit": s["bit_offset"],
                    "type": s["type"],
                    "s7_type": s["s7_type"],
                }
            self.driver.set_signal_addressing(meta)

        self._stats = defaultdict(lambda: defaultdict(int))

        robot = self.cfg.robot
        self.joints  = robot.joints
        self.cmd_ang = robot.cmd_ang
        self.act_ang = robot.act_ang
        self.cmd_ack = robot.cmd_ack
        self.feedback_valid = robot.feedback_valid
        hb = self.cfg.heartbeat
        self.hb_counter = hb.counter
        self.hb_ack     = hb.ack

        # Measurement state
        self._measure_active = False
        self._measure_start_ns = None
        self._measure_samples = 0
        self._measure_warmup_ticks = 10

        state_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
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

        self.pub_conn = self.create_publisher(
            Bool, "/plc/connected", state_qos)

        self._estop_from_plc = any(
            p.topic == "/plc/emergency_stop" and p.type == "bool"
            for p in self.cfg.publish)
        self.pub_cmd_ang = None
        if self.cmd_ang:
            self.pub_cmd_ang = self.create_publisher(
                JointState, robot.cmd_ang_topic, PUB_STREAM_QOS)

        self._probe = (
            _LatencyProbe(self.cfg.latency_probe_file)
            if self.cfg.latency_probe else None)
        self._js_tag = ""
        self._js_recv_ns = 0
        self._probe_sent = 0

        self._last_tick_end_ns = None
        self._last_written_act_ang = None
        self._last_act_write_t = 0.0
        self._estop_lock = threading.Lock()
        self._deadline_miss = 0
        self._max_overrun_ms = 0.0

        self._pending_writes = {}
        self._pending_ack = None

        self.joint_idx = {j: i for i, j in enumerate(self.joints)}
        self._js_lock = threading.Lock()
        self._js_rad = [0.0] * len(self.joints)
        self._js_ok = False
        self._last_js_t = 0.0
        if self.act_ang:
            self.create_subscription(
                JointState, robot.joint_states_topic,
                self.on_joint_states, SUB_STREAM_QOS)

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
            target=self._watchdog_loop, daemon=True, name="plc_watchdog")
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
                FunctionDiagnosticTask("PLC link", self._diag))
            self.create_timer(1.0, self._diag_updater.update)

        poll_hz = self.cfg.poll_hz
        self._poll_period = 1.0 / poll_hz
        self._io_stop = threading.Event()
        self._io_thread = threading.Thread(
            target=self._io_loop, daemon=True, name="plc_io")
        self._io_thread.start()

        self.log.info(
            "bridge_started",
            extra={"driver": driver_name, "poll_hz": poll_hz,
                   "strict_types": self.strict_types,
                   "measure_warmup_ticks": self._measure_warmup_ticks},
        )


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
