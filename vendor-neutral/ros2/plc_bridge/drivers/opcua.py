import threading
import time

from plc_bridge.drivers.base import PlcDriver
from plc_bridge.wire import _wire_type_check


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

    def set_signal_types(self, types): self.types = dict(types)

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
                return {a: _wire_type_check(self.types.get(a), v)
                        for a, v in out.items()}
            out = {}
            for a, v in zip(addrs, values):
                try:
                    if hasattr(v, "tolist"):
                        v = v.tolist()
                    out[a] = _wire_type_check(self.types.get(a), v)
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
                        [self._node(a) for a in ok_addrs], variants)
                    for a, st in zip(ok_addrs, statuses):
                        if hasattr(st, "is_good") and not st.is_good():
                            failed.append(a)
                except Exception:
                    for a in ok_addrs:
                        try:
                            node = self._node(a)
                            node.write_value(
                                ua.DataValue(self._make_variant(a, values[a])))
                        except Exception:
                            failed.append(a)
            return failed
