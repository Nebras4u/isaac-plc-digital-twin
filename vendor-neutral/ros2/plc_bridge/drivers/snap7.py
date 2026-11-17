import time

from plc_bridge.drivers.base import PlcDriver
from plc_bridge.wire import _wire_type_check

try:
    import snap7
    from snap7 import util as s7util
    _HAS_SNAP7 = True
except Exception:
    _HAS_SNAP7 = False
    snap7 = None
    s7util = None


class Snap7Driver(PlcDriver):
    _SIZE = {"bool": 1, "int16": 2, "int32": 4, "lreal": 8}

    def __init__(self, cfg):
        if not _HAS_SNAP7:
            raise RuntimeError(
                "python-snap7 is not installed. "
                "Install with: pip install python-snap7")
        self.host = cfg.host
        self.rack = getattr(cfg, "rack", 0)
        self.slot = getattr(cfg, "slot", 1)
        self.timeout = cfg.timeout
        self.client = None
        self._addr_meta = {}
        self._last_sock_ms = 0.0
        self._last_parse_ms = 0.0

    def set_signal_addressing(self, meta):
        self._addr_meta = dict(meta)

    def connect(self):
        self.close()
        self.client = snap7.client.Client()
        self.client.connect(self.host, self.rack, self.slot)
        if not self.client.get_connected():
            raise ConnectionError(f"Snap7 cannot connect to {self.host}")

    def close(self):
        if self.client is not None:
            try:
                if self.client.get_connected():
                    self.client.disconnect()
            except Exception:
                pass
        self.client = None

    def _resolve_s7_type(self, meta):
        st = meta.get("s7_type")
        if st in self._SIZE:
            return st
        t = meta.get("type", "float")
        if t == "bool": return "bool"
        if t == "int": return "int16"
        if t == "float": return "lreal"
        return "lreal"

    def read(self, addrs):
        if not addrs:
            return {}
        t0 = time.perf_counter()
        out = {}
        try:
            groups = {}
            for a in addrs:
                meta = self._addr_meta.get(a)
                if meta is None:
                    out[a] = None
                    continue
                db = meta["db"]
                st = self._resolve_s7_type(meta)
                size = self._SIZE[st]
                byte_off = meta["byte"]
                g = groups.setdefault(db, {"lo": byte_off,
                                           "hi": byte_off + size,
                                           "items": []})
                if byte_off < g["lo"]:
                    g["lo"] = byte_off
                if byte_off + size > g["hi"]:
                    g["hi"] = byte_off + size
                g["items"].append((a, meta, st, size, byte_off))
            for db, g in groups.items():
                span = g["hi"] - g["lo"]
                data = self.client.db_read(db, g["lo"], span)
                for (a, meta, st, size, byte_off) in g["items"]:
                    rel = byte_off - g["lo"]
                    try:
                        out[a] = self._decode(data, rel, meta, st)
                    except Exception as e:
                        out[a] = f"ERR:{e}"
        except Exception as e:
            for a in addrs:
                out.setdefault(a, f"ERR:{e}")
        t1 = time.perf_counter()
        self._last_sock_ms = (t1 - t0) * 1000.0
        self._last_parse_ms = 0.0
        return {
            a: _wire_type_check(
                (self._addr_meta.get(a) or {}).get("type"), out.get(a))
            for a in addrs
        }

    @staticmethod
    def _decode(data, rel, meta, st):
        bit = meta.get("bit", 0) or 0
        if st == "bool":
            return bool(s7util.get_bool(data, rel, bit))
        if st == "int16":
            return int(s7util.get_int(data, rel))
        if st == "int32":
            return int(s7util.get_dint(data, rel))
        if st == "lreal":
            return float(s7util.get_lreal(data, rel))
        raise ValueError(f"unknown s7_type {st!r}")

    def write(self, values):
        if not values:
            return []
        failed = []
        for a, v in values.items():
            meta = self._addr_meta.get(a)
            if meta is None:
                failed.append(a)
                continue
            try:
                self._write_one(meta, v)
            except Exception as e:
                print(f"snap7 write {a} failed: {e}")
                failed.append(a)
        return failed

    def _write_one(self, meta, value):
        db = meta["db"]
        byte_off = meta["byte"]
        bit = meta.get("bit", 0) or 0
        st = self._resolve_s7_type(meta)
        if st == "bool":
            data = self.client.db_read(db, byte_off, 1)
            s7util.set_bool(data, 0, bit, bool(value))
            self.client.db_write(db, byte_off, data)
            return
        if st == "int16":
            data = bytearray(2)
            s7util.set_int(data, 0, int(value))
            self.client.db_write(db, byte_off, data)
            return
        if st == "int32":
            data = bytearray(4)
            s7util.set_dint(data, 0, int(value))
            self.client.db_write(db, byte_off, data)
            return
        if st == "lreal":
            data = bytearray(8)
            s7util.set_lreal(data, 0, float(value))
            self.client.db_write(db, byte_off, data)
            return
        raise ValueError(f"unsupported s7_type {st!r}")
