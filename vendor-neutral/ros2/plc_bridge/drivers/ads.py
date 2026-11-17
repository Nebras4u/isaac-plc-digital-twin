import time

from plc_bridge.drivers.base import PlcDriver
from plc_bridge.wire import _wire_type_check


class AdsDriver(PlcDriver):
    def __init__(self, cfg):
        self.ams_net_id = cfg.ams_net_id
        self.ams_port   = cfg.ams_port
        self.host       = cfg.host
        self.local_ams  = cfg.local_ams_net_id
        self.plc        = None
        self.types      = {}
        self.lengths    = {}
        self._last_sock_ms  = 0.0
        self._last_parse_ms = 0.0

    def set_signal_types(self, types): self.types = dict(types)
    def set_signal_lengths(self, lengths): self.lengths = dict(lengths)

    def connect(self):
        import pyads
        self.close()
        pyads.open_port()
        pyads.set_local_address(self.local_ams)
        self.plc = pyads.Connection(self.ams_net_id, self.ams_port, self.host)
        self.plc.open()

    def close(self):
        if self.plc is not None:
            try:
                if self.plc.is_open:
                    self.plc.close()
            except Exception:
                pass
        self.plc = None

    @staticmethod
    def _plc_type(t):
        import pyads
        return {
            "bool":   pyads.PLCTYPE_BOOL,
            "int":    pyads.PLCTYPE_INT,
            "float":  pyads.PLCTYPE_LREAL,
            "string": pyads.PLCTYPE_STRING,
        }[t]

    def _type_for(self, addr):
        t = self.types.get(addr, "float")
        n = self.lengths.get(addr, 1)
        pt = self._plc_type(t)
        return pt * n if n > 1 else pt

    @staticmethod
    def _unpack(v):
        if v is None or isinstance(v, (str, bytes)):
            return v
        if hasattr(v, "__iter__"):
            try:
                return list(v)
            except TypeError:
                return v
        return v

    def read(self, addrs):
        if not addrs:
            return {}
        t0  = time.perf_counter()
        req = {a: self._type_for(a) for a in addrs}
        try:
            values = self.plc.read_list_by_name(req)
            if isinstance(values, dict):
                result = values
            else:
                result = dict(zip(req.keys(), values))
        except Exception:
            result = {}
            for a, pt in req.items():
                try:
                    result[a] = self.plc.read_by_name(a, pt)
                except Exception as e:
                    result[a] = f"ERR:{e}"
        t1 = time.perf_counter()
        self._last_sock_ms  = (t1 - t0) * 1000.0
        self._last_parse_ms = 0.0
        return {
            a: _wire_type_check(self.types.get(a), self._unpack(result.get(a)))
            for a in addrs
        }

    def write(self, values):
        if not values:
            return []
        req = {a: (self._type_for(a), v) for a, v in values.items()}
        try:
            self.plc.write_list_by_name(req)
            return []
        except Exception:
            failed = []
            for a, (pt, v) in req.items():
                try:
                    self.plc.write_by_name(a, v, pt)
                except Exception:
                    failed.append(a)
            return failed

    def get_server_types(self):
        return None
