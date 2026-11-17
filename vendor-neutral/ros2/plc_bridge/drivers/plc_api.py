import time

from plc_bridge.drivers.base import PlcDriver
from plc_bridge.wire import _wire_type_check


class PlcApiDriver(PlcDriver):
    def __init__(self, cfg):
        self.instance_name = cfg.instance if cfg.instance else "296"
        self.instance = None
        self.types = {}
        self._last_sock_ms = 0.0
        self._last_parse_ms = 0.0

    def set_signal_types(self, types): self.types = dict(types)

    def connect(self):
        import clr
        import os
        api_dir = r"C:\Program Files (x86)\Common Files\Siemens\PLCSIMADV\API\6.0"
        dll = os.path.join(
            api_dir, "Siemens.Simatic.Simulation.Runtime.Api.x64.dll")
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
                v = self._read_one(a)
            except Exception:
                try:
                    elems = []
                    for i in range(1, 17):
                        try:
                            elems.append(self._read_one(f"{a}[{i}]"))
                        except Exception:
                            break
                    v = elems if elems else None
                except Exception:
                    v = None
            out[a] = _wire_type_check(self.types.get(a), v)
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
