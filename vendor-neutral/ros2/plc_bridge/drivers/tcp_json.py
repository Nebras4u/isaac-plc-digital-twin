import json
import socket
import time

from plc_bridge.constants import _SERVER_TYPE_MAP
from plc_bridge.drivers.base import PlcDriver
from plc_bridge.wire import _wire_type_check


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
        self.types = {}

    def set_signal_types(self, types):
        self.types = dict(types)

    def connect(self):
        self.close()
        self.sock = socket.create_connection(
            (self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        try:
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass
        self.buf = b""
        self._last_seq = 0

    @staticmethod
    def _reject_constant(name):
        raise ValueError(f"non-finite constant from PLC: {name}")

    def _request(self, req):
        self.sock.sendall((json.dumps(req, allow_nan=False) + "\n").encode())
        while b"\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("server closed the connection")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        resp = json.loads(line.decode("utf-8"),
                          parse_constant=self._reject_constant)
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

    def sync(self, read_addrs, write_values):
        req = {"cmd": "sync", "read": list(read_addrs)}
        if write_values:
            req["write"] = write_values
        t0 = time.perf_counter()
        resp = self._request(req)
        t1 = time.perf_counter()
        out = {}
        for a in read_addrs:
            v = resp.get("values", {}).get(a)
            if isinstance(v, str) and v.startswith("ERR:"):
                out[a] = v
            else:
                out[a] = _wire_type_check(self.types.get(a), v)
        failed = []
        if write_values:
            for a in write_values:
                if not resp.get("written", {}).get(a, False):
                    failed.append(a)
        self._last_sock_ms = (t1 - t0) * 1000.0
        self._last_parse_ms = 0.0
        return out, failed

    def read(self, addrs):
        t0 = time.perf_counter()
        resp = self._request({"cmd": "read", "tags": list(addrs)})
        t1 = time.perf_counter()
        if "values" not in resp:
            raise RuntimeError(f"bad read reply: {resp}")
        out = {}
        for a in addrs:
            v = resp["values"].get(a)
            if isinstance(v, str) and v.startswith("ERR:"):
                out[a] = v
            else:
                out[a] = _wire_type_check(self.types.get(a), v)
        t2 = time.perf_counter()
        self._last_sock_ms = (t1 - t0) * 1000.0
        self._last_parse_ms = (t2 - t1) * 1000.0
        return out

    def write(self, values):
        resp = self._request({"cmd": "write", "tags": values})
        if "written" not in resp:
            raise RuntimeError(f"bad write reply: {resp}")
        return [a for a in values if not resp["written"].get(a, False)]

    def get_server_types(self):
        try:
            resp = self._request({"cmd": "types"})
        except Exception:
            return None
        if "error" in resp:
            return None
        out = {}
        for k, v in resp.get("types", {}).items():
            out[k] = _SERVER_TYPE_MAP.get(v, v)
        for k, v in resp.get("arrays", {}).items():
            out[k] = _SERVER_TYPE_MAP.get(v, v)
        return out

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        self.sock = None
