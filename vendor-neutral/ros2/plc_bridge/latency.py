import csv
import time


class _LatencyProbe:
    COLS = ["hb", "t_rs_ns", "t_rd_ns", "js_recv_ns",
            "write_done_ns", "cycle_ns"]
    _FLUSH_EVERY_S = 1.0

    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(self.COLS)
        self._f.flush()
        self._last_hb = None
        self._last_flush = time.monotonic()
        self.rows = 0
        self.attempted = 0
        self.rejected = 0
        self.order_violations = 0

    def record(self, tag, js_recv_ns, write_done_ns, hb_counter_val=0):
        self.attempted += 1
        parts = tag.split(";")
        if len(parts) < 4 or parts[0] != "p":
            self.rejected += 1
            return False
        try:
            hb = int(parts[1])
            t_rs = int(parts[2])
            t_rd = int(parts[3])
        except ValueError:
            self.rejected += 1
            return False
        if hb == self._last_hb:
            return False
        self._last_hb = hb
        cycle_ns = write_done_ns - t_rs
        if not (t_rs <= t_rd <= js_recv_ns <= write_done_ns):
            self.order_violations += 1
        self._w.writerow([hb, t_rs, t_rd, js_recv_ns, write_done_ns, cycle_ns])
        self.rows += 1
        now = time.monotonic()
        if now - self._last_flush >= self._FLUSH_EVERY_S:
            self._f.flush()
            self._last_flush = now
        return True

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass
