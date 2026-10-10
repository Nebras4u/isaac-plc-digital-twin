#!/usr/bin/env python3
import time
import statistics
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# ---- Configuration ----
THRESHOLDS_MS = [5.0, 10.0, 15.0, 20.0, 50.0]   # thresholds for LBP
REPORT_PERIOD_S = 5.0                             # print every N seconds
WARMUP_SAMPLES = 20                               # ignore first N samples


class LatencyAcademic(Node):
    def __init__(self):
        super().__init__('latency_academic')
        self.t_cmd = None
        self.samples = []          # latency samples (ms)
        self.timestamps = []       # arrival timestamps (monotonic) for jitter

        self.create_subscription(JointState, '/plc/cmd_ang', self.on_cmd, 10)
        self.create_subscription(JointState, '/servo_setpoints', self.on_servo, 10)
        self.create_timer(REPORT_PERIOD_S, self.report)

    # -------- callbacks --------
    def on_cmd(self, msg):
        self.t_cmd = time.monotonic()
        self.timestamps.append(self.t_cmd)

    def on_servo(self, msg):
        if self.t_cmd is not None:
            dt_ms = (time.monotonic() - self.t_cmd) * 1000.0
            if 0 < dt_ms < 500:                    # ignore outliers/hiccups
                self.samples.append(dt_ms)
            self.t_cmd = None

    # -------- report --------
    def report(self):
        if len(self.samples) < 2:
            return

        # drop warm-up
        s = self.samples[WARMUP_SAMPLES:] if len(self.samples) > WARMUP_SAMPLES else self.samples
        if len(s) < 2:
            return

        arr = np.array(s)

        # ---- Basic stats ----
        mean  = float(np.mean(arr))
        std   = float(np.std(arr, ddof=1))
        mn    = float(np.min(arr))
        mx    = float(np.max(arr))
        p50   = float(np.percentile(arr, 50))
        p95   = float(np.percentile(arr, 95))
        p99   = float(np.percentile(arr, 99))
        p999  = float(np.percentile(arr, 99.9))     # worst case (99.9th)

        # ---- Jitter (inter-arrival time std dev) ----
        if len(self.timestamps) > 2:
            ts = np.array(self.timestamps)
            inter_arrival = np.diff(ts) * 1000.0    # ms between cmds
            jitter = float(np.std(inter_arrival, ddof=1))
        else:
            jitter = 0.0

        # ---- LBP for each threshold ----
        total = len(arr)
        lbp_lines = []
        for t in THRESHOLDS_MS:
            n_below = int(np.sum(arr <= t))
            pct = 100.0 * n_below / total
            lbp_lines.append(f"    LBP(<= {t:>5.1f} ms) = {pct:6.2f}%")

        # ---- Print report ----
        print("=" * 62)
        print(f"  Latency report  (n = {total} samples)")
        print("-" * 62)
        print(f"  Mean      : {mean:7.2f} ms")
        print(f"  Std dev   : {std:7.2f} ms   (jitter of latency)")
        print(f"  Min       : {mn:7.2f} ms")
        print(f"  Median    : {p50:7.2f} ms")
        print(f"  p95       : {p95:7.2f} ms")
        print(f"  p99       : {p99:7.2f} ms")
        print(f"  p99.9     : {p999:7.2f} ms   ← worst case")
        print(f"  Max       : {mx:7.2f} ms")
        print(f"  Inter-arrival jitter : {jitter:6.2f} ms")
        print("-" * 62)
        print("  Latency Bound Probability (LBP):")
        for line in lbp_lines:
            print(line)
        print("=" * 62)

        self.samples.clear()
        self.timestamps.clear()


def main():
    rclpy.init()
    node = LatencyAcademic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()