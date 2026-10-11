#!/usr/bin/env python3
import time, csv
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


THRESHOLDS_MS     = [5.0, 10.0, 15.0, 20.0, 50.0]
REPORT_PERIOD_S   = 5.0
WARMUP_SAMPLES    = 20
RUN_DURATION_S    = 15.0     # ⏱️ مدة التشغيل


class LatencyAcademic(Node):
    def __init__(self):
        super().__init__('latency_academic')
        self.t_cmd      = None
        self.samples    = []
        self.timestamps = []
        self.start_time = time.monotonic()

        # ---- CSV: كل عينة ----
        self.csv_file   = open("latency_samples.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp_unix", "elapsed_s", "latency_ms"])

        # ---- CSV: ملخص كل تقرير ----
        self.sum_file   = open("latency_summary.csv", "w", newline="")
        self.sum_writer = csv.writer(self.sum_file)
        self.sum_writer.writerow([
            "elapsed_s", "n", "mean", "std", "min", "p50",
            "p95", "p99", "p999", "max", "jitter", *[f"LBP_{t}" for t in THRESHOLDS_MS]
        ])

        self.create_subscription(JointState, '/plc/cmd_ang',      self.on_cmd,   10)
        self.create_subscription(JointState, '/servo_setpoints',  self.on_servo, 10)
        self.create_timer(REPORT_PERIOD_S, self.report)
        self.create_timer(0.1,             self.check_done)

    # ---------------- callbacks ----------------
    def on_cmd(self, msg):
        self.t_cmd = time.monotonic()
        self.timestamps.append(self.t_cmd)

    def on_servo(self, msg):
        if self.t_cmd is not None:
            dt_ms = (time.monotonic() - self.t_cmd) * 1000.0
            if 0 < dt_ms < 500:
                self.samples.append(dt_ms)
                elapsed = time.monotonic() - self.start_time
                self.csv_writer.writerow([f"{time.time():.3f}",
                                          f"{elapsed:.3f}",
                                          f"{dt_ms:.3f}"])
                self.csv_file.flush()
            self.t_cmd = None

    # ---------------- report ----------------
    def report(self):
        if len(self.samples) < 2:
            return

        s = self.samples[WARMUP_SAMPLES:] if len(self.samples) > WARMUP_SAMPLES else self.samples
        if len(s) < 2:
            return

        arr   = np.array(s)
        mean  = float(np.mean(arr))
        std   = float(np.std(arr, ddof=1))
        mn    = float(np.min(arr))
        mx    = float(np.max(arr))
        p50   = float(np.percentile(arr, 50))
        p95   = float(np.percentile(arr, 95))
        p99   = float(np.percentile(arr, 99))
        p999  = float(np.percentile(arr, 99.9))

        if len(self.timestamps) > 2:
            jitter = float(np.std(np.diff(self.timestamps) * 1000.0, ddof=1))
        else:
            jitter = 0.0

        total = len(arr)
        lbp   = {t: 100.0 * float(np.sum(arr <= t)) / total for t in THRESHOLDS_MS}

        elapsed = time.monotonic() - self.start_time

        # ---- عرض ----
        print("=" * 62)
        print(f"  Latency report  [{elapsed:5.1f}s]  (n = {total})")
        print("-" * 62)
        print(f"  Mean   : {mean:7.2f} ms     Std   : {std:6.2f} ms")
        print(f"  Min    : {mn:7.2f} ms     Max   : {mx:7.2f} ms")
        print(f"  p50    : {p50:7.2f} ms     p95   : {p95:7.2f} ms")
        print(f"  p99    : {p99:7.2f} ms     p99.9 : {p999:7.2f} ms")
        print(f"  Inter-arrival jitter : {jitter:.2f} ms")
        print("-" * 62)
        for t in THRESHOLDS_MS:
            print(f"    LBP(<= {t:>5.1f} ms) = {lbp[t]:6.2f}%")
        print("=" * 62)

        # ---- حفظ الملخص ----
        self.sum_writer.writerow([
            f"{elapsed:.1f}", total, f"{mean:.2f}", f"{std:.2f}",
            f"{mn:.2f}", f"{p50:.2f}", f"{p95:.2f}", f"{p99:.2f}",
            f"{p999:.2f}", f"{mx:.2f}", f"{jitter:.2f}",
            *[f"{lbp[t]:.2f}" for t in THRESHOLDS_MS]
        ])
        self.sum_file.flush()

        self.samples.clear()
        self.timestamps.clear()

    # ---------------- auto-stop ----------------
    def check_done(self):
        if time.monotonic() - self.start_time >= RUN_DURATION_S:
            print("\n⏹️  انتهت المدة — إيقاف...")
            self.shutdown()

    def shutdown(self):
        self.report()
        self.csv_file.close()
        self.sum_file.close()
        print("💾 latency_samples.csv + latency_summary.csv")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = LatencyAcademic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()


if __name__ == '__main__':
    main()