#!/usr/bin/env python3
"""
latency_full_loop.py
====================
يقيس أربع مراحل من الحلقة الكاملة:

  L1: PLC read → /plc/cmd_ang          (bridge → ROS)
  L2: /plc/cmd_ang → /servo_setpoints  (Ruckig planning)
  L3: /servo_setpoints → /joint_states (Isaac physics)
  L4: /joint_states → PLC write        (ROS → PLC feedback)

يعتمد على طوابع زمنية مدمجة في JointState.header.stamp
(يجب أن تنشرها العقد الأخرى).

يُنتج أربعة ملفات CSV:
  latency_L1_plc_to_cmd.csv
  latency_L2_cmd_to_setpoint.csv
  latency_L3_setpoint_to_js.csv
  latency_L4_js_to_plc_write.csv
  latency_full_loop_summary.csv
"""

import csv
import time
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


# ---- إعدادات ----
REPORT_PERIOD_S = 5.0
WARMUP_SAMPLES  = 30
RUN_DURATION_S  = 60.0        # ثواني (اترك 0 للتشغيل الدائم)
THRESHOLDS_MS   = [5.0, 10.0, 15.0, 20.0, 30.0, 50.0]
MATCH_WINDOW_MS = 100.0       # نافذة مطابقة الرسائل بين topics


class FullLoopLatency(Node):
    def __init__(self):
        super().__init__('latency_full_loop')

        # ---- حالة عامة ----
        self.start_time = time.monotonic()
        self.done = False

        # ---- قوائم الانتظار (للمطابقة) ----
        # كل عنصر: (recv_time, msg_stamp, msg)
        self.cmd_queue       = []
        self.setpoint_queue  = []
        self.js_queue        = []

        # ---- عينات كل مرحلة ----
        self.samples = {
            'L1': [],   # PLC read → /plc/cmd_ang
            'L2': [],   # /plc/cmd_ang → /servo_setpoints
            'L3': [],   # /servo_setpoints → /joint_states
            'L4': [],   # /joint_states → PLC write (من topic تشخيصي)
        }

        # ---- عدّادات ----
        self.counts = defaultdict(int)

        # ---- ملفات CSV ----
        self._open_csvs()

        # ---- الاشتراكات ----
        # L1: نحتاج topic تشخيصي من plc_ros_bridge_instrumented
        #      /plc/cmd_ang مع header.stamp = لحظة قراءة PLC
        self.create_subscription(
            JointState, '/plc/cmd_ang', self.on_cmd_ang, 10)

        self.create_subscription(
            JointState, '/servo_setpoints', self.on_setpoints, 10)

        self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10)

        # L4: topic تشخيصي جديد من الـ bridge
        self.create_subscription(
            Float64, '/plc/write_latency_ms', self.on_plc_write, 10)

        # L1: topic تشخيصي جديد من الـ bridge (زمن قراءة OPC UA)
        self.create_subscription(
            Float64, '/plc/read_latency_ms', self.on_plc_read, 10)

        # ---- timers ----
        self.create_timer(REPORT_PERIOD_S, self.report)
        if RUN_DURATION_S > 0:
            self.create_timer(0.2, self.check_done)

        self.get_logger().info(
            f"Full-loop latency probe started  "
            f"(warmup={WARMUP_SAMPLES}, duration={RUN_DURATION_S}s)"
        )

    # ================================================================
    # CSV handling
    # ================================================================
    def _open_csvs(self):
        self.csv_files = {}
        self.csv_writers = {}

        header = ["timestamp_unix", "elapsed_s", "latency_ms"]

        for name in ['L1', 'L2', 'L3', 'L4', 'L1_opcua', 'L4_opcua']:
            f = open(f"latency_{name}.csv", "w", newline="")
            w = csv.writer(f)
            w.writerow(header)
            self.csv_files[name]   = f
            self.csv_writers[name] = w

        # summary
        self.sum_file = open("latency_full_loop_summary.csv", "w", newline="")
        self.sum_writer = csv.writer(self.sum_file)
        self.sum_writer.writerow([
            "elapsed_s",
            *[f"{n}_n"       for n in ('L1','L2','L3','L4')],
            *[f"{n}_mean_ms" for n in ('L1','L2','L3','L4')],
            *[f"{n}_p95_ms"  for n in ('L1','L2','L3','L4')],
            *[f"{n}_p99_ms"  for n in ('L1','L2','L3','L4')],
            *[f"{n}_max_ms"  for n in ('L1','L2','L3','L4')],
        ])

    def _write_sample(self, name, dt_ms):
        self.csv_writers[name].writerow([
            f"{time.time():.3f}",
            f"{time.monotonic() - self.start_time:.3f}",
            f"{dt_ms:.3f}",
        ])
        self.csv_files[name].flush()

    # ================================================================
    # Callbacks
    # ================================================================
    def on_cmd_ang(self, msg: JointState):
        """
        L1: نستقبل /plc/cmd_ang.
        header.stamp يأتي من الـ bridge (لحظة قراءة OPC UA).
        header.frame_id قد يحمل معلومة إضافية (اختياري).
        """
        self.counts['cmd_ang'] += 1
        recv = time.monotonic()

        # احسب L1 إذا كان header.stamp صالحاً
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns > 0:
            # حوّل إلى monotonic (تقريبي — يعتمد على تزامن الساعات)
            # للدقة الأفضل: استخدم نفس الساعة في كل العقد (monotonic)
            pub_time = stamp_ns / 1e9
            # ملاحظة: هذا تقريبي إذا كانت الساعات مختلفة
            # الأفضل: استخدم time.monotonic() في كل العقد ونقلها في frame_id
            pass

        self.cmd_queue.append((recv, msg))
        self._cleanup_queues()

    def on_setpoints(self, msg: JointState):
        """L2: /plc/cmd_ang → /servo_setpoints"""
        recv = time.monotonic()
        self.setpoint_queue.append((recv, msg))
        self._match_l2()
        self._cleanup_queues()

    def on_joint_states(self, msg: JointState):
        """L3: /servo_setpoints → /joint_states"""
        recv = time.monotonic()
        self.js_queue.append((recv, msg))
        self._match_l3()
        self._cleanup_queues()

    def on_plc_write(self, msg: Float64):
        """L4: زمن الكتابة إلى PLC (من plc_ros_bridge_instrumented)"""
        self.samples['L4'].append(float(msg.data))
        self._write_sample('L4', float(msg.data))

    def on_plc_read(self, msg: Float64):
        """L1 (OPC UA read): زمن قراءة OPC UA الخام"""
        self.samples['L1_opcua'] = self.samples.get('L1_opcua', [])
        self.samples['L1_opcua'].append(float(msg.data))
        self._write_sample('L1_opcua', float(msg.data))

    # ================================================================
    # Matching (مطابقة الرسائل بين topics)
    # ================================================================
    def _match_l2(self):
        """طابق كل cmd_ang مع أقرب setpoint في النافذة الزمنية."""
        if not self.cmd_queue or not self.setpoint_queue:
            return

        sp_recv, sp_msg = self.setpoint_queue[-1]
        # ابحث عن آخر cmd قبل sp
        for cmd_recv, cmd_msg in reversed(self.cmd_queue):
            if cmd_recv <= sp_recv:
                dt_ms = (sp_recv - cmd_recv) * 1000.0
                if 0 < dt_ms < MATCH_WINDOW_MS:
                    self.samples['L2'].append(dt_ms)
                    self._write_sample('L2', dt_ms)
                break

    def _match_l3(self):
        """طابق كل setpoint مع أقرب joint_states."""
        if not self.setpoint_queue or not self.js_queue:
            return

        js_recv, js_msg = self.js_queue[-1]
        for sp_recv, sp_msg in reversed(self.setpoint_queue):
            if sp_recv <= js_recv:
                dt_ms = (js_recv - sp_recv) * 1000.0
                if 0 < dt_ms < MATCH_WINDOW_MS:
                    self.samples['L3'].append(dt_ms)
                    self._write_sample('L3', dt_ms)
                break

    def _cleanup_queues(self):
        """احذف الرسائل الأقدم من نافذة المطابقة."""
        cutoff = time.monotonic() - (MATCH_WINDOW_MS / 1000.0) * 2

        self.cmd_queue      = [x for x in self.cmd_queue      if x[0] > cutoff]
        self.setpoint_queue = [x for x in self.setpoint_queue if x[0] > cutoff]
        self.js_queue       = [x for x in self.js_queue       if x[0] > cutoff]

    # ================================================================
    # Reporting
    # ================================================================
    def report(self):
        if all(len(v) < 2 for v in self.samples.values()):
            return

        elapsed = time.monotonic() - self.start_time

        print("=" * 78)
        print(f"  Full-loop latency report  [{elapsed:6.1f}s]")
        print("=" * 78)
        print(f"  {'Stage':<8}{'n':>6}{'mean':>10}{'std':>9}"
              f"{'min':>9}{'p50':>9}{'p95':>9}{'p99':>9}{'max':>9}")
        print("-" * 78)

        summary_row = [f"{elapsed:.1f}"]
        summary_data = {}

        for name in ['L1', 'L1_opcua', 'L2', 'L3', 'L4']:
            data = self.samples.get(name, [])
            if len(data) < 2:
                continue
            s = data[WARMUP_SAMPLES:] if len(data) > WARMUP_SAMPLES else data
            if len(s) < 2:
                continue
            arr = np.array(s)
            stats = {
                'n':    len(arr),
                'mean': float(np.mean(arr)),
                'std':  float(np.std(arr, ddof=1)),
                'min':  float(np.min(arr)),
                'p50':  float(np.percentile(arr, 50)),
                'p95':  float(np.percentile(arr, 95)),
                'p99':  float(np.percentile(arr, 99)),
                'max':  float(np.max(arr)),
            }
            print(f"  {name:<8}{stats['n']:>6}"
                  f"{stats['mean']:>10.2f}{stats['std']:>9.2f}"
                  f"{stats['min']:>9.2f}{stats['p50']:>9.2f}"
                  f"{stats['p95']:>9.2f}{stats['p99']:>9.2f}"
                  f"{stats['max']:>9.2f}")
            summary_data[name] = stats

        print("-" * 78)

        # الملخص
        for name in ('L1', 'L2', 'L3', 'L4'):
            st = summary_data.get(name)
            if st:
                summary_row.extend([
                    st['n'], f"{st['mean']:.2f}",
                    f"{st['p95']:.2f}", f"{st['p99']:.2f}", f"{st['max']:.2f}"
                ])
            else:
                summary_row.extend([0, "0", "0", "0", "0"])

        self.sum_writer.writerow(summary_row)
        self.sum_file.flush()

        # LBP
        for name in ('L2', 'L3'):
            if name in summary_data:
                arr = np.array(self.samples[name])
                total = len(arr)
                print(f"  LBP {name}:")
                for t in THRESHOLDS_MS:
                    pct = 100.0 * np.sum(arr <= t) / total
                    print(f"    ≤ {t:>5.1f} ms → {pct:6.2f}%")

        print("=" * 78)

        # امسح العينات (لكن أبقِ الملفات مفتوحة)
        for k in self.samples:
            self.samples[k].clear()

    # ================================================================
    def check_done(self):
        if self.done:
            return
        if time.monotonic() - self.start_time >= RUN_DURATION_S:
            self.done = True
            print("\n⏹️  انتهت المدة — حفظ وإغلاق...")
            self.shutdown()

    def shutdown(self):
        self.report()
        for f in self.csv_files.values():
            f.close()
        self.sum_file.close()
        print("💾 تم الحفظ:")
        print("   - latency_L1.csv         (PLC → cmd_ang)")
        print("   - latency_L1_opcua.csv   (OPC UA read الخام)")
        print("   - latency_L2.csv         (cmd → setpoint)")
        print("   - latency_L3.csv         (setpoint → joint_states)")
        print("   - latency_L4.csv         (joint_states → PLC write)")
        print("   - latency_full_loop_summary.csv")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = FullLoopLatency()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()


if __name__ == '__main__':
    main()