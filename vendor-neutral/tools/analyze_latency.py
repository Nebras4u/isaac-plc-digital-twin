#!/usr/bin/env python3
"""Analyze latency_probe.csv written by plc_ros_bridge.py (latency_probe: true).

Every row is one data item followed through the whole loop on a single
monotonic clock:

  read_start -> read_done -> cmd_pub -> sp_pub -> js_pub -> js_recv -> write_done

Stages (ms):
  L1  PLC read            read_done  - read_start   (driver round trip)
  L1b bridge publish prep cmd_pub    - read_done
  L2  cmd -> setpoint     sp_pub     - cmd_pub      (DDS + wait for servo tick + Ruckig)
  L3  setpoint -> js      js_pub     - sp_pub       (DDS + wait for next Isaac step)
  L3b js -> bridge        js_recv    - js_pub       (DDS)
  L4  js -> PLC write     write_done - js_recv      (wait for next bridge tick + write)
  TOTAL                   write_done - read_start

It measures the age of a data item along the software chain. It does NOT
include (a) the delay between the PLC value changing and the bridge reading
it (up to one poll period), or (b) the physical settling time of the robot.

Usage:
  python3 analyze_latency.py latency_probe.csv [warmup_rows=30]
"""
import csv
import sys

import numpy as np

THRESHOLDS_MS = [5.0, 10.0, 15.0, 20.0, 30.0, 50.0]


def load(path):
    with open(path, newline="") as f:
        r = csv.reader(f)
        next(r)
        rows = [[int(x) for x in row] for row in r if row]
    return np.array(rows, dtype=np.int64)


def stages(a):
    ms = 1e-6
    rs, rd, cp, sp, jp, jr, wd = (a[:, i] for i in range(1, 8))
    return {
        "L1  PLC read": (rd - rs) * ms,
        "L1b bridge pub": (cp - rd) * ms,
        "L2  cmd->setpoint": (sp - cp) * ms,
        "L3  setpoint->js": (jp - sp) * ms,
        "L3b js->bridge": (jr - jp) * ms,
        "L4  js->PLC write": (wd - jr) * ms,
        "TOTAL": (wd - rs) * ms,
    }


def stats(x):
    return (len(x), x.mean(), x.std(ddof=1) if len(x) > 1 else 0.0, x.min(),
            np.percentile(x, 50), np.percentile(x, 95),
            np.percentile(x, 99), np.percentile(x, 99.9), x.max())


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = sys.argv[1]
    warm = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    a = load(path)
    if len(a) <= warm + 1:
        print(f"Not enough rows ({len(a)}); run longer.")
        sys.exit(1)
    a = a[warm:]
    st = stages(a)

    bad = {k: int((v < 0).sum()) for k, v in st.items() if (v < 0).any()}
    if bad:
        print("WARNING: negative stage times (clock mismatch or ordering "
              f"problem, e.g. Isaac on another host): {bad}")

    hdr = f"{'stage':<20}{'n':>7}{'mean':>8}{'std':>8}{'min':>8}" \
          f"{'p50':>8}{'p95':>8}{'p99':>8}{'p99.9':>8}{'max':>8}"
    print("=" * len(hdr))
    print(hdr)
    print("-" * len(hdr))
    out = []
    for name, v in st.items():
        n, mean, sd, mn, p50, p95, p99, p999, mx = stats(v)
        print(f"{name:<20}{n:>7}{mean:>8.2f}{sd:>8.2f}{mn:>8.2f}"
              f"{p50:>8.2f}{p95:>8.2f}{p99:>8.2f}{p999:>8.2f}{mx:>8.2f}")
        out.append([name, n, mean, sd, mn, p50, p95, p99, p999, mx])
    print("=" * len(hdr))

    tot = st["TOTAL"]
    print("TOTAL loop (data age at PLC write):")
    for t in THRESHOLDS_MS:
        print(f"  <= {t:>5.1f} ms : {100.0 * (tot <= t).mean():6.2f} %")
    if len(tot) < 1000:
        print(f"\nNote: n={len(tot)} is too small for a meaningful p99.9; "
              "run at least a few minutes.")

    with open("latency_probe_summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stage", "n", "mean_ms", "std_ms", "min_ms", "p50_ms",
                    "p95_ms", "p99_ms", "p999_ms", "max_ms"])
        for row in out:
            w.writerow([row[0], row[1]] + [f"{x:.3f}" for x in row[2:]])
    print("\nsaved latency_probe_summary.csv")


if __name__ == "__main__":
    main()
