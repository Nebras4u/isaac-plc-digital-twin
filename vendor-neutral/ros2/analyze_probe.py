#!/usr/bin/env python3
"""Analyse latency_probe.csv written by plc_ros_bridge (latency_probe: true).

Usage:  python3 analyze_probe.py latency_probe.csv [--skip N]

* Raw statistics only: NOTHING is trimmed, no 'top 1%' removal.
* Stage times are differences of consecutive timestamps (all monotonic_ns on
  the same host), so every stage must be >= 0.
"""
import csv
import sys

STAGES = [   # name, later column, earlier column
    ("queue_delay",  "read_start_ns", "origin_ns"),
    ("read_net",     "read_done_ns",  "read_start_ns"),
    ("convert_pub",  "cmd_pub_ns",    "read_done_ns"),
    ("ros_ruckig",   "sp_pub_ns",     "cmd_pub_ns"),
    ("isaac_step",   "js_pub_ns",     "sp_pub_ns"),
    ("ros_back",     "js_recv_ns",    "js_pub_ns"),
    ("write_net",    "write_done_ns", "js_recv_ns"),
]
TOTALS = [
    ("cmd_to_feedback", "write_done_ns", "read_done_ns"),   # command known -> PLC has feedback
    ("total_loop",      "write_done_ns", "origin_ns"),      # + polling wait before the read
]
BOUNDS_MS = [10, 15, 20, 30, 40, 50, 80, 100, 150, 200, 300]


def pct(sorted_vals, p):
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * p / 100.0
    lo, hi = int(k), min(int(k) + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = sys.argv[1]
    skip = int(sys.argv[sys.argv.index("--skip") + 1]) if "--skip" in sys.argv else 0
    rows = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(open(path))]
    rows = rows[skip:]
    if not rows:
        print("ERROR: no rows (is latency_probe: true? were nodes running?)")
        sys.exit(1)

    span_s = (rows[-1]["write_done_ns"] - rows[0]["origin_ns"]) / 1e9
    print(f"rows: {len(rows)}   span: {span_s:.1f} s   "
          f"coverage: {len(rows) / span_s:.1f} rows/s" if span_s > 0 else f"rows: {len(rows)}")
    print()
    print(f"{'metric':<16}{'min':>8}{'p50':>8}{'p95':>8}{'p99':>8}{'p99.9':>9}{'max':>9}  (ms)")
    series = {}
    for name, late, early in STAGES + TOTALS:
        vals = sorted((r[late] - r[early]) / 1e6 for r in rows)
        series[name] = vals
        print(f"{name:<16}{vals[0]:>8.2f}{pct(vals,50):>8.2f}{pct(vals,95):>8.2f}"
              f"{pct(vals,99):>8.2f}{pct(vals,99.9):>9.2f}{vals[-1]:>9.2f}")
        if vals[0] < 0:
            print(f"   WARNING: {name} has negative values -> timestamps not on one clock "
                  f"or the tag chain is wrong")

    print("\nLatency bound probability, cmd_to_feedback (share of rows <= bound):")
    v = series["cmd_to_feedback"]
    for b in BOUNDS_MS:
        print(f"  <= {b:>4} ms : {100.0 * sum(1 for x in v if x <= b) / len(v):6.2f}%")

    print("\nWorst 5 rows by total_loop (to see WHICH stage caused a spike):")
    order = sorted(range(len(rows)),
                   key=lambda i: rows[i]["write_done_ns"] - rows[i]["origin_ns"],
                   reverse=True)[:5]
    hdr = ["total_loop"] + [n for n, _, _ in STAGES]
    print("  " + "".join(f"{h:>13}" for h in hdr))
    for i in order:
        r = rows[i]
        cells = [(r["write_done_ns"] - r["origin_ns"]) / 1e6] + \
                [(r[l] - r[e]) / 1e6 for _, l, e in STAGES]
        print("  " + "".join(f"{c:>13.2f}" for c in cells))


if __name__ == "__main__":
    main()
