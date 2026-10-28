#!/usr/bin/env python3
"""Analyse latency_probe.csv written by plc_ros_bridge (latency_probe: true).

Two modes:
  * stats only (default)      — identical CLI/stdout to the original tool
  * stats + plots (--plot)    — time-series curves, histograms, ECDFs

All stage times are differences of consecutive timestamps (all monotonic_ns
on the same host), so every stage must be >= 0.

Usage:
    python3 analyze_probe.py latency_probe.csv [options]

Options:
    --skip N              Skip first N rows (warm-up)
    --poll-hz HZ          Expected PLC poll rate; triggers regression warnings
    --plot                Generate plots, saved to <csv>.png
    --plot-file PATH      Override plot output path
    --show                Display plot interactively (implies --plot)
    --window N            Rolling window in rows for smoothing (default: 50)
    --bins N              Histogram bins (default: 60)
    --export-clean PATH   Write cleaned CSV: row, time_s, <stage_ms>...
    --quiet               Suppress warnings

CSV columns:
    origin_ns, read_start_ns, read_done_ns, cmd_pub_ns,
    sp_pub_ns, js_pub_ns, js_recv_ns, write_done_ns, hb_counter_val
----------------------------------+++++++++++++++++
how to use it:
# 1. إحصاءات فقط (كما قبل، متوافق 100%)
python3 analyze_probe.py /tmp/plc_latency.csv

# 2. إحصاءات + رسوم
python3 analyze_probe.py /tmp/plc_latency.csv \
    --plot \
    --poll-hz 120 \
    --window 80 \
    --export-clean /tmp/plc_latency_clean.csv
# → يحفظ /tmp/plc_latency.png
# → يطبع تحذيرًا إن كان write_net p50 > 15ms

# 3. عرض تفاعلي
python3 analyze_probe.py /tmp/plc_latency.csv --show --window 100

# 4. سكربت على دفعات (headless CI)
MPLBACKEND=Agg python3 analyze_probe.py data.csv --plot --quiet
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np


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
    ("cmd_to_feedback", "write_done_ns", "read_done_ns"),
    ("total_loop",      "write_done_ns", "origin_ns"),
]
BOUNDS_MS = [10, 15, 20, 30, 40, 50, 80, 100, 150, 200, 300]

# Plot styling
STAGE_STYLE = {
    "queue_delay":     ("#1f77b4", 1.0),
    "read_net":        ("#ff7f0e", 1.2),
    "convert_pub":     ("#2ca02c", 1.0),
    "ros_ruckig":      ("#d62728", 1.0),
    "isaac_step":      ("#9467bd", 1.2),
    "ros_back":        ("#8c564b", 1.0),
    "write_net":       ("#e377c2", 1.2),
    "cmd_to_feedback": ("#17becf", 1.5),
    "total_loop":      ("#000000", 1.8),
}


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------
def pct(sorted_vals, p):
    if len(sorted_vals) == 0:
        return float("nan")
    k = (len(sorted_vals) - 1) * p / 100.0
    lo, hi = int(k), min(int(k) + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def rolling(x, window, op=np.median):
    """Edge-padded rolling op. O(N·W); fine for probe-sized CSVs."""
    if window <= 1 or len(x) < 2:
        return np.asarray(x, dtype=float)
    half = window // 2
    pad_l = half
    pad_r = window - half - 1
    if pad_r < 0:
        pad_r = 0
    padded = np.pad(np.asarray(x, dtype=float),
                    (pad_l, pad_r), mode="edge")
    out = np.empty(len(x), dtype=float)
    for i in range(len(x)):
        out[i] = op(padded[i:i + window])
    return out


# ---------------------------------------------------------------------------
# Loading & extraction
# ---------------------------------------------------------------------------
def load_rows(path, skip):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for i, r in enumerate(reader):
            if i < skip:
                continue
            try:
                rows.append({k: int(v) for k, v in r.items()})
            except (ValueError, TypeError) as e:
                print(f"WARNING: skipping malformed row {i}: {e}",
                      file=sys.stderr)
    return rows


def extract_series(rows):
    out = {}
    for name, late, early in STAGES + TOTALS:
        out[name] = np.array(
            [(r[late] - r[early]) / 1e6 for r in rows], dtype=float)
    t0 = rows[0]["origin_ns"]
    out["_time_s"] = np.array(
        [(r["origin_ns"] - t0) / 1e9 for r in rows], dtype=float)
    return out


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def print_stats(rows, series, quiet=False):
    span_s = (rows[-1]["write_done_ns"] - rows[0]["origin_ns"]) / 1e9
    print(f"rows: {len(rows)}   span: {span_s:.1f} s   "
          f"coverage: {len(rows) / span_s:.1f} rows/s"
          if span_s > 0 else f"rows: {len(rows)}")
    print()
    print(f"{'metric':<16}{'min':>8}{'p50':>8}{'p95':>8}{'p99':>8}"
          f"{'p99.9':>9}{'max':>9}{'mean':>9}{'std':>8}  (ms)")

    for name, _, _ in STAGES + TOTALS:
        v = series[name]
        s = np.sort(v)
        print(f"{name:<16}{s[0]:>8.2f}{pct(s, 50):>8.2f}"
              f"{pct(s, 95):>8.2f}{pct(s, 99):>8.2f}"
              f"{pct(s, 99.9):>9.2f}{s[-1]:>9.2f}"
              f"{v.mean():>9.2f}{v.std():>8.2f}")
        if s[0] < 0 and not quiet:
            print(f"   WARNING: {name} has negative values -> timestamps "
                  f"not on one clock or the tag chain is wrong")

    print("\nLatency bound probability, cmd_to_feedback (share of rows <= bound):")
    v = np.sort(series["cmd_to_feedback"])
    for b in BOUNDS_MS:
        prob = 100.0 * np.searchsorted(v, b, side="right") / len(v)
        print(f"  <= {b:>4} ms : {prob:6.2f}%")

    print("\nWorst 5 rows by total_loop (to see WHICH stage caused a spike):")
    order = np.argsort(series["total_loop"])[::-1][:5]
    hdr = ["row", "total_loop"] + [n for n, _, _ in STAGES]
    print("  " + "".join(f"{h:>13}" for h in hdr))
    for i in order:
        cells = [int(i)] + [series["total_loop"][i]] + \
                [series[n][i] for n, _, _ in STAGES]
        line = f"  {cells[0]:>13d}"
        for c in cells[1:]:
            line += f"{c:>13.2f}"
        print(line)


def check_sanity(series, poll_hz_hint=None, quiet=False):
    """Emit warnings that would surface a pipeline regression."""
    if quiet:
        return

    if poll_hz_hint:
        period_ms = 1000.0 / poll_hz_hint
        p50 = float(np.median(series["write_net"]))
        if p50 > 1.8 * period_ms:
            print(f"\nWARNING: write_net p50 = {p50:.1f} ms > 1.8 x "
                  f"poll_period ({1.8 * period_ms:.1f} ms).")
            print("    Likely: pipelined act_ang write (old bridge), "
                  "network RTT > period, or clock skew.")

    for name, _, _ in STAGES + TOTALS:
        if np.any(series[name] < -0.001):
            print(f"\nWARNING: {name} contains negative samples. "
                  f"Check that all timestamps are on the same clock.")


def export_clean(rows, series, path):
    """Write cleaned CSV: row, time_s, every stage in ms."""
    cols = ["row", "time_s"] + [n for n, _, _ in STAGES + TOTALS]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i in range(len(rows)):
            w.writerow([i, f"{series['_time_s'][i]:.6f}"] +
                       [f"{series[n][i]:.3f}" for n, _, _ in STAGES + TOTALS])
    print(f"\nCleaned CSV written to {path}")


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
def build_plots(rows, series, window, bins, plot_file, show):
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.gridspec import GridSpec
    except ImportError:
        print("\nmatplotlib not installed; skipping plots.\n"
              "    Install with:  pip install matplotlib", file=sys.stderr)
        return

    t = series["_time_s"]
    span = t[-1] - t[0] if len(t) > 1 else 0.0

    fig = plt.figure(figsize=(16, 11))
    gs = GridSpec(3, 2, figure=fig,
                  hspace=0.38, wspace=0.22,
                  height_ratios=[1.2, 1.2, 1.0])

    # ---------- (1) All stages over time ----------
    ax = fig.add_subplot(gs[0, :])
    for name, _, _ in STAGES:
        color, lw = STAGE_STYLE[name]
        ax.plot(t, series[name], color=color, linewidth=lw * 0.5,
                alpha=0.30)
        smooth = rolling(series[name], window, np.median)
        ax.plot(t, smooth, color=color, linewidth=lw * 1.4, label=name)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("stage latency (ms)")
    ax.set_title(f"Per-stage latencies over time  "
                 f"(thin = raw, thick = rolling median, w = {window})")
    ax.grid(alpha=0.3, linestyle=":")
    ax.legend(loc="upper right", ncol=4, fontsize=8)

    # ---------- (2) Totals over time ----------
    ax = fig.add_subplot(gs[1, :])
    for name, _, _ in TOTALS:
        color, lw = STAGE_STYLE[name]
        ax.plot(t, series[name], color=color, linewidth=lw * 0.5,
                alpha=0.25)
        ax.plot(t, rolling(series[name], window, np.median),
                color=color, linewidth=lw, label=f"{name} p50")
        ax.plot(t, rolling(series[name], window,
                           lambda x: pct(np.sort(x), 95)),
                color=color, linewidth=lw * 0.7,
                linestyle="--", label=f"{name} p95")
        ax.plot(t, rolling(series[name], window,
                           lambda x: pct(np.sort(x), 99)),
                color=color, linewidth=lw * 0.7,
                linestyle=":", label=f"{name} p99")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("total latency (ms)")
    ax.set_title(f"Totals over time  (rolling p50/p95/p99, w = {window})")
    ax.grid(alpha=0.3, linestyle=":")
    ax.legend(loc="upper right", ncol=3, fontsize=7)

    # ---------- (3) Histogram of totals ----------
    ax = fig.add_subplot(gs[2, 0])
    for name, _, _ in TOTALS:
        color, _ = STAGE_STYLE[name]
        ax.hist(series[name], bins=bins, color=color, alpha=0.55,
                label=name, edgecolor="black", linewidth=0.3)
    ax.set_xlabel("latency (ms)")
    ax.set_ylabel("count")
    ax.set_title("Distribution of totals")
    ax.grid(alpha=0.3, linestyle=":")
    ax.legend()

    # ---------- (4) ECDF of totals ----------
    ax = fig.add_subplot(gs[2, 1])
    for name, _, _ in TOTALS:
        color, _ = STAGE_STYLE[name]
        s = np.sort(series[name])
        y = np.arange(1, len(s) + 1) / len(s) * 100.0
        ax.plot(s, y, color=color, label=name, linewidth=1.5)
    ax.set_xlabel("latency (ms)")
    ax.set_ylabel("cumulative %")
    ax.set_title("ECDF of totals")
    ax.grid(alpha=0.3, linestyle=":")
    # Log-x only if all values > 0; otherwise linear
    if np.all(series["cmd_to_feedback"] > 0):
        ax.set_xscale("log")
    ax.legend()

    fig.suptitle(
        f"latency_probe analysis  -  {len(rows)} rows  over  {span:.1f} s",
        fontsize=13, y=0.995)

    if plot_file:
        try:
            fig.savefig(plot_file, dpi=120, bbox_inches="tight")
            print(f"\nPlot saved to {plot_file}")
        except Exception as e:
            print(f"\nCould not save plot: {e}", file=sys.stderr)
    if show:
        try:
            plt.show()
        except Exception as e:
            print(f"\nCould not display plot: {e}", file=sys.stderr)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("csv", help="path to latency_probe.csv")
    ap.add_argument("--skip", type=int, default=0,
                    help="skip first N rows (warm-up)")
    ap.add_argument("--poll-hz", type=float, default=None,
                    help="expected PLC poll rate; enables regression warnings")
    ap.add_argument("--plot", action="store_true", help="generate plots")
    ap.add_argument("--plot-file", default=None,
                    help="save plot to PATH (default: <csv>.png)")
    ap.add_argument("--show", action="store_true",
                    help="show plot interactively (implies --plot)")
    ap.add_argument("--window", type=int, default=50,
                    help="rolling window in rows (default: 50)")
    ap.add_argument("--bins", type=int, default=60,
                    help="histogram bins (default: 60)")
    ap.add_argument("--export-clean", default=None,
                    help="write cleaned CSV with time_s + per-stage ms columns")
    ap.add_argument("--quiet", action="store_true", help="suppress warnings")
    args = ap.parse_args()

    rows = load_rows(args.csv, args.skip)
    if not rows:
        print("ERROR: no rows (is latency_probe: true? were nodes running?)")
        sys.exit(1)

    series = extract_series(rows)
    print_stats(rows, series, quiet=args.quiet)
    check_sanity(series, poll_hz_hint=args.poll_hz, quiet=args.quiet)

    if args.export_clean:
        export_clean(rows, series, args.export_clean)

    if args.plot or args.show:
        plot_file = args.plot_file
        if plot_file is None and not args.show:
            plot_file = str(Path(args.csv).with_suffix(".png"))
        build_plots(rows, series, args.window, args.bins,
                    plot_file, args.show)


if __name__ == "__main__":
    main()