#!/usr/bin/env python3
"""Clock offset / drift calibration client (reference tool, pairs with srv.py).

Modes
  Session (default): one measurement run, full statistics, raw data saved.
      python3 cli.py <peer_ip> [--duration 600] [--clock auto|perf|mono|raw|wall]

  Track (--track): repeated short bursts for a whole experiment. Each period the
  lowest-RTT sample of the burst is appended to <out>_track.csv. Use it to
  correct timestamps continuously, because the offset is NOT constant.
      python3 cli.py <peer_ip> --track [--period 1] [--burst 50] [--duration 0]
  (--duration 0 = until Ctrl+C.)

  Analyze (--analyze FILE): re-run the analysis offline on a saved *_raw.csv
  or *_track.csv (same code path as a live run, so results are reproducible).
      python3 cli.py --analyze calib_xxx_raw.csv

Method
  Each ping gives four timestamps: t1 (client send), t2 (peer receive),
  t3 (peer send), t4 (client receive).
      offset = ((t2 - t1) + (t3 - t4)) / 2     peer_clock - local_clock
      rtt    = (t4 - t1) - (t3 - t2)
  Ping and reply packets have the same size (48 B), pings are spaced with a
  small random jitter (avoids locking onto periodic interference), and the
  garbage collector is off during measurement.

  Estimators (all reported):
    * least squares line on the lowest-RTT fraction of all pings (--fit-frac)
    * Theil-Sen (median slope) on a series with the lowest-RTT ping of every
      --bin seconds. Robust to outliers; if the two disagree a note is raised.
    * drift per fixed window (--window) to show whether the drift is constant
    * overlapping Allan deviation of that series (standard clock-stability
      metric; insensitive to constant drift). At tau = bin it is dominated by
      measurement noise; read it at larger tau.
  Drift in ppm: 1 ppm = 1 microsecond per second.
  Correcting timestamps: local_equiv = peer_time - offset(local_time), with
  offset interpolated (numpy.interp) from the track file.

Limitations (state them in the paper)
  * Assumes equal forward and backward delay. Worst-case OFFSET error is
    min_rtt / 2 and cannot be reduced in software. A constant asymmetry does
    not affect the DRIFT (slope).
  * drift_se assumes independent residuals; samples are autocorrelated, so
    treat it as a lower bound. Use the window spread as the empirical check.
  * Software timestamps in Python include OS/VM scheduling jitter.
  * Mixing clock kinds on the two sides (e.g. raw vs mono, or wall vs
    monotonic) is allowed but flagged: the result then includes slewing.
  * --clock wall includes NTP/chrony steps and slews; disable time sync for a
    pure oscillator measurement.


# الجهاز الأول
sudo python3 srv.py --clock perf

# الجهاز الثاني
python3 cli.py <ip> --duration 600

"""
import argparse
import bisect
import csv
import datetime
import functools
import gc
import itertools
import json
import math
import os
import platform
import random
import socket
import statistics
import struct
import sys
import time

TOOL_VERSION = "2.0"
SCHEMA_VERSION = 2
MAGIC = b"CLK1"
T_PING, T_INFO = 1, 2
PKT = 48
HDR = struct.Struct("!4sBBHII")      # 16 bytes
REP = struct.Struct("!4sBBHIIqq")    # 32 bytes (+16 zero padding on the wire)
CLOCKS = ("auto", "perf", "mono", "raw", "wall")
WARN_DRIFT_RANGE_PPM = 20.0          # floor for the "drift varies" note


# ----------------------------------------------------------------- clocks / env
def make_clock(kind):
    """Return (now_ns_function, description dict). Exits if unavailable."""
    has_raw = hasattr(time, "CLOCK_MONOTONIC_RAW")
    if kind == "auto":
        kind = "raw" if has_raw else "perf"
    if kind == "raw":
        if not has_raw:
            sys.exit("error: --clock raw needs CLOCK_MONOTONIC_RAW (Linux only)")
        fn = functools.partial(time.clock_gettime_ns, time.CLOCK_MONOTONIC_RAW)
        name = "CLOCK_MONOTONIC_RAW"
        res = time.clock_getres(time.CLOCK_MONOTONIC_RAW) * 1e9
    elif kind == "perf":
        fn, name = time.perf_counter_ns, "perf_counter_ns"
        res = time.get_clock_info("perf_counter").resolution * 1e9
    elif kind == "mono":
        fn, name = time.monotonic_ns, "monotonic_ns"
        res = time.get_clock_info("monotonic").resolution * 1e9
    elif kind == "wall":
        fn, name = time.time_ns, "time_ns"
        res = time.get_clock_info("time").resolution * 1e9
    else:
        sys.exit(f"error: unknown clock {kind!r}")
    return fn, {"kind": kind, "name": name, "resolution_ns": res}


def measure_tick(now, n=20000):
    """Empirical granularity (smallest positive step) and cost of one call."""
    gc.collect()
    first = prev = now()
    step = None
    for _ in range(n):
        cur = now()
        d = cur - prev
        if d > 0 and (step is None or d < step):
            step = d
        prev = cur
    return {"min_step_ns": step, "call_cost_ns": (prev - first) / n}


def tune(cpu, rt):
    applied = {"gc": "disabled"}
    gc.collect()
    gc.disable()
    if cpu is not None:
        try:
            os.sched_setaffinity(0, {cpu})
            applied["cpu"] = cpu
        except (AttributeError, OSError) as e:
            applied["cpu"] = f"failed: {e}"
    if rt is not None:
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(rt))
            applied["sched"] = f"SCHED_FIFO/{rt}"
        except (AttributeError, OSError) as e:
            applied["sched"] = f"failed: {e}"
    return applied


def clocksource():
    try:
        with open("/sys/devices/system/clocksource/clocksource0/current_clocksource") as f:
            return f.read().strip()
    except OSError:
        return None


# ------------------------------------------------------------------- transport
class Prober:
    def __init__(self, peer, port, now, timeout):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.settimeout(timeout)
        self.timeout = timeout
        self.addr = (socket.gethostbyname(peer), port)   # resolve once
        self.now = now
        self.sid = random.getrandbits(32)
        self.sbuf = bytearray(PKT)
        self.rbuf = bytearray(2048)
        self._dirty = False

    def info(self, tries=100):
        """Ask the responder to describe itself; None if unreachable."""
        req = bytearray(PKT)
        HDR.pack_into(req, 0, MAGIC, T_INFO, 0, 0, self.sid, 0)
        self.sock.settimeout(0.2)
        try:
            for _ in range(tries):
                try:
                    self.sock.sendto(req, self.addr)
                    while True:
                        data, _ = self.sock.recvfrom(2048)
                        if len(data) > HDR.size and data[:4] == MAGIC and data[4] == T_INFO:
                            return json.loads(data[HDR.size:].decode())
                except (OSError, ValueError):
                    time.sleep(0.05)
        finally:
            self.sock.settimeout(self.timeout)
        return None

    def ping(self, seq):
        """Return (t1, t2, t3, t4) in ns; raises OSError (timeout) on loss."""
        if self._dirty:
            self.sock.settimeout(self.timeout)
            self._dirty = False
        HDR.pack_into(self.sbuf, 0, MAGIC, T_PING, 0, 0, self.sid, seq)
        now = self.now
        deadline = time.monotonic() + self.timeout     # total budget per ping
        t1 = now()
        self.sock.sendto(self.sbuf, self.addr)
        while True:
            n, _ = self.sock.recvfrom_into(self.rbuf)
            t4 = now()
            if n == PKT:
                magic, typ, _, _, sid, rseq, t2, t3 = REP.unpack_from(self.rbuf, 0)
                if magic == MAGIC and typ == T_PING and sid == self.sid and rseq == seq:
                    return t1, t2, t3, t4
            remaining = deadline - time.monotonic()    # stale/foreign packet
            if remaining <= 0:
                raise socket.timeout("ping timeout")
            self.sock.settimeout(remaining)
            self._dirty = True

    def drain(self):
        self.sock.settimeout(0.0)
        try:
            while True:
                self.sock.recvfrom_into(self.rbuf)
        except OSError:
            pass
        self.sock.settimeout(self.timeout)


# ------------------------------------------------------------------ statistics
def linfit(xs, ys):
    """Least squares y = a + b*x. Returns (a, b, residual_std, slope_se)."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return my, 0.0, 0.0, 0.0
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    a = my - b * mx
    ss = sum((y - (a + b * x)) ** 2 for x, y in zip(xs, ys))
    resid = math.sqrt(ss / n)
    se = math.sqrt(ss / (n - 2) / sxx) if n > 2 else 0.0
    return a, b, resid, se


def theil_sen(t, x, max_pairs=200000, seed=12345):
    """Median-of-pairwise-slopes line. Returns (a, b) or None. Deterministic."""
    n = len(t)
    if n < 3:
        return None
    slopes = []
    if n * (n - 1) // 2 <= max_pairs:
        for i in range(n - 1):
            ti, xi = t[i], x[i]
            for j in range(i + 1, n):
                dt = t[j] - ti
                if dt > 0:
                    slopes.append((x[j] - xi) / dt)
    else:
        rng = random.Random(seed)
        while len(slopes) < max_pairs:
            i, j = rng.randrange(n), rng.randrange(n)
            if i > j:
                i, j = j, i
            if i != j and t[j] > t[i]:
                slopes.append((x[j] - x[i]) / (t[j] - t[i]))
    b = statistics.median(slopes)
    a = statistics.median([xi - b * ti for ti, xi in zip(t, x)])
    return a, b


def allan_deviation(x_s, tau0):
    """Overlapping Allan deviation of a time-error series x (seconds)."""
    n, out, m = len(x_s), [], 1
    while n - 2 * m >= 3:
        cnt = n - 2 * m
        s = 0.0
        for i in range(cnt):
            d = x_s[i + 2 * m] - 2 * x_s[i + m] + x_s[i]
            s += d * d
        out.append({"tau_s": m * tau0,
                    "adev_ppm": 1e6 * math.sqrt(s / (2.0 * m * m * tau0 * tau0 * cnt)),
                    "n": cnt})
        m *= 2
    return out


def fill_gaps(g):
    """Linear interpolation of None entries (first and last must be set)."""
    idx = [i for i, v in enumerate(g) if v is not None]
    out = list(g)
    for p, q in zip(idx, idx[1:]):
        for i in range(p + 1, q):
            out[i] = g[p] + (i - p) / (q - p) * (g[q] - g[p])
    return out[idx[0]: idx[-1] + 1]


def bin_best(recs, t0, bin_s):
    """Lowest-RTT record in every bin. recs: (t1_ns, offset_ns, rtt_ns)."""
    best = {}
    for rec in recs:
        k = int((rec[0] - t0) * 1e-9 / bin_s)
        cur = best.get(k)
        if cur is None or rec[2] < cur[2]:
            best[k] = rec
    keys = sorted(best)
    return keys, [best[k] for k in keys]


def analyze_series(t_s, x_ms, keys, bin_s, phase, min_rtt_ms):
    """Robust drift, step check and Allan deviation of a best-of-bin series.

    t_s: sample times (s), x_ms: offsets (ms), keys: integer grid index of each
    sample (grid time = (key + phase) * bin_s). Returns (result dict, notes).
    """
    res, notes = {"n_points": len(t_s), "bin_s": bin_s}, []
    if len(t_s) < 8:
        return res, ["too few binned points for Theil-Sen / Allan deviation"]
    a_ts, b_ts = theil_sen(t_s, x_ms)
    _, b_ls, resid, se = linfit(t_s, x_ms)
    res.update({"theil_sen_drift_ppm": b_ts * 1000.0, "ls_drift_ppm": b_ls * 1000.0,
                "ls_se_ppm": se * 1000.0, "residual_std_ms": resid})

    r = [x - (a_ts + b_ts * t) for t, x in zip(t_s, x_ms)]
    d = [r[i + 1] - r[i] for i in range(len(r) - 1)]
    med = statistics.median(d)
    sig = max(1.4826 * statistics.median([abs(v - med) for v in d]), 1e-6)
    j = max(range(len(d)), key=lambda i: abs(d[i]))
    if abs(d[j]) > max(8 * sig, min_rtt_ms):
        res["possible_step"] = {"t_s": t_s[j + 1], "size_ms": d[j]}
        notes.append(f"possible clock step/slew change at t={t_s[j + 1]:.1f}s "
                     f"({d[j]:+.3f} ms): check time sync on both machines")

    ngrid = keys[-1] - keys[0] + 1
    res["coverage"] = len(keys) / ngrid
    if ngrid >= 8 and res["coverage"] >= 0.8:
        grid = [None] * ngrid
        for k, t, x in zip(keys, t_s, x_ms):       # move each sample to its grid time
            grid[k - keys[0]] = (x + b_ts * ((k + phase) * bin_s - t)) * 1e-3
        res["adev"] = allan_deviation(fill_gaps(grid), bin_s)
    else:
        notes.append("series too sparse for Allan deviation (coverage < 80%)")
    return res, notes


def analyze(rows, lost, fit_frac=0.1, window=30.0, bin_s=1.0):
    """rows: list of (seq, t1, t2, t3, t4) in time order. Returns a summary dict."""
    if len(rows) < 50:
        raise ValueError("too few replies for analysis")
    recs = [(t1, ((t2 - t1) + (t3 - t4)) / 2.0, (t4 - t1) - (t3 - t2))
            for _, t1, t2, t3, t4 in rows]           # (t1, offset_ns, rtt_ns)
    t0 = recs[0][0]
    tt = [(r[0] - t0) * 1e-9 for r in recs]
    duration = tt[-1]
    by_rtt = sorted(recs, key=lambda r: r[2])
    ref = by_rtt[0][1]                                # keeps numbers small
    k = min(len(recs), max(20, int(len(recs) * fit_frac)))
    fit = by_rtt[:k]
    a, b, resid, se = linfit([(r[0] - t0) * 1e-9 for r in fit],
                             [(r[1] - ref) * 1e-6 for r in fit])

    wins = []
    if window > 0:
        for j in range(int(duration // window)):
            lo = bisect.bisect_left(tt, j * window)
            hi = bisect.bisect_left(tt, (j + 1) * window)
            if hi - lo < 30:
                continue
            idx = sorted(range(lo, hi), key=lambda i: recs[i][2])[:max(10, (hi - lo) // 3)]
            _, wb, _, wse = linfit([tt[i] for i in idx],
                                   [(recs[i][1] - ref) * 1e-6 for i in idx])
            wins.append({"t_start_s": j * window, "drift_ppm": wb * 1000.0,
                         "se_ppm": wse * 1000.0, "n": len(idx)})

    rtts = sorted(r[2] for r in recs)
    min_rtt_ms = rtts[0] * 1e-6
    keys, best = bin_best(recs, t0, bin_s)
    series, snotes = analyze_series(
        [(r[0] - t0) * 1e-9 for r in best], [(r[1] - ref) * 1e-6 for r in best],
        keys, bin_s, 0.5, min_rtt_ms)

    out = {
        "schema_version": SCHEMA_VERSION,
        "received": len(recs), "lost": lost,
        "loss_pct": 100.0 * lost / (len(recs) + lost),
        "duration_s": duration,
        "offset_convention": "peer_clock - local_clock",
        "t0_local_ns": t0,
        "rtt_ms": {"min": min_rtt_ms, "median": statistics.median(rtts) * 1e-6,
                   "p95": rtts[int(0.95 * (len(rtts) - 1))] * 1e-6, "max": rtts[-1] * 1e-6},
        "fit": {"n": k, "frac": fit_frac, "drift_ppm": b * 1000.0,
                "drift_se_ppm": se * 1000.0,
                "offset_at_t0_ms": ref * 1e-6 + a,
                "offset_at_end_ms": ref * 1e-6 + a + b * duration,
                "residual_std_ms": resid},
        "series": series,
        "asymmetry_bound_ms": min_rtt_ms / 2.0,
        "windows": wins,
    }
    out["drift_resolution_ppm"] = 2000.0 * resid / duration if duration > 0 else None
    rng = (max(w["drift_ppm"] for w in wins) - min(w["drift_ppm"] for w in wins)) if wins else None
    out["drift_range_ppm"] = rng

    notes = list(snotes)
    if out["loss_pct"] > 1.0:
        notes.append("packet loss above 1%")
    if rng is not None:
        thr = max(WARN_DRIFT_RANGE_PPM, 4 * statistics.median(w["se_ppm"] for w in wins))
        if rng > thr:
            notes.append(f"drift varies across windows ({rng:.1f} ppm range): use --track "
                         "and correct continuously")
    else:
        notes.append("run too short for the windowed drift check")
    if resid > min_rtt_ms:
        notes.append("residual std exceeds min RTT: a straight line describes the offset poorly")
    if out["drift_resolution_ppm"] and out["drift_resolution_ppm"] > 2.0:
        notes.append(f"slope resolution is only ~{out['drift_resolution_ppm']:.1f} ppm: "
                     "run longer (10+ min) or use a quieter link")
    if "theil_sen_drift_ppm" in series:
        gap = abs(series["theil_sen_drift_ppm"] - out["fit"]["drift_ppm"])
        if gap > max(1.0, 3 * out["fit"]["drift_se_ppm"]):
            notes.append(f"estimators disagree by {gap:.2f} ppm (least squares vs Theil-Sen): "
                         "inspect the raw data")
    if out["rtt_ms"]["median"] > 10 * max(min_rtt_ms, 1e-6):
        notes.append("median RTT is >10x the minimum: noisy path or busy host")
    out["notes"] = notes
    out["quality"] = "ok" if not notes else "check notes"
    return out


def print_summary(s, meta):
    f, r, sr = s["fit"], s["rtt_ms"], s.get("series", {})
    lc = (meta.get("local_clock") or {}).get("name", "?")
    pi = meta.get("peer_info") or {}
    pc = (pi.get("clock") or {}).get("name", meta.get("peer_clock_label") or "?")
    print(f"\nlocal clock        {lc}")
    print(f"peer clock         {pc}   ({pi.get('host', '?')})")
    print(f"received           {s['received']}   lost {s['lost']} ({s['loss_pct']:.2f}%)")
    print(f"duration s         {s['duration_s']:.2f}")
    print(f"drift ppm          {f['drift_ppm']:+.3f}  least squares, lowest {f['frac']:.0%} RTT "
          f"(se {f['drift_se_ppm']:.3f}, lower bound)")
    if "theil_sen_drift_ppm" in sr:
        print(f"drift ppm          {sr['theil_sen_drift_ppm']:+.3f}  Theil-Sen, "
              f"{sr['n_points']} best-of-{sr['bin_s']:g}s points")
    print(f"drift resolution   ~{s['drift_resolution_ppm']:.2f} ppm (heuristic)")
    print(f"offset at t0 ms    {f['offset_at_t0_ms']:+.6f}   (peer - local)")
    print(f"offset at end ms   {f['offset_at_end_ms']:+.6f}")
    print(f"residual std ms    {f['residual_std_ms']:.6f}   (fit on {f['n']} lowest-RTT pings)")
    print(f"rtt ms             min {r['min']:.6f}  median {r['median']:.6f}  "
          f"p95 {r['p95']:.6f}  max {r['max']:.6f}")
    print(f"asymmetry bound ms +/- {s['asymmetry_bound_ms']:.6f}  (= min RTT / 2, offset only)")
    if s["windows"]:
        print("drift per window (ppm): " +
              "  ".join(f"{w['drift_ppm']:+.1f}" for w in s["windows"]))
    print_adev(sr)
    for note in s["notes"]:
        print(f"NOTE: {note}")
    print(f"quality            {s['quality']}")


def print_adev(sr):
    if sr.get("adev"):
        print("allan deviation (tau s: ppm): " +
              "  ".join(f"{p['tau_s']:g}: {p['adev_ppm']:.4f}" for p in sr["adev"]))


# ------------------------------------------------------------------ run modes
def run_session(p, args, rng):
    rows, lost = [], 0
    seqs = itertools.count(1)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        seq = next(seqs)
        try:
            rows.append((seq,) + p.ping(seq))
        except OSError:
            lost += 1
            continue
        if args.interval:
            time.sleep(args.interval * (0.8 + 0.4 * rng.random()))
    return rows, lost


def run_track(p, args, path):
    seqs = itertools.count(1)
    start = time.monotonic()
    k = written = 0
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t1_ns", "offset_ns", "rtt_ns", "n_ok"])
        try:
            while args.duration <= 0 or time.monotonic() - start < args.duration:
                best, ok = None, 0
                for _ in range(args.burst):
                    seq = next(seqs)
                    try:
                        t1, t2, t3, t4 = p.ping(seq)
                    except OSError:
                        continue
                    ok += 1
                    rtt = (t4 - t1) - (t3 - t2)
                    if best is None or rtt < best[2]:
                        best = (t1, ((t2 - t1) + (t3 - t4)) / 2.0, rtt)
                if best:
                    w.writerow([best[0], f"{best[1]:.1f}", best[2], ok])
                    fh.flush()
                    written += 1
                    if written % 10 == 0:
                        print(f"  {written} points, last offset {best[1] * 1e-6:+.4f} ms, "
                              f"rtt {best[2] * 1e-6:.4f} ms")
                k += 1
                delay = start + k * args.period - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        except KeyboardInterrupt:
            pass
    return written


def analyze_file(path, args):
    """Offline analysis of a *_raw.csv (session) or *_track.csv (track)."""
    with open(path, newline="") as fh:
        rd = csv.reader(fh)
        header = next(rd)
        data = [r for r in rd if r]
    prefix = path.rsplit(".", 1)[0]
    for suffix in ("_raw", "_track"):
        if prefix.endswith(suffix):
            prefix = prefix[: -len(suffix)]
    meta = {}
    for cand in (prefix + "_summary.json", prefix + "_track_meta.json"):
        if os.path.exists(cand):
            with open(cand) as fh:
                meta = json.load(fh).get("meta", {})
            break

    if header[0] == "seq":
        rows = [tuple(int(v) for v in r) for r in data]
        s = analyze(rows, args.lost, args.fit_frac, args.window, args.bin)
        print_summary(s, meta)
        result = s
    elif header[0] == "t1_ns":
        t1s = [int(r[0]) for r in data]
        offs = [float(r[1]) for r in data]
        rtts = [float(r[2]) for r in data]
        if len(t1s) < 8:
            sys.exit("too few track points")
        t_s = [(t - t1s[0]) * 1e-9 for t in t1s]
        gaps = sorted(b - a for a, b in zip(t_s, t_s[1:]))
        bin_s = args.bin if args.bin_set else max(round(gaps[len(gaps) // 2], 3), 1e-3)
        seen, keys, ts, xs = set(), [], [], []
        ref = offs[0]
        for t, o in zip(t_s, offs):
            k = int(round(t / bin_s))
            if k in seen:
                continue
            seen.add(k)
            keys.append(k)
            ts.append(t)
            xs.append((o - ref) * 1e-6)
        res, notes = analyze_series(ts, xs, keys, bin_s, 0.0, min(rtts) * 1e-6)
        print(f"\ntrack file         {path}  ({len(t1s)} points, period ~{bin_s:g} s, "
              f"duration {t_s[-1]:.1f} s)")
        if "theil_sen_drift_ppm" in res:
            print(f"drift ppm          {res['theil_sen_drift_ppm']:+.3f}  Theil-Sen")
            print(f"drift ppm          {res['ls_drift_ppm']:+.3f}  least squares "
                  f"(se {res['ls_se_ppm']:.3f}, lower bound)")
            print(f"residual std ms    {res['residual_std_ms']:.6f}")
        print(f"rtt ms             min {min(rtts) * 1e-6:.6f}  "
              f"median {statistics.median(rtts) * 1e-6:.6f}")
        print_adev(res)
        for n in notes:
            print(f"NOTE: {n}")
        result = {"schema_version": SCHEMA_VERSION, "series": res, "notes": notes}
    else:
        sys.exit("unrecognised CSV (expected *_raw.csv or *_track.csv from this tool)")
    with open(prefix + "_analysis.json", "w") as fh:
        json.dump({"meta": meta, "result": result}, fh, indent=2)
    print(f"\nsaved: {prefix}_analysis.json")


def main():
    ap = argparse.ArgumentParser(description="Clock offset/drift calibration client")
    ap.add_argument("peer", nargs="?")
    ap.add_argument("--port", type=int, default=9999)
    ap.add_argument("--duration", type=float, default=None,
                    help="seconds (session default 600; track default 0 = until Ctrl+C)")
    ap.add_argument("--interval", type=float, default=0.002, help="mean pause between pings (s)")
    ap.add_argument("--timeout", type=float, default=0.2, help="per-ping budget (s)")
    ap.add_argument("--warmup", type=int, default=200, help="pings discarded before measuring")
    ap.add_argument("--fit-frac", type=float, default=0.1)
    ap.add_argument("--window", type=float, default=30.0)
    ap.add_argument("--bin", type=float, default=None,
                    help="seconds per best-of-bin point for Theil-Sen/Allan (default 1)")
    ap.add_argument("--clock", choices=CLOCKS, default="auto")
    ap.add_argument("--peer-label", default="", help="free-text note about the peer clock")
    ap.add_argument("--out", default=None, help="output file prefix")
    ap.add_argument("--track", action="store_true")
    ap.add_argument("--period", type=float, default=1.0)
    ap.add_argument("--burst", type=int, default=50)
    ap.add_argument("--cpu", type=int, default=None, help="pin to this CPU (Linux)")
    ap.add_argument("--rt", type=int, default=None, metavar="PRIO",
                    help="SCHED_FIFO priority 1-99 (Linux, needs root)")
    ap.add_argument("--seed", type=int, default=None, help="RNG seed (recorded)")
    ap.add_argument("--analyze", metavar="FILE", help="offline analysis of a saved CSV")
    ap.add_argument("--lost", type=int, default=0, help="lost-ping count for --analyze")
    args = ap.parse_args()
    args.bin_set = args.bin is not None
    if args.bin is None:
        args.bin = 1.0

    if args.analyze:
        return analyze_file(args.analyze, args)
    if not args.peer:
        ap.error("peer address required (or use --analyze FILE)")
    if args.duration is None:
        args.duration = 0.0 if args.track else 600.0
    if args.seed is None:
        args.seed = random.getrandbits(32)
    rng = random.Random(args.seed)

    now, desc = make_clock(args.clock)
    applied = tune(args.cpu, args.rt)
    desc.update(measure_tick(now))
    p = Prober(args.peer, args.port, now, args.timeout)
    peer_info = p.info()
    if peer_info is None:
        sys.exit("responder not reachable (check IP, port, firewall, and that srv.py v2 runs)")
    warns = []
    pk = (peer_info.get("clock") or {}).get("kind")
    if pk != desc["kind"]:
        warns.append(f"clock kinds differ (local {desc['kind']}, peer {pk}): "
                     "slewing of either side is included in the drift")
    if "wall" in (pk, desc["kind"]):
        warns.append("a wall clock is involved: NTP/chrony steps and slews are included")
    for wmsg in warns:
        print(f"WARNING: {wmsg}")

    got = 0
    for seq in range(1, args.warmup * 20 + 1):
        if got >= args.warmup:
            break
        try:
            p.ping(0xFFFF0000 + seq)
            got += 1
        except OSError:
            pass
    p.drain()

    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d_%H%M%S")
    prefix = args.out or f"calib_{platform.node()}_{stamp}"
    meta = {
        "tool": "cli.py", "tool_version": TOOL_VERSION, "protocol": MAGIC.decode(),
        "utc": stamp, "host": platform.node(), "platform": platform.platform(),
        "python": platform.python_version(), "cpu_count": os.cpu_count(),
        "clocksource": clocksource(), "local_clock": desc, "tuning": applied,
        "peer": f"{args.peer}:{args.port}", "peer_info": peer_info,
        "peer_clock_label": args.peer_label, "session_id": p.sid, "seed": args.seed,
        "warnings": warns, "params": vars(args),
    }

    if args.track:
        path = prefix + "_track.csv"
        print(f"tracking (clock {desc['name']}); Ctrl+C to stop -> {path}")
        n = run_track(p, args, path)
        with open(prefix + "_track_meta.json", "w") as fh:
            json.dump({"meta": meta, "points": n}, fh, indent=2)
        print(f"saved {n} points: {path}")
        print(f"analyze later with: python3 cli.py --analyze {path}")
        return

    print(f"measuring for {args.duration:.0f} s (clock {desc['name']}, peer "
          f"{(peer_info.get('clock') or {}).get('name')}) ...")
    rows, lost = run_session(p, args, rng)
    try:
        s = analyze(rows, lost, args.fit_frac, args.window, args.bin)
    except ValueError as e:
        sys.exit(str(e))
    s["notes"] = warns + s["notes"]
    s["quality"] = "ok" if not s["notes"] else "check notes"
    print_summary(s, meta)

    with open(prefix + "_raw.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["seq", "t1_ns", "t2_ns", "t3_ns", "t4_ns"])
        w.writerows(rows)
    with open(prefix + "_summary.json", "w") as fh:
        json.dump({"meta": meta, "result": s}, fh, indent=2)
    print(f"\nsaved: {prefix}_raw.csv, {prefix}_summary.json")


if __name__ == "__main__":
    main()
