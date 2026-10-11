#!/usr/bin/env python3
"""Clock calibration reference responder (pairs with cli.py, protocol CLK1).

Run on one machine and cli.py on the other:

    python3 srv.py [--port 9999] [--bind 0.0.0.0] [--clock auto|perf|mono|raw|wall]
                   [--cpu N] [--rt PRIO]

Wire format (UDP, network byte order). Every ping packet is exactly 48 bytes in
BOTH directions, so serialization delay is symmetric. Unused bytes are zero.

    header : 'CLK1' | type u8 | flags u8 | reserved u16 | session u32 | seq u32   (16 B)
    ping   : request  header(type=1)
             reply    header(type=1) | t2 i64 | t3 i64
    info   : request  header(type=2)
             reply    header(type=2) | JSON description of this responder

t2 = request received, t3 = reply sent, both read from THIS machine's clock
(ns). t3 is stamped last, after the reply is built (one 8-byte field is written
afterwards), so responder processing stays outside the measured round trip.

Clocks (--clock):
    auto : raw if available, otherwise perf (default; the choice is reported)
    perf : time.perf_counter_ns()      (CLOCK_MONOTONIC on Linux, QPC on Windows)
    mono : time.monotonic_ns()         (slewed by NTP/chrony on Linux)
    raw  : CLOCK_MONOTONIC_RAW         (Linux only; NOT frequency-adjusted)
    wall : time.time_ns()              (system clock, includes NTP steps/slews)
An unavailable clock is a hard error, never a silent fallback. The client reads
the clock description from the info reply and stores it with the results.

Tuning (best effort, recorded in the info reply): the garbage collector is
disabled; --cpu pins the process (Linux); --rt sets SCHED_FIFO (Linux, root).



# الجهاز الأول
sudo python3 srv.py --clock perf

# الجهاز الثاني
python3 cli.py <ip> --duration 600
"""
import argparse
import functools
import gc
import json
import os
import platform
import socket
import struct
import sys
import time

TOOL_VERSION = "2.0"
MAGIC = b"CLK1"
T_PING, T_INFO = 1, 2
PKT = 48
HDR = struct.Struct("!4sBBHII")      # 16 bytes
REP = struct.Struct("!4sBBHIIqq")    # 32 bytes (+16 zero padding on the wire)
T3 = struct.Struct("!q")
T3_OFFSET = HDR.size + 8
CLOCKS = ("auto", "perf", "mono", "raw", "wall")


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


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=9999)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--clock", choices=CLOCKS, default="auto")
    ap.add_argument("--cpu", type=int, default=None, help="pin to this CPU (Linux)")
    ap.add_argument("--rt", type=int, default=None, metavar="PRIO",
                    help="SCHED_FIFO priority 1-99 (Linux, needs root)")
    args = ap.parse_args()

    now, desc = make_clock(args.clock)
    applied = tune(args.cpu, args.rt)
    desc.update(measure_tick(now))
    info = {
        "tool": "srv.py", "tool_version": TOOL_VERSION, "protocol": MAGIC.decode(),
        "host": platform.node(), "platform": platform.platform(),
        "python": platform.python_version(), "pid": os.getpid(),
        "cpu_count": os.cpu_count(), "clocksource": clocksource(),
        "clock": desc, "tuning": applied,
    }
    info_payload = json.dumps(info).encode()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind((args.bind, args.port))
    print(f"responder on {args.bind}:{args.port} | clock {desc['name']} "
          f"(res {desc['resolution_ns']:.0f} ns, step {desc['min_step_ns']} ns, "
          f"call {desc['call_cost_ns']:.0f} ns) | tuning {applied} | "
          f"{platform.platform()} | python {platform.python_version()}")

    buf = bytearray(256)
    reply = bytearray(PKT)
    recv, send = sock.recvfrom_into, sock.sendto
    unpack, pack, pack_t3 = HDR.unpack_from, REP.pack_into, T3.pack_into
    served = infos = 0
    try:
        while True:
            try:
                n, addr = recv(buf)
            except OSError:  # e.g. ConnectionResetError on Windows
                continue
            t2 = now()
            if n != PKT:
                continue
            magic, typ, _, _, sid, seq = unpack(buf, 0)
            if magic != MAGIC:
                continue
            if typ == T_PING:
                pack(reply, 0, MAGIC, T_PING, 0, 0, sid, seq, t2, 0)
                pack_t3(reply, T3_OFFSET, now())
                try:
                    send(reply, addr)
                except OSError:
                    continue
                served += 1
            elif typ == T_INFO:
                try:
                    send(HDR.pack(MAGIC, T_INFO, 0, 0, sid, seq) + info_payload, addr)
                except OSError:
                    continue
                infos += 1
    except KeyboardInterrupt:
        print(f"\nstopped, served {served} pings, {infos} info requests")


if __name__ == "__main__":
    main()
