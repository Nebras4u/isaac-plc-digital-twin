# Clock Offset / Drift Calibration Report
## Windows 10 guest (QEMU/KVM) vs Ubuntu 24.04 host

## Setup
- Host: Ubuntu 24.04, TSC clocksource (stable per dmesg)
- Guest: Windows 10 in QEMU/KVM
- cli.py on Windows, srv.py --clock perf on host
- 31,851 pings over ≈ 756.7 s

## Key result
Mean drift of the guest clock relative to the host: ≈ 80 ppm
  = 4.8 ms/min = 0.29 s/hour

Drift is NOT constant: window-to-window spread exceeds 20 ppm,
with occasional jumps (seq ≈ 15680, 21000) attributable to vCPU
scheduling.

## RTT
min 1.2 ms, median 2.0–2.5 ms, max 4–5 ms
→ hard floor on offset error: ±0.6 ms (min_rtt / 2)

## Interpretation
This number is the *practical* timing error of a Windows guest
in this VM configuration. It includes:
  - host TSC stability (good)
  - QEMU emulation overhead (dominant, likely)
  - vCPU scheduling jitter
  - actual Windows clock drift

It does NOT measure the intrinsic oscillator drift of the
Windows machine's hardware.

## Recommendations
1. Use --track, not a single offset.
2. Fix QEMU: enable hypervclock, disable hpet, pin vCPUs.
3. Re-run to separate emulation error from real drift.
4. For pure oscillator measurement: use two physical machines.
