# CLOCK OFFSET / DRIFT CALIBRATION REPORT

**Windows 10 (DESKTOP-7RFMLG3) ↔ Ubuntu 24.04 (Nebras)**

---

## 1. EXECUTIVE SUMMARY

| Item                  | Value                                              |
|-----------------------|----------------------------------------------------|
| Client (`cli.py`)     | Windows 10, `DESKTOP-7RFMLG3`, Python 3.12.3       |
| Reference (`srv.py`)  | Ubuntu, `Nebras`, clocksource `tsc`, Python 3.12.3 |
| Peer address          | `111.111.111.2:9999`                               |
| Measurement duration  | **299.98 s** (~5 minutes)                          |
| Pings received        | **97,471**                                         |
| Packet loss           | **0.00 %** (0 of 97,471)                           |
| Drift (least squares) | **+95.73 ppm**                                     |
| Drift (Theil-Sen)     | **+95.81 ppm**                                     |
| Quality               | **ok** (no warnings)                               |
| Slope resolution      | ±0.20 ppm (heuristic)                              |

---

## 2. MEASUREMENT SETUP

### 2.1 Client (Windows)

| Item            | Value                   |
|-----------------|-------------------------|
| CPU cores       | 4                       |
| Clock           | `perf_counter_ns` (QPC) |
| Resolution      | 100 ns                  |
| Minimum step    | 300 ns                  |
| Clock call cost | 388 ns                  |
| GC              | disabled                |

### 2.2 Reference (Linux)

| Item            | Value                                 |
|-----------------|---------------------------------------|
| CPU cores       | 24                                    |
| Clocksource     | `tsc`                                 |
| Clock           | `perf_counter_ns` (`CLOCK_MONOTONIC`) |
| Resolution      | 1 ns                                  |
| Minimum step    | 56 ns                                 |
| Clock call cost | 67.9 ns                               |
| GC              | disabled                              |

Both sides use the same clock kind (`perf`). The comparison is homogeneous: no `raw` vs `mono` or `wall` vs `monotonic` mixing.

---

## 3. MAIN RESULT

### 3.1 Drift

    drift = +95.73 ppm   (least squares, lowest 10 % RTT)
            +95.81 ppm   (Theil-Sen, best-of-1s points)

Interpretation:

- Linux clock runs **~95.7 ppm faster** than Windows clock
- **~5.74 ms per minute**
- **~0.344 s per hour**
- **~8.26 s per day**

### 3.2 Offset

    offset at t0   = +8,785,337.617341 ms   (peer − local)
    offset at end  = +8,785,366.333791 ms
    delta offset   = +28.716 ms over 300 s

    delta offset / delta t = 28.716 ms / 300 s ≈ 95.7 ppm  (consistent)

### 3.3 Estimator agreement

| Estimator                         | Value (ppm)   |
|-----------------------------------|---------------|
| Least squares (lowest 10 % RTT)   | +95.728       |
| Theil-Sen (300 points, bin = 1 s) | +95.810       |
| Difference                        | **0.082 ppm** |

The two agree to better than 0.1 ppm → result is **robust**.

---

## 4. LINK QUALITY

### 4.1 Round-trip time (RTT)

| Metric  | Value (ms) |
|---------|------------|
| Minimum | **0.166**  |
| Median  | 0.598      |
| p95     | 0.848      |
| Maximum | 1.436      |

- min RTT = 0.166 ms → excellent for a wired LAN.
- median / min = 3.6 → well below 10× warning threshold.
- Hard floor on offset error = `min_rtt / 2` = **±0.083 ms**.

### 4.2 Packet loss

    lost = 0 of 97,471   (0.00 %)

### 4.3 Drift stability across 30-s windows

    drift per window (ppm):
      +95.6  +96.1  +95.1  +95.8  +95.6  +95.4  +96.1  +95.6  +97.1

- Total range: **1.99 ppm** only.
- Drift is essentially **constant** across the 5-minute run.

---

## 5. ALLAN DEVIATION

| τ (s) | adev (ppm) | n   |
|-------|------------|-----|
| 1     | 58.22      | 298 |
| 2     | 31.63      | 296 |
| 4     | 15.17      | 292 |
| 8     | 7.69       | 284 |
| 16    | 4.12       | 268 |
| 32    | 1.85       | 236 |
| 64    | 0.92       | 172 |
| 128   | 0.52       | 44  |

- At τ = 1 s, adev is dominated by measurement noise.
- At τ = 128 s, adev drops to **0.52 ppm** → clock is stable.
- Curve falls as `1/√τ` → white-noise behaviour.

---

## 6. CORRECTION MODEL

### 6.1 Simple linear correction

    offset(t) ≈ offset_at_t0 + drift × (t − t0)
    offset(t) ≈ 8,785,337.617 + 0.095728 × (t − t0)   [ms, t in seconds]

    local_time = peer_time − offset(t)

### 6.2 Long sessions (> 10 minutes)

    drift_range_ppm = 1.99   (well below 20 ppm)

- Fixed offset + linear drift is sufficient.
- For sessions > 1 hour, use `--track`.

### 6.3 Correction accuracy

| Metric                      | Value                      |
|-----------------------------|----------------------------|
| Absolute offset uncertainty | ±0.083 ms (= min_rtt / 2)  |
| Drift uncertainty           | ±0.20 ppm                  |
| Fit residual std            | 0.030 ms                   |
| **Overall accuracy**        | **±0.1 ms** over 5 minutes |

---

## 7. COMPARISON WITH PREVIOUS (QEMU/KVM) RUN

| Metric           | QEMU/KVM guest     | Physical machines      | Improvement |
|------------------|--------------------|------------------------|-------------|
| min RTT          | 1.2 ms             | **0.166 ms**           | **7.2×**    |
| median RTT       | 2.0–2.5 ms         | **0.598 ms**           | ~4×         |
| Fitted drift     | ~80 ppm (variable) | **95.73 ppm (const.)** | stable now  |
| Drift range      | > 20 ppm           | **1.99 ppm**           | **> 10×**   |
| Fit se           | large              | **0.004 ppm**          | very high   |
| Fit residual std | large              | **0.030 ms**           | —           |
| Quality          | check notes        | **ok**                 | ✅          |

The physical-machine setup removes the QEMU emulation overhead and produces a stable, well-characterized drift.

---

## 8. CONCLUSIONS

- The two physical machines drift at a constant rate of **95.7 ppm**.
- Linux clock is consistently **ahead** of the Windows clock.
- The link is clean: **0 % loss**, min RTT = **0.166 ms**.
- The drift is stable (range **1.99 ppm** over 5 minutes).
- A single linear correction is sufficient for sessions up to ~10 min.
- For longer sessions, use `--track` and interpolate.
- Achievable correction accuracy: **±0.1 ms**.

---

## 9. RECOMMENDATIONS

1. Use the linear model in §6.1 for sessions ≤ 10 minutes.
2. Switch to `--track` for sessions > 1 hour.
3. Keep NTP/chrony disabled on both machines for pure measurement.
4. Repeat the run 3 times at different times of day to confirm.
5. Store the calibration coefficients with each experiment.

---

*END OF REPORT*
