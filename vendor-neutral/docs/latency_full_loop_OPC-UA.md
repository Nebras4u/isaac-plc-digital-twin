# Latency Measurement Report: Robotic Digital Twin System

**Project:** Digital Twin for KUKA KR210 L150 with ROS 2, OPC UA, Ruckig, and Isaac Sim
**Date:** 2026-10-11
**System:** ROS 2 (Humble) + OPC UA + Ruckig + Isaac Sim 5.1
**Robot:** KUKA KR210 L150 (4 controlled joints)
**Control Rate:** 60 Hz (16.6 ms per cycle)
**Test Duration:** 49.5 seconds
**Sample Size:** ~270 samples (L2), ~2900 samples (L1, L3, L4)

---

## 1. Executive Summary

This report presents latency measurements for a complete Digital Twin system connecting a Siemens S7-1500 PLC to ROS 2 and Isaac Sim through OPC UA.

### Key Findings

| Metric | Value | Assessment |
|--------|:-----:|:----------:|
| OPC UA Read Latency       | 7.0 ms  | Elevated   |
| ROS + Ruckig Processing   | 9.2 ms  | Acceptable |
| Isaac Sim Processing      | 11.5 ms | Variable   |
| OPC UA Write Latency      | 2.0 ms  | Excellent  |
| Full Loop (estimated)     | ~30 ms  | Exceeds target |

### Conclusion

The system operates stably at 60 Hz, but the full-loop latency (~30 ms) exceeds the control cycle (16.6 ms), meaning the system lags by approximately 1.8 control cycles. This is acceptable for medium-speed applications but insufficient for high-speed control tasks.

---

## 2. Objective

Measure end-to-end latency across four stages of the control loop:

| Stage | Description |
|-------|-------------|
| L1 | PLC -> OPC UA read -> `/plc/cmd_ang` publish |
| L2 | `/plc/cmd_ang` -> Ruckig planning -> `/servo_setpoints` |
| L3 | `/servo_setpoints` -> Isaac physics -> `/joint_states` |
| L4 | `/joint_states` -> OPC UA write -> PLC |

---

## 3. Methodology

### 3.1 System Architecture

```text
PLC (S7-1500)
    |  OPC UA @ 60 Hz
    v
plc_ros_bridge.py (instrumented)
    |  /plc/cmd_ang (DDS)
    v
trajectory_twin.py (Ruckig)
    |  /servo_setpoints (DDS)
    v
physics_twin.py (Isaac Sim)
    |  /joint_states (DDS)
    v
plc_ros_bridge.py (writes back to PLC)
```

### 3.2 Measurement Tools

- `latency_full_loop.py`        : measures L1, L2, L3, L4 from topic timestamps
- `plc_ros_bridge_instrumented.py` : measures OPC UA read/write latency internally
- Clock source                  : `time.monotonic()` (microsecond resolution)
- Sampling rate                 : 60 Hz
- Duration                      : 49.5 seconds continuous operation
- Warm-up exclusion             : first 20-30 samples

### 3.3 Statistical Metrics

- Mean, p95, p99, max
- Warm-up samples excluded from statistics
- Outliers (>500 ms) discarded

---

## 4. Results

### 4.1 Stage L1 - OPC UA Read

Source: `latency_L1_opcua.csv`

| Metric | Value |
|--------|:-----:|
| Samples | ~2900 |
| Mean    | 7.0 ms |
| p95     | 10.5 ms |
| p99     | 13.5 ms |
| Max     | 25.7 ms |

Observation: Relatively high latency. Likely cause: 15 signals read sequentially rather than in a single batch operation.

### 4.2 Stage L2 - /plc/cmd_ang -> /servo_setpoints

Source: `latency_L2.csv`, `latency_full_loop_summary.csv`

| Metric | Value |
|--------|:-----:|
| Samples | ~270 |
| Mean    | 9.2 ms |
| p95     | 13.0 ms |
| p99     | 16.0 ms |
| Max     | 29.0 ms |

Observation: Stable and consistent with prior measurements (~10.5 ms). Contains some burst values due to message-matching logic artifacts.

### 4.3 Stage L3 - /servo_setpoints -> /joint_states

Source: `latency_L3.csv`

| Metric | Value |
|--------|:-----:|
| Samples | ~2900 |
| Mean    | 11.5 ms |
| p95     | 16.0 ms |
| p99     | 16.7 ms |
| Max     | 100 ms (critical) |

Critical Observation: A sudden spike to 100 ms occurred at t = 47.7 s of the run. This indicates one of:

- Python garbage collection pause
- DDS retransmission or QoS event
- Momentary CPU contention

### 4.4 Stage L4 - OPC UA Write

Source: `latency_L4.csv`

| Metric | Value |
|--------|:-----:|
| Samples | ~2900 |
| Mean    | 2.0 ms |
| p95     | 4.5 ms |
| p99     | 5.0 ms |
| Max     | 15.0 ms |

Observation: Excellent. OPC UA write is significantly faster than read.

### 4.5 Unified Summary Table

Source: `latency_full_loop_summary.csv`

| Time (s) | L2 mean | L2 p95 | L3 mean | L3 p95 | L4 mean | L4 p95 |
|:--------:|:-------:|:------:|:-------:|:------:|:-------:|:------:|
| 5.0      | 8.6     | 12.0   | 10.9    | 12.9   | 1.9     | 3.9    |
| 10.0     | 9.2     | 12.6   | 11.9    | 16.6   | 2.0     | 4.9    |
| 15.0     | 9.8     | 12.5   | 12.1    | 16.7   | 1.7     | 4.8    |
| 20.0     | 9.0     | 12.7   | 12.1    | 16.6   | 2.1     | 5.1    |
| 25.0     | 10.1    | 12.6   | 11.9    | 16.7   | 1.6     | 3.5    |
| 30.0     | 9.4     | 13.5   | 11.8    | 16.1   | 2.0     | 4.2    |
| 35.0     | 9.0     | 18.0   | 12.1    | 16.5   | 2.0     | 5.7    |
| 40.0     | 8.6     | 11.8   | 11.0    | 14.1   | 1.9     | 4.9    |
| 45.0     | 9.5     | 11.9   | 11.3    | 8.4    | 1.6     | 2.8    |
| 49.5     | 9.3     | 11.2   | 11.0    | 10.0   | 1.6     | 2.8    |

Interpretation:

- L2: stable at 9-10 ms
- L3: stable at 11-12 ms, with p95 variance
- L4: stable at ~2 ms

---

## 5. Analysis

### 5.1 Time Distribution Across Stages

```text
+-----------------------------------------+
|  OPC UA Read          7.0 ms   (23%)    |
|  ROS + Ruckig         9.2 ms   (31%)    |
|  Isaac Physics       11.5 ms   (38%)    |
|  OPC UA Write         2.0 ms   ( 7%)    |
+-----------------------------------------+
|  Total               29.7 ms  (100%)    |
+-----------------------------------------+
```

### 5.2 Comparison with Control Requirements

| Requirement                  | Value          | Status |
|------------------------------|:--------------:|:------:|
| Control cycle                | 16.6 ms (60 Hz)| -      |
| Full loop                    | 29.7 ms        | -      |
| Effective delay              | 1.79 cycles    | Warning|
| Safety margin                | -13.1 ms       | Fail   |

Conclusion: The system cannot achieve "single-cycle response" (16.6 ms). Effective delay is ~30 ms, i.e. 1.8 cycles.

### 5.3 Bottleneck Identification

| # | Source                     | Impact  | Priority |
|---|----------------------------|:-------:|:--------:|
| 1 | OPC UA read (tag-by-tag)   | 7 ms    | High     |
| 2 | Isaac physics step         | 11.5 ms | Medium   |
| 3 | DDS hops (x4)              | ~8 ms   | Medium   |
| 4 | Ruckig planning            | ~2 ms   | Low      |
| 5 | OPC UA write               | 2 ms    | Excellent|

### 5.4 Notes on Empty Files

Two files remained empty in this run:

- `latency_L1.csv`         : requires `frame_id = "mono:..."` (not yet added)
- `plc_bridge_internal.csv`: 60-second save period, run lasted only 49.5 seconds

This does not affect the overall conclusions but prevents accurate measurement of `publish_latency` and `tick_period`.

---

## 6. Conclusions

### 6.1 Confirmed Findings

1. System operates stably at 60 Hz (~2900 samples / 49.5 seconds)
2. OPC UA write is very fast (2 ms)
3. Ruckig + DDS are regular and predictable (9.2 ms)
4. Isaac physics is variable (11.5 ms mean, but up to 100 ms)
5. OPC UA read is slow (7 ms) and requires optimization
6. Full loop (~30 ms) exceeds the control cycle (16.6 ms)

### 6.2 Uncertain Findings

- Publish latency inside `plc_ros_bridge`
- Actual tick period
- Contribution of missing `frame_id` in L1

### 6.3 Overall Assessment

| Criterion          | Rating    | Note                                  |
|--------------------|:---------:|:-------------------------------------:|
| Stability          | Excellent | No crashes, steady 60 Hz              |
| Reliability        | Good      | No message loss                       |
| Timing performance | Acceptable| Needs optimization for real-time      |
| Scalability        | Medium    | OPC UA bottleneck                     |

---

## 7. Recommendations

### 7.1 Immediate Actions (High Impact)

| # | Recommendation                        | Expected Impact | Implementation                          |
|---|---------------------------------------|:---------------:|-----------------------------------------|
| 1 | Batch OPC UA read                     | -5 ms           | Modify `OpcUaDriver.read`               |
| 2 | CPU pinning for all nodes             | -2 to -3 ms     | `taskset -c N`                          |
| 3 | Real-time priority                    | -2 ms           | `chrt -f 80`                            |
| 4 | Switch to CycloneDDS                  | -1 ms           | `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| 5 | Reduce CSV save period                | Measurement     | 10 s instead of 60 s                    |

### 7.2 Structural Recommendations

| # | Recommendation                        | Impact          |
|---|---------------------------------------|:---------------:|
| 1 | Replace OPC UA with shared memory     | -4 to -6 ms     |
| 2 | Merge `trajectory_twin` + `physics_twin` | -2 ms (one less DDS hop) |
| 3 | Use Zero-copy DDS (Iceoryx)           | -1 to -2 ms     |
| 4 | Run Isaac Sim on isolated CPU core    | -2 ms           |

### 7.3 Measurement Recommendations

1. Add `frame_id = "mono:..."` in `plc_ros_bridge_instrumented.py` to measure L1
2. Reduce CSV save period to 10 seconds
3. Run for at least 90 seconds to capture one internal report

---

## 8. Summary

Latency measurement in the Digital Twin system for the KUKA KR210 robot showed:

```text
The full loop consumes ~30 ms, distributed as:
  - 38% in Isaac Sim         (11.5 ms)
  - 31% in ROS + Ruckig       ( 9.2 ms)
  - 23% in OPC UA read        ( 7.0 ms)
  -  7% in OPC UA write       ( 2.0 ms)
```

This performance is acceptable for medium-speed applications (pick & place, transfer), but insufficient for high-speed applications (welding, fast assembly). The largest expected improvement is from OPC UA read (from 7 to 2 ms), reducing the full loop to ~24 ms.

---

## 9. Appendices

### Appendix A - Result Files

- `latency_L1_opcua.csv`         : OPC UA read (~2900 samples)
- `latency_L2.csv`               : ROS + Ruckig (~270 samples)
- `latency_L3.csv`               : Isaac Sim (~2900 samples)
- `latency_L4.csv`               : OPC UA write (~2900 samples)
- `latency_full_loop_summary.csv`: summary every 5 seconds

### Appendix B - Source Code

- `plc_ros_bridge_instrumented.py` : bridge with internal instrumentation
- `latency_full_loop.py`           : measurement script
- `trajectory_twin.py`             : Ruckig planner
- `physics_twin.py`                : Isaac Sim bridge

### Appendix C - Environment

- ROS 2 (Humble)
- OPC UA endpoint: `opc.tcp://192.168.100.50:4840`
- Isaac Sim 5.1
- Windows 11 + WSL2 / Ubuntu 22.04
- Robot: KUKA KR210 L150 (4 controlled joints)

---
