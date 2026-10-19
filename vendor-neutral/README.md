# Isaac Digital Twin — Vendor-Neutral Platform

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![PLC](https://img.shields.io/badge/PLC-Vendor--Neutral-lightgrey)](#supported-plcs)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-5.1-green)](https://developer.nvidia.com/isaac-sim)
[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.11-yellow)](https://www.python.org)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?logo=docker&logoColor=white)](https://www.docker.com/)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen)]()
[![Latency](https://img.shields.io/badge/Mean%20latency-5.2%20ms-success)](#control-loop-latency)
[![Max](https://img.shields.io/badge/Max%20latency-16.3%20ms-success)](#control-loop-latency)

A **vendor-neutral Digital Twin platform** for industrial robot arms. The platform synchronizes any robot arm with any PLC through ROS 2, using a transport-agnostic bridge and a robot-agnostic configuration format.

This folder (`vendor-neutral/`) is the evolution of two robot-specific implementations (KUKA KR210 and Yaskawa GP110) into a reusable, configuration-driven platform.

---

## Safety Notice

> **This project is a simulation / digital-twin research platform. It is
> NOT a certified safety function and must not be used to protect people
> or physical machinery.**

- The e-stop, heartbeat, and watchdog logic (bridge, trajectory twin, PLC
  counters) is **functional monitoring** that stops the *simulated* robot.
  It is not designed, validated, or certified according to IEC 61508,
  ISO 13849-1, or IEC 62061.
- `tcp_json`, OPC UA, and ROS 2 links carry no safety-rated integrity
  (no black channel, no safety protocol).
- A real installation needs an independent certified safety layer (for
  example an F-CPU with PROFIsafe and hardware E-Stop circuits).

---

## Table of Contents

- [Safety Notice](#safety-notice)
- [Motivation](#motivation)
- [What "Vendor-Neutral" Means](#what-vendor-neutral-means)
- [Architecture](#architecture)
- [Repository Structure](#repository-structure)
- [Supported Combinations](#supported-combinations)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Results](#results)
  - [Tracking Accuracy](#tracking-accuracy)
  - [Control-Loop Latency](#control-loop-latency)
  - [Bridge Real-Time Behaviour](#bridge-real-time-behaviour)
  - [Clock Drift Calibration](#clock-drift-calibration)
- [Performance Tuning](#performance-tuning)
- [Documentation](#documentation)
- [Lineage](#lineage)
- [Roadmap](#roadmap)
- [License](#license)
- [Contact](#contact)

---

## Motivation

Previous generations were tightly coupled to a specific robot and a specific PLC transport. Every migration required rewriting multiple files.

This folder removes those constraints:

> **Switch robots and PLCs by editing YAML — not by editing Python.**

Three abstractions make this possible:

1. **Transport abstraction** — `plc_drivers.py` provides a common `PlcDriver` interface with three implementations: `tcp_json`, `opcua`, `plc_api`.
2. **Signal abstraction** — `plc_config.py` handles address mapping and value extraction, decoupled from any specific PLC protocol.
3. **Robot abstraction** — a single `robot.yaml` describes joints, limits, drives, and Isaac paths for any robot.

The main nodes (`trajectory_twin.py`, `plc_ros_bridge.py`, `physics_twin.py`) stay focused on their responsibility and delegate these concerns to the modules above.

---

## What "Vendor-Neutral" Means

| Axis             | Before                          | Now                                             |
| :----------------| :-------------------------------| :-----------------------------------------------|
| **PLC vendor**   | Siemens only                    | Any PLC with OPC UA, TCP/JSON, or PLCSIM API    |
| **Robot vendor** | KUKA only, then Yaskawa only    | Any articulated robot described in `robot.yaml` |
| **Transport**    | TCP/JSON only, then OPC UA only | Selected by a single YAML line                  |

The same code runs unchanged across all combinations. Only configuration differs.

---

## Architecture

    +----------------------------------------------------------------------+
    |                          PLC (any vendor)                            |
    |   Siemens S7-1500 - Beckhoff TwinCAT - Rockwell - OPC UA servers     |
    +--------------------------------+-------------------------------------+
                                     |
                           one of three transports
                                     |
            +------------------------+--------------------------+
            |                        |                          |
       tcp_json                    opcua                     plc_api
     (Windows server)           (native OPC UA)         (PLCSIM direct)
            |                        |                          |
            +------------------------+--------------------------+
                                     |
                        +------------v------------+
                        |  linux/plc_ros_bridge.py|
                        |  (+ plc_drivers.py)     |
                        |  (+ plc_config.py)      |
                        +------------+------------+
                                     |
                                ROS 2 topics
                                     |
                        +------------v------------+
                        |linux/trajectory_twin.py |
                        | (Ruckig + state machine)|
                        +------------+------------+
                                     |
                        +------------v------------+
                        |  isaac/physics_twin.py  |
                        |  (inside Isaac Sim)     |
                        +-------------------------+

### Runtime threading model

`plc_ros_bridge.py` runs on a **`SingleThreadedExecutor`**. This is deliberate:

- Timer (`tick()`) and subscriber (`on_joint_states()`) callbacks run strictly in sequence.
- They cannot race for the Python GIL mid-tick, which was causing 200-1000 ms `read` stalls on VMs.
- The 60 Hz tick is preserved; the only cost is that a subscriber callback can delay the next tick by its own runtime (~20-50 µs).

`trajectory_twin.py` and `physics_twin.py` use the default executor; they do not touch the same critical timing as the bridge.

---

## Repository Structure

    vendor-neutral/
    |
    +-- README.md                         # This file
    +-- CHANGELOG.md                      # Gen 3 history
    |
    +-- windows/
    |   +-- plc_server_win.py             # PLCSIM Advanced -> TCP/JSON
    |
    +-- linux/
    |   +-- trajectory_twin.py            # Ruckig trajectory + state machine
    |   +-- plc_ros_bridge.py             # ROS 2 <-> PLC bridge
    |   +-- plc_drivers.py                # 3 transports (tcp_json/opcua/plc_api)
    |   +-- plc_config.py                 # signal mapping + extractor
    |
    +-- isaac/
    |   +-- physics_twin.py               # Articulation driver
    |   +-- Main_Robot_L_URDF_tool.urdf
    |   +-- Full_Control.usd
    |
    +-- config/
    |   +-- plc_ros_bridge.yaml           # Signal mapping
    |   +-- robot.yaml                    # Joints, limits, Ruckig, drives
    |
    +-- tools/
    |   +-- latency_simpel.py             # Quick latency check
    |   +-- latency_academic.py           # LBP / p99.9 / jitter
    |   +-- latency_full_loop.py          # Full-loop L1+L2+L3+L4
    |   +-- compare_results.py            # A/B comparison tool
    |   +-- srv.py                        # Clock responder
    |   +-- cli.py                        # Clock drift client
    |   +-- info.txt
    |
    +-- plc/
    |   +-- SCL_heartbeat_OB30.txt
    |   +-- SCL_MAIN.txt
    |   +-- SCL_heartbeat_OB30-init.txt
    |   +-- SCL_MAIN-init.txt
    |   +-- DB.txt
    |
    +-- docs/
        +-- migration.md
        +-- latency_comparison.md         # Full comparison report
        +-- latency_full_loop_OPC-UA.md   # Baseline report

---

## Supported Combinations

| PLC                           | Transport  | Robot                     | Mean Latency | Status     |
| :---------------------------- | :--------- | :------------------------ | :----------: | :--------- |
| Siemens S7-1500 (PLCSIM Adv.) | `plc_api`  | KUKA KR210                | TBD          | Verified   |
| Siemens S7-1500 (PLCSIM Adv.) | `tcp_json` | KUKA KR210                | 5.2 ms       | Verified   |
| Any OPC UA server             | `opcua`    | Any robot in `robot.yaml` | ~7 ms        | Structural |
| Siemens S7-1500               | `opcua`    | Yaskawa GP110             | TBD          | Structural |

**"Structural"** = code path exercised, imports work; full end-to-end verification on that combination is pending.

---

## Quick Start

### Prerequisites

**Windows (only if using `tcp_json`):**

- Python 3.x with `pythonnet`
- PLCSIM Advanced V6.0
- `pip install msgpack`

**Linux (bridge + trajectory twin):**

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `pip install ruckig numpy pyyaml msgpack`

**Isaac Sim host:**

- Isaac Sim 5.1 (5.0 supported)
- ROS 2 Bridge extension enabled

### Startup Sequence

    # 1. Windows: TCP/JSON server (only if driver: tcp_json)
    cd windows && python plc_server_win.py --host 0.0.0.0 --port 5000

    # 2. Isaac Sim Script Editor: physics twin
    #    Run: isaac/physics_twin.py

    # 3. Linux: PLC <-> ROS bridge (with CPU pinning for latency)
    taskset -c 2 python3 plc_ros_bridge.py \
        --ros-args -p config:=../config/plc_ros_bridge.yaml

    # 4. Linux: trajectory twin (Ruckig trajectory generator)
    taskset -c 3 python3 trajectory_twin.py \
        --ros-args -p robot:=../config/robot.yaml

### Switching Transport

Edit one line in `config/plc_ros_bridge.yaml`:

    plc:
      driver: opcua          # change to: tcp_json | opcua | plc_api

Everything else stays the same.

### Run Tests (unit tests for planner + config)

    cd linux
    python3 -m pytest ../tests -v

*(Tests to be added in a follow-up commit.)*

---

## Configuration

### `config/plc_ros_bridge.yaml` — Signal mapping

    latency_probe: false        # enable per-tick CSV timing probe

    plc:
      driver: tcp_json               # opcua | tcp_json | plc_api
      endpoint: opc.tcp://192.168.100.50:4840   # opcua only
      host: 192.168.100.18                      # tcp_json only
      port: 5000                                # tcp_json only
      timeout: 1.5                              # all drivers — see note below

    poll_hz: 60
    watchdog_timeout: 4.0

    logging:
      level: INFO
      console: true
      to_ros: false              # do NOT duplicate to ROS logger
      file: logs/bridge.log
      json_file: logs/bridge.jsonl
      max_bytes: 10485760
      backup_count: 5

    vars:
      ns: 'ns=3;s="MAIN_KUKA_KR210_L150_L"'
      db: 'MAIN_KUKA_KR210_L150_L'

    signals:
      cmd_ang: {addr: 'MainRobotLeft_CmdAng', type: float, dir: read}
      act_ang: {addr: 'MainRobotLeft_ActAng', type: float, dir: write}
      estop:   {addr: 'MainRobotLeft_Emergency_Stop_Active', type: bool, dir: read}

    publish:
      - {topic: /plc/emergency_stop, signal: estop, type: bool}

    heartbeat:
      counter: hb_counter
      ack: ros_ack_counter

    robot:
      joints:  [joint_1, joint_2, joint_3, joint_4]
      cmd_ang: [cmd_ang]     # one array signal (all joints)
      act_ang: [act_ang]
      cmd_ack: cmd_ack

**Why `timeout: 1.5`?** The PLC server occasionally has a 200-1000 ms
cold-start stall (new TCP connection, first read after reconnect). A
timeout under ~200 ms fires the reconnect cascade on every such stall,
publishes a spurious e-stop, and requires a 1 s backoff. 1.5 s absorbs
all observed stalls in a single read attempt while still surfacing a
genuine link failure within two poll periods.

**Why `to_ros: false`?** When both the console/file handler and the ROS
logger are attached, every record appears twice on the terminal. The
JSONL file retains full structure; the ROS echo is redundant.

**Why `watchdog_timeout: 4.0`?** The bridge treats a gap of >4 s between
successful ticks as a liveness failure and publishes `/plc/emergency_stop`.
At 60 Hz with 200-400 ms server stalls, 4 s is the first threshold that
does not produce false positives.

If a signal referenced in the `robot`, `publish`, or `heartbeat` sections is
missing from `signals`, the bridge refuses to start.

If `/plc/emergency_stop` is published from a PLC signal (as above), the
bridge takes the e-stop state from the PLC after a reconnect. Without that
signal it clears the e-stop on reconnect.

**Important:** `/plc/cmd_ang` is republished on **every** tick, even when
the joint targets have not changed. `trajectory_twin.py` uses message
recency as a liveness signal (0.5 s watchdog). Deduplicating unchanged
`cmd_ang` messages causes the twin to stop publishing setpoints and the
robot silently stops moving. Only the **state** topics
(`/plc/velocity`, `/plc/axis_enable`, `/plc/emergency_stop`,
`/plc/ros_alive`, `/plc/comm_state`) are deduped — they are latched
state and rarely change.

### `config/robot.yaml` — Robot description

Used by `trajectory_twin.py` (and the Isaac physics twin): joints, limits
(`limits_deg`), Ruckig limits (`ruckig`), drive gains, and Isaac prim paths.
`all_joints` includes the passive joint (`joint_5`); `controlled_joints` are
the axes commanded by the PLC.

### Terminal output

`trajectory_twin.py` prints a column-aligned table once per second:

    joint           cmd_deg    actual_deg     setpt_deg
    --------------------------------------------------
    joint_1           33.000        33.004        33.002
    joint_2          -60.000       -59.981       -59.987
    joint_3           66.000        66.006        66.003
    joint_4            0.000        -0.000        -0.000

`cmd_deg` comes from `/plc/cmd_ang`, `actual_deg` from `/joint_states`
(feedback from the Isaac physics twin), and `setpt_deg` is the Ruckig
output going to `/servo_setpoints`. All angles in degrees for readability;
the published `JointState` messages still use radians, per ROS convention.

`plc_ros_bridge.py` prints a `tick_stats` line every ~10 s:

    tick_stats | ticks=600 slow=1 slow_pct=0.17 avg_slow_sock_ms=324.13 avg_slow_gap_ms=0.0

- `slow_pct` should stay under 0.5% on a healthy host.
- `avg_slow_gap_ms > 0` indicates GIL contention (bridge-side).
- `avg_slow_sock_ms > 0` indicates real server-side stalls (not bridge-side).

---

## Results

### Tracking Accuracy

Measured on KUKA KR210 through `tcp_json`:

| Joint       | Setpoint  | Actual    | Error   |
| :---------- | :-------- | :-------- | :------ |
| joint_1     | 30.0002°  | 30.0002°  | 0.0001° |
| joint_2     | -29.9603° | -29.9349° | 0.0254° |
| joint_3     | 60.0097°  | 60.0133°  | 0.0036° |
| joint_4     | -0.0025°  | -0.0033°  | 0.0008° |
| joint_5     | -0.4668°  | -0.4668°  | 0.0000° |

Maximum error: **0.0254°** on `joint_2` under full gravity load.

*Note: `joint_5` is passive. Its setpoint is taken from the measured position, so its 0.0000° error is zero by construction and is not a tracking result.*

### Control-Loop Latency

Measured with `tools/latency_full_loop.py` (L2 + L3, after TCP-JSON optimization).

#### Summary (3 independent runs)

| Stage                       | Mean         | p95        | p99        | Max        |
| :-------------------------- | :----------: | :--------: | :--------: | :--------: |
| L1 — OPC UA/TCP read        | ~8 ms        | ~12 ms     | ~20 ms     | ~25 ms     |
| L2 — ROS + Ruckig           | ~6.5 ms      | ~12 ms     | ~24 ms     | ~24 ms     |
| L3 — Isaac physics          | ~3–13 ms     | ~16 ms     | ~17 ms     | ~17 ms     |
| L4 — PLC write              | ~4.5 ms      | ~6 ms      | ~15 ms     | ~16 ms     |
| **Full loop (L1+L2+L3+L4)** | **~15–25 ms**| **~40 ms** | **~55 ms** | **~80 ms** |

#### Comparison across transports

| Transport              | Mean       | Max        | Notes                           |
| :--------------------- | :--------: | :--------: | :------------------------------ |
| OPC UA (baseline)      | 11.5 ms    | 100 ms     | Original — tag-by-tag reads     |
| TCP-JSON (baseline)    | 12.5 ms    | 233 ms     | Original — Windows GIL noise    |
| TCP-JSON optimized     | 5.2 ms     | 16.3 ms    | Batch + msgpack + TCP_NODELAY   |

#### Latency Bound Probability (LBP)

| Threshold | LBP      |
| :-------- | :------: |
| <= 5 ms   | 46.9%    |
| <= 10 ms  | 92.9%    |
| <= 15 ms  | 98.3%    |
| <= 20 ms  | 100%     |
| <= 50 ms  | 100%     |

**Result:** 100% of full-loop samples stay under 20 ms. This is well within the control cycle for medium-speed industrial applications.

See `docs/latency_comparison.md` for the full report.

### Bridge Real-Time Behaviour

Measured with the `tick_stats` diagnostic (60 Hz, 10 s window, `tcp_json` transport, TSC clocksource).

| Metric                              | Value     | Notes                                  |
| :---------------------------------- | :-------- | :------------------------------------- |
| Effective tick rate                 | 60.0 Hz   | `ticks` increments by exactly 600/10 s |
| Slow reads (>200 ms)                | 0–1 per 10 s | one per ~10 s, then zero              |
| `slow_pct`                          | 0.00–0.17% | stabilizes under 0.1% after warm-up   |
| Median read (`sock_ms`)             | 0.2 ms    | measured against the same server      |
| Slow-read `gap_ms` (GIL wait)       | **0.0**   | after `SingleThreadedExecutor` fix     |
| Slow-read `sock_ms` (socket wait)   | 219–406 ms | server-side cold-start stall           |

**Interpretation:** after the GIL-contention fixes, the bridge sustains a genuine 60 Hz loop. The residual ~0.05% `slow_read` warnings are **server-side stalls**, not bridge-side timing artifacts. They are tolerated by `timeout: 1.5` and `watchdog_timeout: 4.0` and do not cause spurious e-stops or setpoint interruptions.

For comparison, before the fixes:

| Metric                              | Before    | After     |
| :---------------------------------- | :-------- | :-------- |
| Slow reads (>200 ms)                | ~5%       | <0.2%     |
| Slow-read `gap_ms` (GIL wait)       | 200–1000 ms | 0.0 ms  |
| Per-tick CPU (1 thread)             | ~3.3 ms   | <1 ms     |
| Spurious e-stops from timeouts      | frequent  | none      |
| Duplicate terminal lines            | yes (×2)  | no        |

### Clock Drift Calibration

Two independent measurements were performed to characterize the clock behavior between Windows and Linux. The scope is **offset and drift between the two endpoints**, not the intrinsic oscillator accuracy of either machine.

Both endpoints use the same clock kind (`perf_counter_ns`), so the comparison is homogeneous — no mixing of `raw` vs `mono` or `wall` vs `monotonic`.

#### Physical machines (Windows 10 ↔ Ubuntu 24.04)

**Setup:**

| Item              | Value                                |
| :---------------- | :----------------------------------- |
| Client            | Windows 10, `DESKTOP-7RFMLG3`        |
| Reference         | Ubuntu 24.04, `Nebras`, `tsc`        |
| Peer address      | `111.111.111.2:9999`                 |
| Duration          | 299.98 s (~5 minutes)                |
| Samples           | 97,471 pings                         |

**Main result:**

| Metric                | Value             |
| :-------------------- | :---------------- |
| Packet loss           | 0.00 % (0/97,471) |
| Drift (least squares) | **+95.73 ppm**    |
| Drift (Theil-Sen)     | +95.81 ppm        |
| Estimator agreement   | 0.082 ppm         |
| Drift stability       | 1.99 ppm range    |
| Quality               | ok (no warnings)  |

**Interpretation:**

- Linux clock runs **~95.7 ppm faster** than Windows
- = **~5.74 ms per minute**
- = **~0.344 s per hour**
- = **~8.26 s per day**

**Link quality:**

| Metric  | Value     |
| :------ | :-------- |
| Min RTT | 0.166 ms  |
| Median  | 0.598 ms  |
| p95     | 0.848 ms  |
| Max     | 1.436 ms  |

The hard floor on offset error is `min_rtt / 2` = **±0.083 ms**.

**Allan deviation:**

| τ (s) | adev (ppm) | n   |
| :---: | :--------: | :-- |
| 1     | 58.22      | 298 |
| 2     | 31.63      | 296 |
| 4     | 15.17      | 292 |
| 8     | 7.69       | 284 |
| 16    | 4.12       | 268 |
| 32    | 1.85       | 236 |
| 64    | 0.92       | 172 |
| 128   | 0.52       | 44  |

Curve falls as `1/√τ` → white-noise behavior; clock is stable.

**Correction model:**

    offset(t) ≈ offset_at_t0 + drift × (t − t0)
    offset(t) ≈ 8,785,337.617 + 0.095728 × (t − t0)   [ms, t in seconds]

    local_time = peer_time − offset(t)

**Achievable correction accuracy:** **±0.1 ms** over 5 minutes.

#### QEMU/KVM guest (Windows guest ↔ Ubuntu host)

**Setup:**

| Item              | Value                                |
| :---------------- | :----------------------------------- |
| Host              | Ubuntu 24.04, TSC clocksource        |
| Guest             | Windows 10 in QEMU/KVM               |
| Duration          | 756.7 s                              |
| Samples           | 31,851 pings                         |

**Main result:**

| Metric              | Value             |
| :------------------ | :---------------- |
| Mean drift          | **≈ 80 ppm**      |
| Min RTT             | 1.2 ms            |
| Median RTT          | 2.0–2.5 ms        |
| Max RTT             | 4–5 ms            |
| Drift stability     | unstable (>20 ppm range) |

**Interpretation:**

This number is the *practical* timing error of a Windows guest in this VM configuration. It includes:

- host TSC stability (good)
- QEMU emulation overhead (dominant)
- vCPU scheduling jitter
- actual Windows clock drift

It does **NOT** measure the intrinsic oscillator drift of the Windows machine's hardware.

#### Comparison of both setups

| Metric           | QEMU/KVM guest     | Physical machines       | Improvement |
| :--------------- | :----------------: | :---------------------: | :---------: |
| Min RTT          | 1.2 ms             | **0.166 ms**            | **7.2×**    |
| Median RTT       | 2.0–2.5 ms         | 0.598 ms                | ~4×         |
| Fitted drift     | ~80 ppm (variable) | **95.73 ppm (const.)**  | stable now  |
| Drift range      | > 20 ppm           | **1.99 ppm**            | **> 10×**   |
| Fit residual std | large              | 0.030 ms                | —           |
| Quality          | check notes        | **ok**                  | OK          |

#### Conclusions

- The two physical machines drift at a constant rate of **+95.73 ppm**.
- The Linux clock is consistently **ahead** of the Windows clock.
- The link between physical machines is clean: **0% loss**, min RTT = **0.166 ms**.
- The drift is stable (range **1.99 ppm** over 5 minutes).
- A single linear correction is sufficient for sessions up to ~10 min.
- For longer sessions, use `--track` and interpolate.
- Achievable correction accuracy on physical machines: **±0.1 ms**.
- For QEMU/KVM guests, the emulation overhead dominates — use physical machines for pure oscillator measurement.

#### Recommendations

1. Use the linear model in the physical-machines section for sessions ≤ 10 minutes.
2. Switch to `--track` for sessions > 1 hour.
3. Keep NTP/chrony disabled on both machines for pure measurement.
4. Repeat the run 3 times at different times of day to confirm.
5. Store the calibration coefficients with each experiment.
6. For QEMU/KVM: enable `hypervclock`, disable `hpet`, pin vCPUs, then re-run.

---

## Performance Tuning

The platform can be tuned for different latency/robustness trade-offs.

### Quick Wins (no code changes)

| Tuning             | Command                                        | Expected gain |
| :----------------- | :--------------------------------------------- | :-----------: |
| CPU pinning        | `taskset -c 2,3 python3 trajectory_twin.py`      | -2 to -3 ms   |
| Real-time priority | `sudo chrt -f 80 python3 plc_ros_bridge.py`    | -2 ms         |
| CycloneDDS         | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | -1 ms         |

### Bridge-Side Tuning (built in)

These are already enabled by the current code and matter more than any external knob at 60 Hz:

| Tuning | Effect |
| :----- | :----- |
| `SingleThreadedExecutor` | Eliminates GIL contention between timer and subscriber callbacks (was 200-1000 ms stalls) |
| State-topic publish dedupe | Cuts DDS traffic from ~300 msg/s to <10 msg/s at steady state |
| `TCP_NODELAY` on tcp_json socket | Prevents Nagle/delayed-ACK stalls (defensive) |
| Monotonic timing | Removes NTP/hypervisor clock-jump artifacts from `slow_read` diagnostics |
| `tick_stats` every 10 s | Separates `gap_ms` (bridge/GIL) from `sock_ms` (server) automatically |

**If you still see `slow_read` after these:**
- `gap_ms > 0` → bridge-side, check host CPU load (`top -H`, `taskset`)
- `sock_ms > 0` → server-side, see `plc_server_win.py` — usually a ~10 s periodic task holding a lock or doing a slow fsync

### Transport Tuning

**TCP-JSON:**

- Use msgpack framing (not JSON) — 3x faster serialization
- Set TCP_NODELAY on both client and server sockets
- Use asyncio on the Windows server (not threading)
- Batch all reads/writes into a single request

**OPC UA:**

- Use `read_values(nodes)` instead of per-node `read_value()`
- Consider OPC UA Subscriptions for change-based updates
- Set TCP_NODELAY on the OPC UA client socket

### Expected Performance

| Transport             | Mean      | p99        | Max        |
| :-------------------- | :-------: | :--------: | :--------: |
| OPC UA (naive)        | 11.5 ms   | 14 ms      | 100 ms     |
| OPC UA (batch)        | ~5 ms     | ~10 ms     | ~20 ms     |
| TCP-JSON (naive)      | 12.5 ms   | 20 ms      | 233 ms     |
| TCP-JSON (optimized)  | 5.2 ms    | 15.5 ms    | 16.3 ms    |

---

## Documentation

| Document                            | Description                           |
| :---------------------------------- | :------------------------------------ |
| `docs/README_full.md`               | Gen 3 full documentation              |
| `docs/migration.md`                 | How this folder was split from v2.0.0 |
| `docs/latency_comparison.md`        | Full latency comparison report        |
| `docs/latency_full_loop_OPC-UA.md`  | OPC UA baseline report                |
| `../CHANGELOG.md`                   | Version history (all generations)     |

---

## Lineage

| Version | Robot         | Transport         | Architecture      | Location                |
| :------ | :------------ | :---------------- | :---------------- | :---------------------- |
| v0.1.0  | KUKA KR210    | TCP/JSON          | Monolithic        | archived                |
| v1.0.0  | KUKA KR210    | TCP/JSON          | 5 processes       | `archive/v1.0-kuka/`    |
| v0.2.0  | Yaskawa GP110 | OPC UA            | Monolithic bridge | `archive/v0.2-yaskawa/` |
| v2.0.0  | Yaskawa GP110 | OPC UA            | Modular (6 files) | `archive/v0.2-yaskawa/` |
| v3.0.0  | Any robot     | Any transport     | Vendor-neutral    | `vendor-neutral/`       |

---

## Roadmap

### Completed

- [x] Extract PLC transports into `plc_drivers.py`
- [x] Extract signal handling into `plc_config.py`
- [x] All three transports selectable from one YAML line
- [x] Repository reorganized by environment (windows/linux/isaac/plc/tools)
- [x] Latency + drift measurement tools preserved
- [x] TCP-JSON optimization: batch reads + msgpack + TCP_NODELAY
  - Mean latency: 12.5 ms -> 5.2 ms (-58%)
  - Max latency: 233 ms -> 16.3 ms (-93%)
  - Full-loop p99 under 20 ms in 100% of samples
- [x] Latency characterization across OPC UA and TCP-JSON
- [x] `trajectory_twin.py` reads joints, limits, and Ruckig parameters from `config/robot.yaml`
- [x] Fix: a persistent bad command no longer cancels an in-progress brake
- [x] Fix: bridge no longer auto-clears the e-stop on PLC reconnect
- [x] Clock drift characterization (physical + QEMU/KVM)
- [x] **Bridge reaches clean 60 Hz**
  - `SingleThreadedExecutor` eliminates GIL contention
  - State-topic publish dedupe (~300 msg/s → <10 msg/s)
  - Monotonic timing in all latency measurements
  - `tick_stats` diagnostic splits `gap_ms` (GIL) from `sock_ms` (server)
  - `TCP_NODELAY` set on `tcp_json` sockets
  - `slow_pct` <0.2% sustained; residual stalls are server-side (~0.05%)
- [x] **`trajectory_twin.py` terminal table**
  - Side-by-side `cmd_deg` / `actual_deg` / `setpt_deg`, once per second
  - File/JSONL log keeps compact single-line record with `position_deg`
- [x] **Eliminate duplicate terminal lines** — `logging.to_ros: false`
- [x] **Fix liveness semantics** — `/plc/cmd_ang` always republished;
      only state topics are deduped

### In Progress

- [ ] Unit tests for `plc_config` and `plc_drivers`
- [ ] Yaskawa GP110 verification on this branch
- [ ] Eliminate residual server-side stalls (~400 ms every ~10 s) —
      needs `plc_server_win.py` instrumentation

### Planned

- [ ] Cartesian motion layer (inverse kinematics)
- [ ] Trajectory queue and blending
- [ ] Certified safety layer (F-CPU + PROFIsafe) — postponed; see Safety Notice
- [ ] Native OPC UA on Windows (remove PLC server dependency)

---

## License

MIT License. See [LICENSE](LICENSE).

---

## Contact

- **YouTube:** [Digital Twin Series](https://www.youtube.com/playlist?list=PLYLnoPt4fbu8)
- **Developer:** Nebras — nebras4u@gmail.com
- **Repository:** [isaac-plc-digital-twin](https://github.com/Nebras4u/isaac-plc-digital-twin)

---

*Built for Industry 4.0 — isaac-plc-digital-twin © 2026*
