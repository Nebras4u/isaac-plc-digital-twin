# Isaac Digital Twin — Multi-Generation Platform

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23312010.svg)](https://doi.org/10.5281/zenodo.23312010)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen)]()
[![Tests](https://img.shields.io/badge/Tests-51%20pytest%20%2B%208%20standalone-success)](#testing)
[![Reference mean latency](https://img.shields.io/badge/Reference%20mean%20latency-5.2%20ms-success)](#latency-performance)
[![Reference max latency](https://img.shields.io/badge/Reference%20max%20latency-16.3%20ms-success)](#latency-performance)
[![VM post-v3.7](https://img.shields.io/badge/VM%20post--v3.7%20p50-14.48%20ms-blueviolet)](#results-this-machine)

An industrial-grade **Digital Twin platform** for robot arms driven by
**Siemens S7-1500** PLCs, synchronized with **NVIDIA Isaac Sim** through
**ROS 2**.

![S7-1500T ↔ ROS2 ↔ Isaac Sim](https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/yaskawa/docs/images/PLC-ROS2-ISAAC.jpg)

The repository hosts three generations of the platform, side by side:

| Generation                 | Robot               | Transport                  | Location                                         | Status       |
| :------------------------- | :------------------ | :------------------------- | :----------------------------------------------- | :----------- |
| **Gen 3 — Vendor-Neutral** | Any (config-driven) | tcp_json / opcua / plc_api | [`vendor-neutral/`](vendor-neutral/)             | **Active**   |
| Gen 2 — Yaskawa GP110      | Yaskawa GP110       | OPC UA                     | [`yaskawa/`](yaskawa/)                           | Reference    |
| Gen 1 — KUKA KR210         | KUKA KR210 L150 L   | TCP/JSON                   | [`kuka/`](kuka/)                                 | Reference    |

---

## Safety Notice

> **This project is a simulation / digital-twin research platform. It is
> NOT a certified safety function and must not be used to protect people
> or physical machinery.**

- The e-stop, heartbeat, and watchdog logic in this repository (bridge,
  trajectory twin, PLC counters) is **functional monitoring** that stops
  the *simulated* robot. It is not designed, validated, or certified
  according to IEC 61508, ISO 13849-1, or IEC 62061.
- Communication over `tcp_json`, OPC UA, or ROS 2 has no safety-rated
  integrity (no black channel, no safety protocol).
- Any real installation needs an independent, certified safety layer
  (for example an F-CPU with PROFIsafe and hardware E-Stop circuits).
  This is on the roadmap and deliberately postponed for the current
  simulation-only stage.

---

## Environment

> **The latency numbers in this README depend on the environment.**
> Always re-measure on your own hardware before drawing conclusions.

The reference numbers (5.2 ms mean, 16.3 ms max) were measured on the
**author's reference machine**:

| Factor         | Reference machine                   |
| :------------- | :---------------------------------- |
| OS             | Native Ubuntu 24.04                 |
| CPU            | Dedicated cores (no throttling)     |
| GPU            | NVIDIA, no passthrough              |
| PLC server     | Native Windows, low-latency         |
| Isaac Sim      | Native, 60 FPS stable               |
| ROS 2          | Jazzy                               |
| RMW            | FastDDS                             |
| `tcp_json`     | msgpack + `TCP_NODELAY` + batch     |

> **Note on v3.7:** The reference numbers above were measured on the
> reference machine with batch enabled at **60 Hz** (v3.1.2 and
> earlier). v3.5–v3.7 added probe observability, trajectory fixes,
> and `poll_hz: 200`. Those changes were measured on the **Docker/VM**,
> **not** on the reference machine. The reference machine has **not
> been re-measured** for v3.7. See
> [`vendor-neutral/README.md`](vendor-neutral/README.md) for the VM
> comparison.

The **same code** on a **Docker / VM environment** (RTX 5060, shared
vCPUs, PLCSIM Advanced over TCP/JSON) gives different numbers — see
[Results (this machine)](#results-this-machine) below. This is expected
and does not indicate a regression.

---

## Active Development: `vendor-neutral/`

The current generation removes hard-coded coupling to any specific robot
or PLC vendor. **Switch robots and PLCs by editing YAML — not by editing
Python.**

Highlights:

- 3 transports in one abstraction: `tcp_json`, `opcua`, `plc_api`
- **Batch PLC I/O** (`ReadSignals`/`WriteSignals` — one API call per
  request instead of per-tag; v3.1.3)
- Robot description in `config/robot.yaml`
- Signal mapping in `config/plc_ros_bridge.yaml`
- Ruckig-based online trajectory generation (`trajectory_twin.py`,
  configured from `config/robot.yaml`)
- **Gravity is kept enabled** (real physics). Compensation is achieved
  implicitly via high drive stiffness in `config/robot.yaml` — not by
  disabling gravity or weightlessness.
- Controlled braking on e-stop, heartbeat/feedback loss, or an
  out-of-limits command (never an abrupt setpoint jump)
- Counter-based heartbeat protocol
- Sub-0.03° tracking accuracy verified under full gravity load
- **Full-loop latency: 5.2 ms mean, 16.3 ms max** (TCP-JSON optimized,
  reference machine — see [Environment](#environment))
- **58% faster** than the naive TCP-JSON baseline on the same machine
  (mean 12.5 ms → 5.2 ms)
- **51 pytest tests** covering the config schema, migration, E-stop
  forwarding, Ruckig parameter validation, `JointMapper`, and trajectory
  state machine
- **8 standalone probe tests** covering the latency-tag chain
  (`ros2/test_probe_chain.py`)
- **Config schema v2.1** (Pydantic v2, fail-fast, audit trail)
- **Offline latency analysis** tool: `ros2/analyze_probe.py`
  (with `--plot`, `--poll-hz`, `--export-clean`)

**See [`vendor-neutral/README.md`](vendor-neutral/README.md) for full docs.**

---

## Recent Changes

See [`CHANGELOG.md`](CHANGELOG.md) for the full list. Latest entry:
**v3.7.0 — Probe Observability + Production Path** (2026-10-28).

**v3.7.0 — Probe Observability + Production Path:**

- **`plc_ros_bridge.py` — `probe_attempted` counter (new).**
  Distinguishes "bridge didn't publish a tag" (`probe_sent == 0`) from
  "tag arrived but was rejected" (`probe_attempted > probe_rows`).
  In production, `probe_attempted == probe_rows == 2691` (100% capture).
- **`plc_ros_bridge.py` — `_js_tag` starvation fix.**
  `probe_rows/probe_attempted` rose from 7.5% to **100%**.
- **`plc_ros_bridge.py` — same-tick `act_ang` write.** `write_net`
  p50 −24%.
- **`trajectory_twin.py` — multiple fixes.** Passive axis not
  interpolated; `set_target` called after `reset()`; `RuckigPlanner`
  validates all parameters; tag consumed only by `_publish`.
- **`physics_twin.py` — `DEFAULT_PUBLISH_HZ = 120.0` (was 60.0).**
  `_applied_tag` survives multiple publishes; 7-part `frame_id` in
  >90% of messages.
- **`joint_mapper.py` (new)** — standalone module for Isaac + Linux.
- **`analyze_probe.py`** — new options `--plot`, `--poll-hz`,
  `--export-clean`.
- **Tests:** 51 pytest + 8 standalone (was 29 + 10).
- **`plc_ros_bridge.yaml`: `poll_hz: 120 → 200`.**
- **Impact (Docker/VM, not reference machine, final measurement):** median
  `cmd_to_feedback` 18.55 → **14.62 ms** (−21%); p95 22.62 ms; p99 35.50 ms;
  median `total_loop` 25.72 → **19.21 ms** (−25%); LBP ≤ 20 ms
  59.17% → **91.40%** (+32pp); LBP ≤ 30 ms 97.64%; `write_failed` = 0
  (was persistent); `probe_attempted == probe_rows == 1396` (100% capture).

**v3.6.0 — Latency-Probe Chain Fixes** (2026-10-28):

- **`_js_tag` starvation fix** — tag consumed only on actual `act_ang`
  write.
- **`_cmd_tag` consumed only by `_publish`** (was consumed every tick).
- **`_applied_tag` survives multiple publishes** (was consumed on first
  publish).
- **Same-tick `act_ang` write** — `write_net` p50 −24%.

**v3.5.0 — Trajectory Twin Fixes** (2026-10-28):

- **`RuckigPlanner` validates all parameters** at construction.
- **`needs_target_update` compares against applied** (was received).
- **`_publish` no longer uses `np.interp`** on passive axes.

**v3.1.3 — PLC Batch Mode + 120 Hz Tuning** (2026-10-26):

- **`windows/plc_server_win.py` — batch PLC I/O enabled by default.**
  **Fixed:** `_TYPE_NAMES` maps `LReal→Double`, `DInt→Int32`,
  `Int→Int16` (EPrimitiveDataType uses .NET names, not Siemens names).
- **`config/plc_ros_bridge.yaml`: `poll_hz: 60 → 120`.**
- **`ros2/analyze_probe.py` (new)** — offline CSV analysis.
- **`ros2/test_probe_chain.py` (new)** — 10 standalone tests.
- **Impact (Docker/VM):** median `cmd_to_feedback` 34.60 → **18.55 ms**
  (−46%); median `total_loop` 45.95 → **25.72 ms** (−44%).

**v3.1.2 — Performance Fixes for VM Environments** (2026-10-25):

- **`plc_ros_bridge.py` — PLC I/O moved to a dedicated thread.**
- **`physics_twin.py` — `spin_once` before `step`/`publish_states`.**
- **QoS** — `BEST_EFFORT` + `depth=1` on high-rate streams.

**v3.1.1 — Environment Transparency and Real VM Results** (2026-10-23):

- **README:** added `Environment` section and `Results (this machine)`.
- **Fix:** a persistent out-of-limits command no longer cancels an
  in-progress brake.
- **Fix:** on PLC reconnect, the bridge no longer publishes
  `/plc/emergency_stop = False` before the real PLC e-stop value has been
  read.
- Added the Safety Notice above.

---

## Latency Performance

> **Reference machine only.** All numbers in this section are from the
> **native Ubuntu 24.04** reference machine described in
> [Environment](#environment). They predate v3.1.3. See
> [Results (this machine)](#results-this-machine) for VM numbers, which
> use different metrics and are **not directly comparable**.

Measured across all transports (see
[`vendor-neutral/docs/latency_comparison.md`](vendor-neutral/docs/latency_comparison.md)
for the full report):

| Transport                 | Mean      | p99        | Max        | Full Loop |
| :------------------------ | :-------: | :--------: | :--------: | :-------: |
| OPC UA (naive)            | 11.5 ms   | 14 ms      | 100 ms     | ~20 ms    |
| TCP-JSON (naive)          | 12.5 ms   | 20 ms      | 233 ms     | ~25 ms    |
| **TCP-JSON (optimized)**  | **5.2 ms** | **15.5 ms** | **16.3 ms** | **~15 ms** |

**Optimization techniques applied:**

1. `msgpack` framing instead of JSON — 3× faster serialization
2. `TCP_NODELAY` on both client and server sockets
3. `asyncio` on the Windows server — eliminates GIL contention
4. Single-request batch reads — 15 tags in one packet
5. CPU pinning + real-time priority

**Result:** On the **reference machine** (pre-v3.1.3, 60 Hz, batch
enabled), **100% of full-loop samples stayed under 20 ms** — well within
the control cycle for medium-speed industrial applications.

---

### Results (this machine — v3.7)

Measured on a **different environment** than the reference machine:

| Item         | Value                                  |
| :----------- | :------------------------------------- |
| OS           | Ubuntu 24.04 (Docker)                  |
| CPU          | Shared vCPU (24 cores, `taskset`)      |
| GPU          | RTX 5060, 8 GB                         |
| PLC server   | PLCSIM Advanced V6.0, TCP/JSON         |
| ROS 2        | Jazzy                                  |
| RMW          | FastDDS                                |

**v3.7 full-loop latency** (60 s motion, `cmd_ang: [cmd_ang]`,
n=2691 records):

| Stage          | p50      | p95      | p99      | p99.9    |
| :------------- | -------: | -------: | -------: | -------: |
| `queue_delay`  | 3.08 ms  | 3.53 ms  | 3.61 ms  | 4.08 ms  |
| `read_net`     | 1.72 ms  | 3.90 ms  | 17.11 ms | 46.83 ms |
| `convert_pub`  | 0.05 ms  | 0.08 ms  | 0.11 ms  | 0.14 ms  |
| `ros_ruckig`   | 3.08 ms  | 6.46 ms  | 8.31 ms  | 9.06 ms  |
| `isaac_step`   | 5.15 ms  | 8.39 ms  | 9.50 ms  | 11.05 ms |
| `ros_back`     | 0.29 ms  | 0.44 ms  | 0.65 ms  | 3.12 ms  |
| `write_net`    | 4.73 ms  | 13.42 ms | 40.48 ms | 75.46 ms |
| **`cmd_to_feedback`** | **14.48 ms** | **23.38 ms** | **45.91 ms** | **86.45 ms** |
| **`total_loop`** | **19.04 ms** | **29.25 ms** | **60.88 ms** | **89.61 ms** |

**LBP (Latency Bound Probability) — `cmd_to_feedback`:**

| Threshold | LBP      |
| :-------- | -------: |
| ≤ 10 ms   | 21.96%   |
| ≤ 15 ms   | 60.47%   |
| **≤ 20 ms** | **90.37%** |
| ≤ 30 ms   | 97.29%   |
| ≤ 40 ms   | 98.44%   |
| ≤ 50 ms   | 99.10%   |

**Comparison v3.1.3 → v3.7 (same VM):**

| Stage          | v3.1.3   | **v3.7**     | Δ |
| :------------- | -------: | -----------: | -: |
| `queue_delay`  | 5.87 ms  | **3.08 ms**  | −48% |
| `read_net`     | 1.36 ms  | 1.72 ms      | +26% |
| `ros_ruckig`   | 4.32 ms  | **3.08 ms**  | −29% |
| `isaac_step`   | 9.71 ms  | **5.15 ms**  | −47% |
| `write_net`    | 5.28 ms  | **4.73 ms**  | −10% |
| **`cmd_to_feedback`** | **18.55 ms** | **14.48 ms** | **−22%** |
| **`total_loop`** | **25.72 ms** | **19.04 ms** | **−26%** |
| LBP ≤ 20 ms    | 59.17%   | **90.37%**   | **+31pp** |

**Probe health:** `probe_attempted == probe_rows == 2691`,
`probe_rejected == 0`, `probe_order_violations == 0`,
`write_failed == 0`.

**Interpretation:**

- v3.7 improves every metric over v3.1.3 (median −22% to −26%).
- The remaining `write_net` p99 (~40 ms) is TCP tail latency on the
  libvirt VM, not bridge code.
- The reference machine has **not** been re-measured for v3.7.
  The reference numbers (5.2 ms mean, 16.3 ms max) still refer to
  v3.1.2 and earlier.

---

## Future Vision

The vendor-neutral philosophy is being extended to encompass broader
hardware ecosystems, software runtimes, and the **simulation layer**
itself.

### 1. Unified Control Layer (Multi-PLC Vendor expansion)

- **Beckhoff TwinCAT 3:** Support is being added to integrate
  TwinCAT runtime environments natively as a core component of our
  vendor-neutral PLC architecture. This will allow developers to swap
  seamlessly between Siemens S7-1500 and Beckhoff TwinCAT controllers
  by adjusting runtime mapping files.

### 2. Multi-Simulator Agnostic Framework

- **Isaac Sim** remains a first-class target, but will also be brought
  under the same vendor-neutral abstraction.
- **CoppeliaSim** will be added as a **proof of concept** for
  multi-simulator support.

The long-term goal is a fully portable digital twin: change the robot,
the PLC ecosystem (Siemens/TwinCAT), or the simulator by editing
configuration — not by rewriting the core platform.

---

## Repository Layout

    .
    ├── vendor-neutral/              # Active — Gen 3 (vendor-neutral platform)
    │   ├── README.md                # Full Gen 3 documentation
    │   ├── docs/
    │   ├── linux/
    │   ├── ros2/                    # v3.1.3+: analyze_probe.py, test_probe_chain.py
    │   ├── windows/
    │   ├── isaac/                   # v3.7: joint_mapper.py, physics_twin.py
    │   ├── config/
    │   ├── plc/
    │   └── tools/                   # v3.7: clean_run.sh
    ├── kuka/                        # Gen 1 — KUKA KR210 (TCP/JSON)
    ├── yaskawa/                     # Gen 2 — Yaskawa GP110 (OPC UA)
    ├── docs/                        # Cross-generation documentation
    └── README.md                    # This file

---

## Quick Start (Gen 3)

```bash
cd vendor-neutral

# 1. Windows: TCP/JSON server (only if driver: tcp_json)
cd windows && python plc_server_win.py --host 0.0.0.0 --port 5000

# 2. Isaac Sim Script Editor: run isaac/physics_twin.py

# 3. Linux: PLC bridge (with CPU pinning for latency)
cd linux
taskset -c 8,9 python3 plc_ros_bridge.py \
    --ros-args -p config:=../config/plc_ros_bridge.yaml

# 4. Linux: trajectory twin (Ruckig + state machine)
taskset -c 10 python3 trajectory_twin.py \
    --ros-args -p robot:=../config/robot.yaml
```

Full instructions: [`vendor-neutral/README.md`](vendor-neutral/README.md).

---

## Documentation

| Document | Description |
| :--- | :--- |
| [`vendor-neutral/README.md`](vendor-neutral/README.md) | Gen 3 (active) — vendor-neutral platform |
| [`vendor-neutral/docs/latency_comparison.md`](vendor-neutral/docs/latency_comparison.md) | Full latency comparison across transports |
| [`kuka/README.md`](kuka/README.md) | Gen 1 — KUKA KR210 |
| [`yaskawa/docs/README.md`](yaskawa/docs/README.md) | Gen 2 — Yaskawa GP110 |
| [`CHANGELOG.md`](CHANGELOG.md) | All generations |
| [`docs/comparison.md`](docs/comparison.md) | Side-by-side comparison |

---

## How to Cite

If you use this software in academic work, please cite:

```bibtex
@software{damsarkho2026isaac,
  author       = {Damsarkho, Nebras},
  title        = {{Isaac Digital Twin — Vendor-Neutral Platform for
                   PLC, ROS 2, and Isaac Sim}},
  month        = oct,
  year         = 2026,
  publisher    = {Zenodo},
  version      = {3.7.0},
  doi          = {10.5281/zenodo.23312011},
  url          = {https://doi.org/10.5281/zenodo.23312011}
}
```

Or in plain text:

> Damsarkho, N. (2026). *Isaac Digital Twin — Vendor-Neutral Platform
> for PLC, ROS 2, and Isaac Sim* (Version 3.7.0) [Computer software].
> Zenodo. https://doi.org/10.5281/zenodo.23312011

---

## Contact

- **YouTube:** [Digital Twin Series](https://www.youtube.com/playlist?list=PLYLnoPt4fbu8)
- **Developer:** Nebras — nebras4u@gmail.com
- **Repository:** [isaac-plc-digital-twin](https://github.com/Nebras4u/isaac-plc-digital-twin)

---

*Built for Industry 4.0 — isaac-plc-digital-twin © 2026*
