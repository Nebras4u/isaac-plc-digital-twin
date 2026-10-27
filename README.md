# Isaac Digital Twin — Multi-Generation Platform

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23312010.svg)](https://doi.org/10.5281/zenodo.23312010)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen)]()
[![Reference mean latency](https://img.shields.io/badge/Reference%20mean%20latency-5.2%20ms-success)](#latency-performance)
[![Reference max latency](https://img.shields.io/badge/Reference%20max%20latency-16.3%20ms-success)](#latency-performance)

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

The **same code** on a **Docker / VM environment** (RTX 5060, shared
vCPUs, PLCSIM Advanced over TCP/JSON) gives different numbers — see
[Results (this machine)](#results-this-machine) below. This is expected
and does not indicate a regression.

**How to measure on your machine:**

1. Set `latency_probe: true` in `vendor-neutral/config/plc_ros_bridge.yaml`.
2. Run the bridge for ≥ 30 s.
3. Analyze `latency_probe.csv` with
   `vendor-neutral/tools/latency_full_loop.py`, or use the inline snippet
   in [Results (this machine)](#results-this-machine).

---

## Active Development: `vendor-neutral/`

The current generation removes hard-coded coupling to any specific robot
or PLC vendor. **Switch robots and PLCs by editing YAML — not by editing
Python.**

Highlights:

- 3 transports in one abstraction: `tcp_json`, `opcua`, `plc_api`
- Robot description in `config/robot.yaml`
- Signal mapping in `config/plc_ros_bridge.yaml`
- Ruckig-based online trajectory generation (`trajectory_twin.py`,
  configured from `config/robot.yaml`)
- Controlled braking on e-stop, heartbeat/feedback loss, or an
  out-of-limits command (never an abrupt setpoint jump)
- Counter-based heartbeat protocol
- Sub-0.03° tracking accuracy verified
- **Full-loop latency: 5.2 ms mean, 16.3 ms max** (TCP-JSON optimized,
  reference machine — see [Environment](#environment))
- **58% faster** than the naive TCP-JSON baseline on the same machine
  (mean 12.5 ms → 5.2 ms)
- **29 unit tests** covering the config schema, migration, E-stop
  forwarding, and Ruckig parameter validation
- **Config schema v2.1** (Pydantic v2, fail-fast, audit trail)

**See [`vendor-neutral/README.md`](vendor-neutral/README.md) for full docs.**

---

## Recent Changes

See [`CHANGELOG.md`](CHANGELOG.md) for the full list. Latest entry:
**v3.1.3 — PLC Batch Mode + 120 Hz Tuning** (2026-10-27).

**v3.1.3 — PLC Batch Mode + 120 Hz Tuning:**

- **`plc_ros_bridge.py` — batch mode for PLC I/O.** PLC reads/writes
  are grouped per control cycle to reduce round-trips and jitter on
  slow transports.
- **120 Hz tuning.** Control-loop parameters re-tuned for 120 Hz
  operation; re-measure latency on your own hardware before comparing
  against the reference numbers.
- *(Align the detailed bullets above with the `CHANGELOG.md` v3.1.3
  entry — this README will reflect the full note once the CHANGELOG is
  updated.)*

**v3.1.2 — Performance Fixes for VM Environments** (2026-10-25):

- **`plc_ros_bridge.py` — PLC I/O moved to a dedicated thread.**
  `tick()` (PLC I/O) and `on_joint_states()` (ROS callback) no longer
  share the `SingleThreadedExecutor`. Result: `ros_back` −92%,
  `total_loop` −30%.
- **`plc_ros_bridge.py` — `_last_tick_end_ns` for real queue delay.**
  `queue_delay` is now measurable (was 0.00 ms).
- **`physics_twin.py` — `spin_once` before `step`/`publish_states`.**
  A setpoint that arrived since the last frame is applied in the same
  frame. Result: `isaac_step` −62%.
- **`trajectory_twin.py` — `SingleThreadedExecutor` (was `MultiThreaded`).**
  Intended to remove executor overhead on `ros_ruckig`. The measured
  effect against v3.1.1 is still unexplained — see
  [Interpretation](#results-this-machine) and `VL-16` in the CHANGELOG.
- **QoS** — `BEST_EFFORT` + `depth=1` on high-rate streams. Old samples
  are dropped instead of queued.

**v3.1.1 — Environment Transparency and Real VM Results** (2026-10-23):

- **README:** added `Environment` section and `Results (this machine)`
  with real Docker / RTX 5060 numbers.
- **README:** documented the `Articulation.initialize()` hang on Isaac
  Sim 5.1 and the ROS 2 Bridge environment (Python 3.11 vs 3.12).
- **README:** documented the `latency_probe` `origin_ns` fix and the
  CycloneDDS caveat.
- **Badges:** `Tests 26 → 29`, `Schema 2.0 → 2.1`, latency badges
  renamed to "Reference ..." to avoid implying universal numbers.
- `krc_twin.py` is replaced by the vendor-neutral `trajectory_twin.py`.
- **Fix:** a persistent out-of-limits command no longer cancels an
  in-progress brake.
- **Fix:** on PLC reconnect, the bridge no longer publishes
  `/plc/emergency_stop = False` before the real PLC e-stop value has been
  read.
- Added the Safety Notice above.

---

## Latency Performance

> **Reference machine only.** See [Environment](#environment) and
> [Results (this machine)](#results-this-machine) for the VM numbers.

> **Note:** the latency tables in this section and the next one were
> measured on **v3.1.2**. Re-measurement on v3.1.3 (batch mode + 120 Hz
> tuning) is pending.

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

**Result:** 100% of full-loop samples stay under 20 ms on the
**reference machine** — well within the control cycle for medium-speed
industrial applications.

---

### Results (this machine)

Measured on a **different environment** than the reference machine:

| Item         | Value                                  |
| :----------- | :------------------------------------- |
| OS           | Ubuntu 24.04 (Docker)                  |
| CPU          | Shared vCPU                            |
| GPU          | RTX 5060, 8 GB                         |
| PLC server   | PLCSIM Advanced V6.0, TCP/JSON         |
| ROS 2        | Jazzy                                  |
| RMW          | FastDDS                                |

**Full-loop latency** (1,881 samples, `latency_probe: true`, v3.1.2):

| Stage          | Median   | p95      | p99       | Max        |
| :------------- | -------: | -------: | --------: | ---------: |
| `queue_delay`  | 5.42 ms  | 7.41 ms  | 8.04 ms   | 10.30 ms   |
| `read_net`     | 6.73 ms  | 13.08 ms | 46.32 ms  | 118.33 ms  |
| `convert_pub`  | 0.05 ms  | 0.08 ms  | 0.10 ms   | 0.16 ms    |
| `ros_ruckig`   | 8.14 ms  | 21.81 ms | 75.53 ms  | 502.71 ms  |
| `isaac_step`   | 10.25 ms | 26.11 ms | 30.79 ms  | 103575.93 ms |
| `ros_back`     | 0.37 ms  | 1.83 ms  | 2.64 ms   | 3.33 ms    |
| `write_net`    | 12.15 ms | 20.61 ms | 36.79 ms  | 132.64 ms  |
| **`total_loop`** | **47.20 ms** | **67.28 ms** | **151.49 ms** | **104091.64 ms** |

**LBP (Latency Bound Probability) — this machine (v3.1.2):**

| Threshold | LBP    |
| :-------- | -----: |
| <= 20 ms  | 0.00%  |
| <= 30 ms  | 5.10%  |
| <= 40 ms  | 37.59% |
| <= 50 ms  | 63.37% |
| <= 80 ms  | 96.60% |
| <= 100 ms | 97.24% |
| <= 200 ms | 99.89% |

**Filtered (remove top 1% outliers):**

| Metric | Value     |
| :----- | --------: |
| n      | 1,863     |
| median | 47.01 ms  |
| p95    | 64.51 ms  |
| max    | 151.49 ms |

**Comparison: before vs after fixes (v3.1.1 → v3.1.2):**

| Stage          | Before (v3.1.1) | After (v3.1.2) | Improvement |
| :------------- | --------------: | -------------: | ----------: |
| `queue_delay`  | 4.67 ms         | **5.42 ms**    | (measurable now) |
| `read_net`     | 6.88 ms         | **6.73 ms**    | −2%         |
| `convert_pub`  | 0.06 ms         | **0.05 ms**    | −17%        |
| `ros_ruckig`   | 6.57 ms         | **8.14 ms**    | +24% (unexplained, see below) |
| `isaac_step`   | 27.05 ms        | **10.25 ms**   | **−62%** ✅ |
| `ros_back`     | 4.66 ms         | **0.37 ms**    | **−92%** ✅ |
| `write_net`    | 15.72 ms        | **12.15 ms**   | −23%        |
| **`total_loop`** | **67.36 ms**  | **47.20 ms**   | **−30%** ✅ |
| LBP ≤ 50 ms    | 6.78%           | **63.37%**     | **+56.59%** ✅ |
| LBP ≤ 80 ms    | 75.45%          | **96.60%**     | **+21.15%** ✅ |

**Interpretation:**

- The VM is still **~3× slower** than the reference machine (47 ms vs ~15 ms).
- The dominant remaining contributors are `write_net` (~12 ms) and
  `isaac_step` (~10 ms). Both are tied to the **QEMU/KVM environment**:
  PLCSIM Advanced API on Windows VM and Isaac Sim physics step on shared
  vCPU.
- `convert_pub` (0.05 ms) and `queue_delay` (5.42 ms) are close to the
  reference machine — the bridge-side code is **not** the bottleneck.
- `ros_ruckig` went **up** (6.57 → 8.14 ms) between v3.1.1 and v3.1.2,
  although the executor change in `trajectory_twin.py` was meant to
  reduce its overhead. The earlier "(DDS)" attribution was not verified.
  Treat this as an open item (`VL-16`) and re-measure before drawing
  conclusions.
- The single outlier (`max = 103575 ms`) is a **103-second stall** in
  Isaac Sim (Play/Pause or re-init), not a bridge or PLC issue. Excluded
  from the filtered statistics; its root cause is still open (`VL-14`).

**Conclusion:** the fixes reduced median full-loop latency by **30%**
(67.36 → 47.20 ms) and increased the fraction of samples under 50 ms
from **6.78% → 63.37%**. The remaining gap to the reference machine is
mostly environmental (VM), not algorithmic.

---

## Future Vision

The vendor-neutral philosophy is being extended to encompass broader hardware
ecosystems, software runtimes, and the **simulation layer** itself.

### 1. Unified Control Layer (Multi-PLC Vendor expansion)

- **Beckhoff TwinCAT 3 (TwinCAT 3.1):** Support is being added to integrate
  TwinCAT runtime environments natively as a core component of our
  vendor-neutral PLC architecture. This will allow developers to swap
  seamlessly between Siemens S7-1500 and Beckhoff TwinCAT controllers by
  adjusting runtime mapping files, without modifying the underlying Python
  trajectory or data translation layers.

### 2. Multi-Simulator Agnostic Framework

- **Isaac Sim** remains a first-class target, but will also be brought under
  the same vendor-neutral abstraction — so the digital twin is no longer
  locked to a single simulator.
- **CoppeliaSim** will be added as a **proof of concept** for multi-simulator
  support, demonstrating that the same PLC bridge, robot description, and
  trajectory stack can drive more than one physics engine through a thin,
  config-driven adapter.

The long-term goal is a fully portable digital twin: change the robot, the
PLC ecosystem (Siemens/TwinCAT), or the simulator by editing configuration —
not by rewriting the core platform.

---

## Repository Layout

```text
.
├── vendor-neutral/              # Active — Gen 3 (vendor-neutral platform)
│   ├── README.md                # Full Gen 3 documentation
│   ├── docs/
│   │   ├── latency_comparison.md
│   │   └── latency_full_loop_OPC-UA.md
│   ├── linux/
│   ├── windows/
│   ├── isaac/
│   ├── config/
│   ├── plc/
│   └── tools/
├── archive/
│   ├── kuka/                    # Gen 1 — KUKA KR210 (TCP/JSON)
│   └── yaskawa/                 # Gen 2 — Yaskawa GP110 (OPC UA)
├── docs/                        # Cross-generation documentation
└── README.md                    # This file
```

---

## Quick Start (Gen 3)

```bash
cd vendor-neutral

# 1. Windows: TCP/JSON server (only if driver: tcp_json)
cd windows && python plc_server_win.py --host 0.0.0.0 --port 5000

# 2. Isaac Sim Script Editor: run isaac/physics_twin.py

# 3. Linux: PLC bridge (with CPU pinning for latency)
cd linux
taskset -c 2 python3 plc_ros_bridge.py \
    --ros-args -p config:=../config/plc_ros_bridge.yaml

# 4. Linux: trajectory twin (Ruckig + state machine)
taskset -c 3 python3 trajectory_twin.py \
    --ros-args -p robot:=../config/robot.yaml
```

Full instructions: [`vendor-neutral/README.md`](vendor-neutral/README.md).

---

## Documentation

| Document                                                                                 | Description                                |
| :--------------------------------------------------------------------------------------- | :----------------------------------------- |
| [`vendor-neutral/README.md`](vendor-neutral/README.md)                                   | Gen 3 (active) — vendor-neutral platform   |
| [`vendor-neutral/docs/latency_comparison.md`](vendor-neutral/docs/latency_comparison.md) | Full latency comparison across transports  |
| [`archive/kuka/README.md`](kuka/README.md)                                               | Gen 1 — KUKA KR210                         |
| [`archive/yaskawa/docs/README.md`](yaskawa/docs/README.md)                               | Gen 2 — Yaskawa GP110                      |
| [`CHANGELOG.md`](CHANGELOG.md)                                                           | All generations                            |
| [`docs/comparison.md`](docs/comparison.md)                                               | Side-by-side comparison                    |

---

## How to Cite

If you use this software in academic work, please cite **the specific
version you used** (Version DOI is stable and does not change):

```bibtex
@software{damsarkho2026isaac,
  author       = {Damsarkho, Nebras},
  title        = {{Isaac Digital Twin — Vendor-Neutral Platform for
                   PLC, ROS 2, and Isaac Sim}},
  month        = oct,
  year         = 2026,
  publisher    = {Zenodo},
  version      = {3.1.3},
  doi          = {10.5281/zenodo.23672149},
  url          = {https://doi.org/10.5281/zenodo.23672149}
}
```

Or in plain text:

> Damsarkho, N. (2026). *Isaac Digital Twin — Vendor-Neutral Platform for PLC, ROS 2, and Isaac Sim* (Version 3.1.3) [Computer software]. Zenodo. https://doi.org/10.5281/zenodo.23672149

> **Note on DOIs:** the badge at the top of this file uses the
> **Concept DOI** (`10.5281/zenodo.23312011`) — it always resolves to
> the **latest** release. The citation above uses the **Version DOI**
> (`10.5281/zenodo.23672149`) — it is pinned to **v3.1.3**. If you want
> to cite "whatever is latest at the time of reading", use the Concept
> DOI instead, and omit the `version` field.

---

## Contact

- **YouTube:** [Digital Twin Series](https://www.youtube.com/playlist?list=PLYLnoPt4fbu8)
- **Developer:** Nebras — nebras4u@gmail.com
- **Repository:** [isaac-plc-digital-twin](https://github.com/Nebras4u/isaac-plc-digital-twin)

---

*Built for Industry 4.0 — isaac-plc-digital-twin © 2026*
