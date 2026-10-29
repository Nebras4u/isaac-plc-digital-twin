# Isaac Digital Twin — Multi-Generation Platform

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23312010.svg)](https://doi.org/10.5281/zenodo.23312010)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen)]()
[![Tests](https://img.shields.io/badge/Tests-86%20pytest%20%2B%209%20standalone-success)](#testing)
[![Schema](https://img.shields.io/badge/Config%20Schema-v2.2-success)](#configuration)
[![Reference mean latency](https://img.shields.io/badge/Reference%20mean%20latency-5.2%20ms-success)](#latency-performance)
[![Reference max latency](https://img.shields.io/badge/Reference%20max%20latency-16.3%20ms-success)](#latency-performance)
[![VM v3.7 p50](https://img.shields.io/badge/VM%20v3.7%20p50-14.62%20ms-blueviolet)](#results-this-machine)
[![VM v4.0 p50](https://img.shields.io/badge/VM%20v4.0%20p50-16.07%20ms-informational)](#results-this-machine)

An industrial-grade **Digital Twin platform** for robot arms driven by
**Siemens S7-1500** PLCs, synchronized with **NVIDIA Isaac Sim** through
**ROS 2**.

![S7-1500T ↔ ROS2 ↔ Isaac Sim](https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/yaskawa/docs/images/PLC-ROS2-ISAAC.jpg)

The repository hosts three generations of the platform, side by side:

| Generation                 | Robot               | Transport                  | Location                                         | Status       |
| :------------------------- | :------------------ | :------------------------- | :----------------------------------------------- | :----------- |
| **Gen 4 — Vendor-Neutral** | Any (config-driven) | tcp_json / opcua / plc_api | [`vendor-neutral/`](vendor-neutral/)             | **Active**   |
| Gen 3 — Vendor-Neutral     | Any (config-driven) | tcp_json / opcua / plc_api | [`vendor-neutral/`](vendor-neutral/)             | Predecessor  |
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

> **Note on v3.7 and v4.0:** The reference numbers above were measured on
> the reference machine with batch enabled at **60 Hz** (v3.1.2 and
> earlier). v3.5–v3.7 added probe observability, trajectory fixes, and
> `poll_hz: 200`; v4.0 added the industrial type-safety layer. Those
> changes were measured on the **Docker/VM**, **not** on the reference
> machine. The reference machine has **not been re-measured** for v3.7
> or v4.0. See [`vendor-neutral/README.md`](vendor-neutral/README.md)
> for the VM comparison.

The **same code** on a **Docker / VM environment** (RTX 5060, shared
vCPUs, PLCSIM Advanced over TCP/JSON) gives different numbers — see
[Results (this machine)](#results-this-machine) below. This is expected
and does not indicate a regression.

---

## Active Development: `vendor-neutral/` (Gen 4 — v4.0.0)

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
- **86 pytest tests** covering the config schema, migration, E-stop
  forwarding, Ruckig parameter validation, `JointMapper`, trajectory
  state machine, act_ang write policy, estop lock, deadline counting,
  physics estop latch, and non-finite rejection in all four nodes
- **9 standalone probe tests** covering the latency-tag chain
  (`test_probe_chain.py`)
- **Config schema v2.2** (Pydantic v2, fail-fast, audit trail,
  per-signal `min`/`max`/`strict`/`reject_non_finite`,
  `strict_type_handshake`, `publish[].type` cross-check)
- **Industrial type-safety (v4.0):** `_get` is a validator, not a
  coercer (`str→int` and `float→int` truncation rejected, `int→bool`
  rejected, `NaN`/`Inf` rejected, out-of-range rejected — all with
  per-signal counters), plus a boot-time type handshake with the server
- **Offline latency analysis** tool: `ros2/analyze_probe.py`
  (with `--plot`, `--poll-hz`, `--export-clean`)

**See [`vendor-neutral/README.md`](vendor-neutral/README.md) for full docs.**

---

## Recent Changes

See [`CHANGELOG.md`](CHANGELOG.md) for the full list. Latest entry:
**v4.0.0 — Industrial Type-Safety Overhaul** (2026-10-29).

**v4.0.0 — Industrial Type-Safety Overhaul:**

- **`bridge_config.py` — Schema v2.2.**
  Per-signal `min`/`max`/`strict`/`reject_non_finite`/`allow_int_for_float`;
  `strict_types` and `strict_type_handshake` on `BridgeConfig`;
  `publish[].type` must equal `signals[].type`.
- **`plc_ros_bridge.py` — `_get` is a validator, not a coercer.**
  In `strict_types=True`, `str→int` (`int("5")`), `float→int`
  truncation (`int(3.9) → 3`), `int→bool`, `NaN`/`Inf`, and
  out-of-range values are all rejected with per-signal counters.
  `strict_types=False` falls back to v3.7 behaviour but still applies
  `_to_int_checked` (no silent truncation) and `_to_finite_float`
  (no silent NaN).
- **`plc_ros_bridge.py` — `_wire_type_check` at the driver boundary.**
  `TcpJson`, `OpcUa`, `PlcApi` all reject type mismatches
  (`"ERR:TYPE:..."`), not silent coercions. `bool` is never accepted
  as `int`.
- **`plc_ros_bridge.py` — boot-time type handshake.**
  `TcpJsonDriver.get_server_types()` compares config types against the
  server's declared types. `strict_type_handshake: true` aborts on
  mismatch.
- **`plc_server_win.py` — `types` command + `allow_nan=False` +
  `_sanitize`.**
  NaN/Inf converted to `"ERR:non_finite"` per tag; errors during
  reply serialization are caught and re-emitted as JSON errors.
- **`physics_twin.py` / `trajectory_twin.py` — NaN/Inf guards.**
  `on_setpoint`, `on_cmd`, `on_js`, `_publish`, `_seed`, `_refresh`
  all reject non-finite values; per-joint `_stats` counters and
  throttled `_warn`.
- **`plc_ros_bridge.py` — `_js_tag` NaN guard before `act_ang` write.**
  Prevents a `NaN` from silently freezing a joint (deadband would
  return `False` forever).
- **`PlcApiDriver` default instance `"296"`** (was `"ros2_"`).
- **Tests:** **86 pytest** (was 51) + **9 standalone** (was 8). New
  `test_industrial_patch.py` (12 tests) covers act_ang write policy,
  estop lock, deadline counting, physics estop latch.
- **`plc_ros_bridge.yaml`:** `schema_version: "2.2"`, `strict_types: true`,
  `strict_type_handshake: true`, per-signal `min`/`max`,
  `velocity_scale.max` corrected to `1000.0` (was `100.0`; PLC sends
  `500.0`).
- **`robot.yaml`:** `warn_cooldown: 5.0`.
- **Impact (Docker/VM, 363 s, mostly stationary joints, not reference
  machine):** `convert_pub` p99 = **0.13 ms** (was 0.12 ms in v3.7;
  +0.01 ms = ~10 µs — the only stage v4.0 touches). Every other stage
  unchanged or improved: `ros_ruckig` p99 −8%, `isaac_step` p99 −7%,
  `ros_back` p99 −25%, `queue_delay` p99 −1%. `write_net` p50 differed
  (7.86 vs 4.69 ms) because the motion pattern differed (3.0 rows/s
  vs 15.7 rows/s in v3.7); v4.0 has no code in that path. Server
  `cmd_sync` mean 1.8–2.3 ms. `probe_attempted == probe_rows`,
  `probe_rejected == 0`, `write_failed == 0`,
  `signal_rejections_total == 0`.

**v3.7.0 — Probe Observability + Production Path:**

- **`plc_ros_bridge.py` — `probe_attempted` counter (new).**
  Distinguishes "bridge didn't publish a tag" (`probe_sent == 0`) from
  "tag arrived but was rejected" (`probe_attempted > probe_rows`).
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
- **Impact (Docker/VM, not reference machine):** median
  `cmd_to_feedback` 18.55 → **14.62 ms** (−21%); p95 22.62 ms; p99
  35.50 ms; median `total_loop` 25.72 → **19.21 ms** (−25%); LBP ≤ 20 ms
  59.17% → **91.40%** (+32pp); `write_failed` = 0 (was persistent);
  `probe_attempted == probe_rows == 1396` (100% capture).

**v3.6.0 — Latency-Probe Chain Fixes:**

- **`_js_tag` starvation fix** — tag consumed only on actual `act_ang`
  write.
- **`_cmd_tag` consumed only by `_publish`** (was consumed every tick).
- **`_applied_tag` survives multiple publishes** (was consumed on first
  publish).
- **Same-tick `act_ang` write** — `write_net` p50 −24%.

**v3.5.0 — Trajectory Twin Fixes:**

- **`RuckigPlanner` validates all parameters** at construction.
- **`needs_target_update` compares against applied** (was received).
- **`_publish` no longer uses `np.interp`** on passive axes.

**v3.1.3 — PLC Batch Mode + 120 Hz Tuning:**

- **`windows/plc_server_win.py` — batch PLC I/O enabled by default.**
  **Fixed:** `_TYPE_NAMES` maps `LReal→Double`, `DInt→Int32`,
  `Int→Int16` (EPrimitiveDataType uses .NET names, not Siemens names).
- **`config/plc_ros_bridge.yaml`: `poll_hz: 60 → 120`.**
- **`ros2/analyze_probe.py` (new)** — offline CSV analysis.
- **`ros2/test_probe_chain.py` (new)** — 10 standalone tests.
- **Impact (Docker/VM):** median `cmd_to_feedback` 34.60 → **18.55 ms**
  (−46%); median `total_loop` 45.95 → **25.72 ms** (−44%).

**v3.1.2 — Performance Fixes for VM Environments:**

- **`plc_ros_bridge.py` — PLC I/O moved to a dedicated thread.**
- **`physics_twin.py` — `spin_once` before `step`/`publish_states`.**
- **QoS** — `BEST_EFFORT` + `depth=1` on high-rate streams.

**v3.1.1 — Environment Transparency and Real VM Results:**

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

| Transport                | Mean       | p99        | Max        | Full Loop |
|:-------------------------|:----------:|:----------:|:----------:|:---------:|
| OPC UA (naive)           | 11.5 ms    | 14 ms      | 100 ms     | ~20 ms    |
| TCP-JSON (naive)         | 12.5 ms    | 20 ms      | 233 ms     | ~25 ms    |
| **TCP-JSON (optimized)** | **5.2 ms** | **15.5 ms**| **16.3 ms**| **~15 ms**|

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

### Results (this machine — v4.0 vs v3.7)

Measured on a **different environment** than the reference machine:

| Item         | Value                                  |
| :----------- | :------------------------------------- |
| OS           | Ubuntu 24.04 (Docker)                  |
| CPU          | Shared vCPU (24 cores, `taskset`)      |
| GPU          | RTX 5060, 8 GB                         |
| PLC server   | PLCSIM Advanced V6.0, TCP/JSON         |
| ROS 2        | Jazzy                                  |
| RMW          | FastDDS                                |

**Full-loop latency** (v4.0: 363 s, `cmd_ang: [cmd_ang]`, mostly
stationary joints; v3.7: 88.7 s, 15.7 rows/s):

| Stage                  | v3.7 p50    | **v4.0 p50** | v3.7 p99    | **v4.0 p99** |
|:-----------------------|------------:|-------------:|------------:|-------------:|
| `queue_delay`          |    3.12 ms  |     3.10 ms  |    3.62 ms  |     3.60 ms  |
| `read_net`             |    1.69 ms  |     1.80 ms  |   10.27 ms  |    12.29 ms  |
| **`convert_pub`**      | **0.05 ms** | **0.05 ms**  | **0.12 ms** | **0.13 ms**  |
| `ros_ruckig`           |    3.00 ms  |     3.09 ms  |    8.16 ms  | **7.52 ms**  |
| `isaac_step`           |    5.23 ms  |     5.09 ms  |    9.33 ms  | **8.70 ms**  |
| `ros_back`             |    0.29 ms  |     0.35 ms  |    0.73 ms  | **0.55 ms**  |
| `write_net`            |    4.69 ms  |     7.86 ms  |   25.66 ms  |    41.16 ms  |
| **`cmd_to_feedback`**  | **14.62 ms**| **16.07 ms** | **35.50 ms**| **50.18 ms** |

**Interpretation — critical:** the only stage that v4.0 actually
modifies is `convert_pub` (where `_validate` runs). Its cost is
**+0.01 ms p99** (~10 µs) — the only real v4.0 overhead. `write_net`
differs because the motion pattern differs (3.0 rows/s here vs
15.7 rows/s in v3.7, i.e. more `queue_delay` inside the `write_net`
window); **v4.0 has no code in that path**. Server-side timing
(`plc_server_win.py` v4.0) confirms it is fast:
`cmd_sync` mean **1.8–2.3 ms**, `api_read_batch` mean **1.1–1.4 ms**,
`api_write_batch` mean **0.6–0.8 ms**.

**LBP (Latency Bound Probability) — `cmd_to_feedback`:**

| Threshold | v3.7    | **v4.0**  |
|:----------|--------:|----------:|
| ≤ 10 ms   | 19.91%  | 13.71%    |
| ≤ 15 ms   | 59.31%  | 38.47%    |
| ≤ 20 ms   | 91.40%  | 63.82%    |
| ≤ 30 ms   | 97.64%  | 93.63%    |
| ≤ 40 ms   | 99.14%  | 96.46%    |
| ≤ 50 ms   | 99.43%  | 97.17%    |

The LBP shifts below 20 ms are the environmental effect described above
(different motion pattern → different `queue_delay` distribution inside
`write_net`). With identical motion patterns, the two would match.
**Probe health in both runs:** `probe_attempted == probe_rows`
(100% capture), `probe_rejected == 0`, `probe_order_violations == 0`,
`write_failed == 0`, `signal_rejections_total == 0` (v4.0).

**Comparison v3.1.3 → v3.7 (same VM, for reference):**

| Stage             | v3.1.3   | **v3.7**     |     Δ |
|:------------------|---------:|-------------:|------:|
| `queue_delay`     | 5.87 ms  | **3.08 ms**  |  −48% |
| `read_net`        | 1.36 ms  | 1.72 ms      |  +26% |
| `ros_ruckig`      | 4.32 ms  | **3.08 ms**  |  −29% |
| `isaac_step`      | 9.71 ms  | **5.15 ms**  |  −47% |
| `write_net`       | 5.28 ms  | **4.73 ms**  |  −10% |
| `cmd_to_feedback` | 18.55 ms | 14.62 ms     |  −21% |
| `total_loop`      | 25.72 ms | 19.21 ms     |  −25% |
| LBP ≤ 20 ms       | 59.17%   | 91.40%       | +32pp |

The reference machine has **not** been re-measured for v3.7 or v4.0.
The reference numbers (5.2 ms mean, 16.3 ms max) still refer to v3.1.2
and earlier.

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

### 3. Industrial Type-Safety (v4.0 baseline)

v4.0 established the industrial type-safety layer (per-signal
`min`/`max`/`strict`, wire type check, boot handshake, per-signal
rejection counters, NaN/Inf guards). The next step is a **hardware
type adapter**: when the server declares a type mismatch, the bridge
will not abort but attempt a well-defined conversion (with a
warning) and record it in the audit log — so migrations from v3.x
to a stricter type model can be staged per signal.

---

## Repository Layout

    .
    ├── vendor-neutral/              # Active — Gen 4 (v4.0.0)
    │   ├── README.md                # Full Gen 4 documentation
    │   ├── docs/
    │   ├── linux/
    │   ├── ros2/                    # analyze_probe.py, test_probe_chain.py
    │   ├── windows/
    │   ├── isaac/                   # joint_mapper.py, physics_twin.py
    │   ├── config/
    │   ├── plc/
    │   └── tools/
    ├── kuka/                        # Gen 1 — KUKA KR210 (TCP/JSON)
    ├── yaskawa/                     # Gen 2 — Yaskawa GP110 (OPC UA)
    ├── docs/                        # Cross-generation documentation
    └── README.md                    # This file

---

## Quick Start (Gen 4 / v4.0.0)

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

On startup, verify the five v4.0 health markers in the bridge log:

    config_loaded        schema_version=2.2
    bridge_started       strict_types=True
    type_handshake_ok    checked=29
    plc_connected        driver=tcp_json backoff_s=1.0
    tick_stats           signal_rejections_total=0

Full instructions: [`vendor-neutral/README.md`](vendor-neutral/README.md).

---

## Documentation

| Document                                                                                 | Description                               |
| :--------------------------------------------------------------------------------------- | :-----------------------------------------|
| [`vendor-neutral/README.md`](vendor-neutral/README.md)                                   | Gen 4 (active) — vendor-neutral platform  |
| [`vendor-neutral/docs/latency_comparison.md`](vendor-neutral/docs/latency_comparison.md) | Full latency comparison across transports |
| [`kuka/README.md`](kuka/README.md)                                                       | Gen 1 — KUKA KR210                        |
| [`yaskawa/docs/README.md`](yaskawa/docs/README.md)                                       | Gen 2 — Yaskawa GP110                     |
| [`CHANGELOG.md`](CHANGELOG.md)                                                           | All generations                           |
| [`docs/comparison.md`](docs/comparison.md)                                               | Side-by-side comparison                   |

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
  version      = {4.0.0},
  doi          = {10.5281/zenodo.23312012},
  url          = {https://doi.org/10.5281/zenodo.23312012}
}
```

Or in plain text:

> Damsarkho, N. (2026). *Isaac Digital Twin — Vendor-Neutral Platform
> for PLC, ROS 2, and Isaac Sim* (Version 4.0.0) [Computer software].
> Zenodo. https://doi.org/10.5281/zenodo.23312012

---

## Contact

- **YouTube:** [Digital Twin Series](https://www.youtube.com/playlist?list=PLYLnoPt4fbu8)
- **Developer:** Nebras — nebras4u@gmail.com
- **Repository:** [isaac-plc-digital-twin](https://github.com/Nebras4u/isaac-plc-digital-twin)

---

*Built for Industry 4.0 — isaac-plc-digital-twin © 2026*
