# Isaac Digital Twin — Multi-Generation Platform

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23312011.svg)](https://doi.org/10.5281/zenodo.23312011)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen)]()
[![Latency](https://img.shields.io/badge/Mean%20latency-5.2%20ms-success)](#latency-performance)
[![Max](https://img.shields.io/badge/Max%20latency-16.3%20ms-success)](#latency-performance)

An industrial-grade **Digital Twin platform** for robot arms driven by
**Siemens S7-1500** PLCs, synchronized with **NVIDIA Isaac Sim** through
**ROS 2**.

![S7-1500T ↔ ROS2 ↔ Isaac Sim](https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/yaskawa/docs/images/PLC-ROS2-ISAAC.jpg)

The repository hosts three generations of the platform, side by side:

| Generation                 | Robot               | Transport                  | Location                                         | Status       |
| :------------------------- | :------------------ | :------------------------- | :----------------------------------------------- | :----------- |
| **Gen 3 — Vendor-Neutral** | Any (config-driven) | tcp_json / opcua / plc_api | [`vendor-neutral/`](vendor-neutral/)             | **Active**   |
| Gen 2 — Yaskawa GP110      | Yaskawa GP110       | OPC UA                     | [`archive/v0.2-yaskawa/`](yaskawa/) | Reference    |
| Gen 1 — KUKA KR210         | KUKA KR210 L150 L   | TCP/JSON                   | [`archive/v1.0-kuka/`](kuka/)       | Reference    |

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
- **Full-loop latency: 5.2 ms mean, 16.3 ms max** (TCP-JSON optimized)
- **58% faster** than the original TCP-JSON baseline

**See [`vendor-neutral/README.md`](vendor-neutral/README.md) for full docs.**

---

## Recent Changes

See [`CHANGELOG.md`](CHANGELOG.md) (section *Unreleased*) for details.

- `krc_twin.py` is replaced by the vendor-neutral `trajectory_twin.py`,
  which reads joints, limits, and Ruckig parameters from `config/robot.yaml`.
- **Fix:** a persistent out-of-limits command no longer cancels an
  in-progress brake (the planner was reset every cycle, causing an abrupt
  stop and about 3 log lines per cycle at 60 Hz).
- **Fix:** on PLC reconnect, the bridge no longer publishes
  `/plc/emergency_stop = False` before the real PLC e-stop value has been
  read.
- Added the Safety Notice above.

---

## Latency Performance

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

**Result:** 100% of full-loop samples stay under 20 ms — well within the
control cycle for medium-speed industrial applications.

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

| Document                                                                                | Description                                |
| :-------------------------------------------------------------------------------------- | :----------------------------------------- |
| [`vendor-neutral/README.md`](vendor-neutral/README.md)                                   | Gen 3 (active) — vendor-neutral platform   |
| [`vendor-neutral/docs/latency_comparison.md`](vendor-neutral/docs/latency_comparison.md) | Full latency comparison across transports  |
| [`kuka/README.md`](kuka/README.md)                                                       | Gen 1 — KUKA KR210                         |
| [`yaskawa/README.md`](yaskawa/docs/README.md)                                            | Gen 2 — Yaskawa GP110                      |
| [`CHANGELOG.md`](CHANGELOG.md)                                                           | All generations                            |
| [`docs/comparison.md`](docs/comparison.md)                                               | Side-by-side comparison                    |

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
  version      = {3.0.1},
  doi          = {10.5281/zenodo.23312011},
  url          = {https://doi.org/10.5281/zenodo.23312011}
}
```

Or in plain text:

> Damsarkho, N. (2026). *Isaac Digital Twin — Vendor-Neutral Platform for PLC, ROS 2, and Isaac Sim* (Version 3.0.1) [Computer software]. Zenodo. https://doi.org/10.5281/zenodo.23312011

---

## Contact

- **YouTube:** [Digital Twin Series](https://www.youtube.com/playlist?list=PLYLnoPt4fbu8)
- **Developer:** Nebras — nebras4u@gmail.com
- **Repository:** [isaac-plc-digital-twin](https://github.com/Nebras4u/isaac-plc-digital-twin)

---

*Built for Industry 4.0 — isaac-plc-digital-twin © 2026*
