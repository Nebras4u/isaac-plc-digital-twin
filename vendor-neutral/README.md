# Isaac Digital Twin — Vendor-Neutral Platform

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![PLC](https://img.shields.io/badge/PLC-Vendor--Neutral-lightgrey)](#supported-plcs)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-5.0-green)](https://developer.nvidia.com/isaac-sim)
[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.11-yellow)](https://www.python.org)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen)]()

A **vendor-neutral Digital Twin platform** for industrial robot arms. The platform synchronizes any robot arm with any PLC through ROS 2, using a transport-agnostic bridge and a robot-agnostic configuration format.

This folder (`vendor-neutral/`) is the evolution of two robot-specific implementations (KUKA KR210 and Yaskawa GP110) into a reusable, configuration-driven platform.

---

## Table of Contents

- [Motivation](#motivation)
- [What "Vendor-Neutral" Means](#what-vendor-neutral-means)
- [Architecture](#architecture)
- [Repository Structure](#repository-structure)
- [Supported Combinations](#supported-combinations)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Results](#results)
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

The main nodes (`krc_twin.py`, `plc_ros_bridge.py`, `physics_twin.py`) stay focused on their responsibility and delegate these concerns to the modules above.

---

## What "Vendor-Neutral" Means

| Axis | Before | Now |
| :--- | :--- | :--- |
| **PLC vendor** | Siemens only | Any PLC with OPC UA, TCP/JSON, or PLCSIM API |
| **Robot vendor** | KUKA only, then Yaskawa only | Any articulated robot described in `robot.yaml` |
| **Transport** | TCP/JSON only, then OPC UA only | Selected by a single YAML line |

The same code runs unchanged across all combinations. Only configuration differs.

---

## Architecture

```text
┌──────────────────────────────────────────────────────────────────────┐
│                          PLC (any vendor)                            │
│   Siemens S7-1500 · Beckhoff TwinCAT · Rockwell · OPC UA servers     │
└────────────────────────────────┬─────────────────────────────────────┘
                                 │
                       one of three transports
                                 │
        ┌────────────────────────┼──────────────────────────┐
        │                        │                          │
   tcp_json                 opcua                     plc_api
 (Windows server)      (native OPC UA)         (PLCSIM direct)
        │                        │                          │
        └────────────────────────┼──────────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │  linux/plc_ros_bridge.py│
                    │  (+ plc_drivers.py)     │
                    │  (+ plc_config.py)      │
                    └────────────┬────────────┘
                                 │
                            ROS 2 topics
                                 │
                    ┌────────────▼────────────┐
                    │   linux/krc_twin.py     │
                    │  (Ruckig + state machine)│
                    └────────────┬────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │  isaac/physics_twin.py  │
                    │  (inside Isaac Sim)     │
                    └─────────────────────────┘
```

---

## Repository Structure

```text
vendor-neutral/
│
├── README.md                         # This file
├── CHANGELOG.md                      # Gen 3 history
│
├── windows/
│   └── plc_server_win.py             # PLCSIM Advanced -> TCP/JSON
│
├── linux/
│   ├── krc_twin.py                   # Ruckig trajectory + state machine
│   ├── plc_ros_bridge.py             # ROS 2 <-> PLC bridge
│   ├── plc_drivers.py                # 3 transports (tcp_json/opcua/plc_api)
│   └── plc_config.py                 # signal mapping + extractor
│
├── isaac/
│   ├── physics_twin.py               # Articulation driver
│   ├── Main_Robot_L_URDF_tool.urdf
│   └── Full_Control.usd
│
├── config/
│   ├── robot.yaml                    # Robot description
│   └── plc_ros_bridge.yaml           # Signal mapping
│
├── tools/
│   ├── latency_simpel.py             # Quick latency check
│   ├── latency_academic.py           # LBP / p99.9 / jitter
│   ├── srv.py                        # Clock responder
│   ├── cli.py                        # Clock drift client
│   └── info.txt
│
├── plc/
│   ├── SCL_heartbeat_OB30.txt
│   ├── SCL_MAIN.txt
│   ├── SCL_heartbeat_OB30-init.txt
│   ├── SCL_MAIN-init.txt
│   └── DB.txt
│
└── docs/
    ├── README_full.md
    └── migration.md
```

---

## Supported Combinations

| PLC | Transport | Robot | Status |
| :--- | :--- | :--- | :--- |
| Siemens S7-1500 (PLCSIM Adv.) | `plc_api` | KUKA KR210 | ✅ Verified |
| Siemens S7-1500 (PLCSIM Adv.) | `tcp_json` | KUKA KR210 | ✅ Verified |
| Any OPC UA server | `opcua` | Any robot in `robot.yaml` | ✅ Structural |
| Siemens S7-1500 | `opcua` | Yaskawa GP110 | ✅ Structural |

**"Structural"** = code path exercised, imports work; full end-to-end verification on that combination is pending.

---

## Quick Start

### Prerequisites

**Windows (only if using `tcp_json`):**
- Python 3.x with `pythonnet`
- PLCSIM Advanced V6.0

**Linux (bridge + KRC twin):**
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `pip install ruckig numpy pyyaml`

**Isaac Sim host:**
- Isaac Sim 4.x or 5.0
- ROS 2 Bridge extension enabled

### Startup Sequence

```bash
# 1. Windows: TCP/JSON server (only if driver: tcp_json)
cd windows && python plc_server_win.py

# 2. Isaac Sim Script Editor: physics twin
#    Run: isaac/physics_twin.py

# 3. Linux: PLC <-> ROS bridge
cd linux
python3 plc_ros_bridge.py --ros-args -p config:=../config/plc_ros_bridge.yaml

# 4. Linux: KRC digital twin (trajectory generator)
python3 krc_twin.py
```

### Switching Transport

Edit one line in `config/plc_ros_bridge.yaml`:

```yaml
plc:
  driver: opcua          # ← change to: tcp_json | opcua | plc_api
```

Everything else stays the same.

### Run Tests (unit tests for planner + config)

```bash
cd linux
python3 -m pytest ../tests -v
```

*(Tests to be added in a follow-up commit.)*

---

## Configuration

### `config/robot.yaml` — Robot description

Describes joints, limits, drives, Ruckig params. Currently the KRC twin reads these inline; migration to YAML is scheduled for v3.1.

### `config/plc_ros_bridge.yaml` — Signal mapping

```yaml
plc:
  driver: tcp_json       # tcp_json | opcua | plc_api
  host: 192.168.100.18
  port: 5000
  timeout: 0.015

vars:
  db: MAIN_KUKA_KR210_L150_L

signals:
  cmd_ang_1:  {addr: "{db}.MainRobotLeft_CmdAng[1]", type: float, dir: read}
  ...

robot:
  joints:  [joint_1, joint_2, joint_3, joint_4]
  cmd_ang: [cmd_ang_1, cmd_ang_2, cmd_ang_3, cmd_ang_4]
  act_ang: [act_ang_1, act_ang_2, act_ang_3, act_ang_4]
  cmd_ack: cmd_ack
```

Both files are validated at startup. If a signal referenced in the `robot` section is missing from `signals`, the bridge refuses to start.

---

## Results

### Tracking Accuracy

Measured on KUKA KR210 through `tcp_json`:

| Joint | Setpoint | Actual | Error |
| :--- | :--- | :--- | :--- |
| **joint_1** | 30.0002° | 30.0002° | 0.0001° |
| **joint_2** | -29.9603° | -29.9349° | 0.0254° |
| **joint_3** | 60.0097° | 60.0133° | 0.0036° |
| **joint_4** | -0.0025° | -0.0033° | 0.0008° |
| **joint_5** | -0.4668° | -0.4668° | 0.0000° |

Maximum error: **0.0254°** on `joint_2` under full gravity load.

### Control-Loop Latency

Measured with `tools/latency_academic.py`:

| Metric | Value |
| :--- | :--- |
| Mean | ~11.5 ms |
| Std dev | ~1.3 ms |
| p99.9 | ~14.3 ms |
| LBP(≤ 15 ms) | 100% |

### Clock Drift (15-minute measurement)

| Metric | Value |
| :--- | :--- |
| Duration | 900 s |
| Samples | 293,585 |
| Packet Loss | 0.00% |
| Drift | **-0.065 ppm** |
| Standard Error | 0.00016 ppm |
| Asymmetry Bound | ±16.0 µs |

Drift of -0.065 ppm = accumulated error < 0.25 ms per hour. This is 80× smaller than the PLC cycle time (20 ms).

---

## Documentation

| Document | Description |
| :--- | :--- |
| `docs/README_full.md` | Gen 3 full documentation |
| `docs/migration.md` | How this folder was split from v2.0.0 |
| `../CHANGELOG.md` | Version history (all generations) |

---

## Lineage

| Version | Robot | Transport | Architecture | Location |
| :--- | :--- | :--- | :--- | :--- |
| v0.1.0 | KUKA KR210 | TCP/JSON | Monolithic | archived |
| v1.0.0 | KUKA KR210 | TCP/JSON | 5 processes | `archive/v1.0-kuka/` |
| v0.2.0 | Yaskawa GP110 | OPC UA | Monolithic bridge | `archive/v0.2-yaskawa/` |
| v2.0.0 | Yaskawa GP110 | OPC UA | Modular (6 files) | `archive/v0.2-yaskawa/` |
| **v3.0.0** | **Any robot** | **Any transport** | **Vendor-neutral** | **`vendor-neutral/`** |

---

## Roadmap

### Completed
- [x] Extract PLC transports into `plc_drivers.py`
- [x] Extract signal handling into `plc_config.py`
- [x] All three transports selectable from one YAML line
- [x] Repository reorganized by environment (windows/linux/isaac/plc/tools)
- [x] Latency + drift measurement tools preserved

### In Progress
- [ ] Move `krc_twin.py` inline config into `config/robot.yaml`
- [ ] Unit tests for `plc_config` and `plc_drivers`
- [ ] Yaskawa GP110 verification on this branch

### Planned
- [ ] Cartesian motion layer (inverse kinematics)
- [ ] Trajectory queue and blending
- [ ] Certified safety layer (F-CPU + PROFIsafe)
- [ ] Native OPC UA on Windows (remove PLC server dependency)

---

## License

MIT License. See [LICENSE](LICENSE).

---

## Contact

* **Developer:** Nebras — nebras4u@gmail.com
* **Repository:** [isaac-plc-digital-twin-v2](https://github.com/Nebras4u/isaac-plc-digital-twin)
* **Location:** `vendor-neutral/`

***

*Built for Industry 4.0 — GlassSync © 2026*
