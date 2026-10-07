# Isaac Digital Twin — Multi-Robot Platform

[![License](https://shields.io)](LICENSE)
[![Siemens](https://shields.io)](https://siemens.com)
[![ROS 2](https://shields.io)](https://ros.org)
[![Isaac Sim](https://shields.io)](https://nvidia.com)
[![Python](https://shields.io)](https://python.org)

An industrial-grade **Digital Twin platform** for robot arms driven by **Siemens S7-1500** PLCs, synchronized with **NVIDIA Isaac Sim** through **ROS 2**.

This repository documents the **continuous evolution** of the platform across two robot generations — from an initial Yaskawa GP110 prototype to the current **KUKA KR210** implementation with full Ruckig-based trajectory generation and counter-based heartbeat protocol.

---

## Table of Contents

- [Active Project — KUKA KR210](#active-project--kuka-kr210)
- [Project Evolution](#project-evolution)
- [Repository Structure](#repository-structure)
- [Quick Start](#quick-start)
- [Results](#results)
- [Documentation](#documentation)
- [Roadmap](#roadmap)
- [License](#license)
- [Contact](#contact)

---

## Active Project — KUKA KR210

* **Robot:** KUKA KR210 L150 L (4 controlled axes + 1 passive)
* **PLC:** Siemens S7-1500 in PLCSIM Advanced
* **Simulator:** NVIDIA Isaac Sim 5.0
* **Transport:** TCP/JSON between Windows and Linux
* **Location:** [`kuka/`](kuka/)

A complete five-layer Digital Twin with:

* **Ruckig online trajectory generation** — jerk-limited, time-optimal
* **KRC Digital Twin** — state machine (IDLE / RUNNING / STOPPING)
* **Counter-based heartbeat protocol** — immune to clock drift
* **Jump protection** — 0.3 rad threshold
* **Stop Category 1 braking** — per IEC 60204-1
* **Sub-0.03° tracking accuracy** — verified across all axes

### Architecture

```text
[PLC S7-1500] ←TCP/JSON→ [Windows Server] ←TCP/JSON→ [Linux Bridge]
                                                            ↓
                                                     [KRC Digital Twin]
                                                            ↓
                                                       [Isaac Sim]
```

### Performance (Verified)

| Metric | Value |
| :--- | :--- |
| Tracking error (max) | 0.0254° |
| Control rate | 60 Hz |
| PLC poll rate | 20 Hz |
| Heartbeat rate | 50 Hz |
| Clock drift | -0.065 ppm |
| Asymmetry bound | ±16 µs |

For full technical documentation, see [`kuka/docs/PROJECT_DOCUMENTATION.txt`](kuka/docs/PROJECT_DOCUMENTATION.txt).

---

## Project Evolution

This repository contains two generations of the same platform.

### Generation 1 — Yaskawa GP110 (Reference)

* **Location:** [`yaskawa/`](yaskawa/)

The initial implementation focused on Yaskawa GP110 with:
* OPC UA transport instead of TCP/JSON
* Modular bridge architecture (6 focused modules)
* Isaac Home offset calibration
* Joint renaming (`joint_6 → joint_4`)
* The **Hidden A5** diagnostic journey (Appendix E)

This generation is preserved as a **technical reference**, especially for:
* OPC UA integration patterns
* Modular Python architecture
* Zero-pose validation methodology
* The Hidden A5 problem and its solution

For full documentation, see [`yaskawa/docs/README_full.md`](yaskawa/docs/README_full.md).

### Generation 2 — KUKA KR210 (Active)

* **Location:** [`kuka/`](kuka/)

The current development line. Migrated from Yaskawa GP110 to KUKA KR210 with a focus on:
* TCP/JSON transport (Windows ↔ Linux)
* Counter-based heartbeat protocol
* Ruckig online trajectory generation
* Full KRC Digital Twin behavior

**Status:** Active development.

### Key Differences

| Aspect | Generation 1 (Yaskawa) | Generation 2 (KUKA) |
| :--- | :--- | :--- |
| **Robot** | Yaskawa GP110 | KUKA KR210 L150 L |
| **PLC** | S7-1500T | S7-1500 |
| **Transport** | OPC UA | TCP/JSON |
| **Trajectory** | Isaac native | Ruckig (KRC Twin) |
| **Heartbeat** | — | Counter-based (50 Hz) |
| **Architecture** | Modular (6 files) | 5 processes |
| **Location** | `yaskawa/` | `kuka/` |

---

## Repository Structure

```text
isaac-plc-digital-twin-v2/
│
├── README.md                           # This file
├── CHANGELOG.md                        # Version history
├── LICENSE                             # MIT License
├── .gitignore
│
├── kuka/                               # Active project — KUKA KR210
│   ├── README.md
│   ├── requirements.txt
│   ├── isaac/
│   │   ├── physics_twin.py
│   │   ├── Main_Robot_L_URDF_tool.urdf
│   │   └── Full_Control.usd
│   ├── linux/
│   │   ├── plc_ros_bridge.py
│   │   ├── krc_twin.py
│   │   └── check_accuracy.py
│   ├── plc/
│   │   ├── DB.txt
│   │   ├── SCL_heartbeat_OB30.txt
│   │   └── SCL_MAIN.txt
│   ├── tools/
│   │   ├── srv.py
│   │   └── cli.py
│   ├── windows/
│   │   └── plc_server_win.py
│   ├── docs/
│   │   ├── PROJECT_DOCUMENTATION.txt
│   │   └── commissioning.md
│   ├── config/
│   └── data/calibration/
│
└── yaskawa/                            # Reference — Yaskawa GP110
    ├── plc_bridge/
    ├── isaac_sim/
    ├── PLC_1500T/
    ├── aas/
    ├── docs/
    │   └── README_full.md
    └── images/
```

---

## Quick Start

The active project is **KUKA KR210** under `kuka/`. For Yaskawa GP110, see [`yaskawa/`](yaskawa/).

### Prerequisites

**Windows (PLC host):**
* Windows 10/11 (64-bit)
* TIA Portal V15.1 or later
* PLCSIM Advanced V6.0
* Python 3.x with `pythonnet`
* Network: `192.168.100.x`

**Linux (Bridge + KRC Twin):**
* Ubuntu 22.04
* ROS 2 Humble Hawksbill
* Python 3.10
* `ruckig`, `numpy`

**Isaac Sim host:**
* NVIDIA Isaac Sim 4.x or 5.0
* ROS 2 Bridge extension enabled
* KUKA KR210 URDF loaded

### Startup Sequence

```bash
# 1. Windows — PLC simulation
cd kuka/windows
python plc_server_win.py

# 2. Isaac Sim — Physics twin
# Run kuka/isaac/physics_twin.py in Isaac Sim Script Editor

# 3. Linux — ROS 2 bridge
cd kuka/linux
python3 plc_ros_bridge.py

# 4. Linux — KRC Digital Twin
python3 krc_twin.py

# 5. Linux — Accuracy verification (optional)
python3 check_accuracy.py
```

For full commissioning instructions, see [`kuka/docs/commissioning.md`](kuka/docs/commissioning.md).

---

## Results

### Tracking Accuracy (KUKA KR210)

After commanding `CmdAng = [30.0, -30.0, 60.0, 0.0]` and settling:

| Joint | Setpoint | Actual | Error |
| :--- | :--- | :--- | :--- |
| **joint_1** | 30.0002° | 30.0002° | 0.0001° |
| **joint_2** | -29.9603° | -29.9349° | 0.0254° |
| **joint_3** | 60.0097° | 60.0133° | 0.0036° |
| **joint_4** | -0.0025° | -0.0033° | 0.0008° |
| **joint_5** | -0.4668° | -0.4668° | 0.0000° |

* **Maximum error:** 0.0254° on `joint_2` under full gravity load.

### Clock Drift (15-minute measurement)

| Metric | Value |
| :--- | :--- |
| **Duration** | 900 s |
| **Samples** | 293,585 |
| **Packet Loss** | 0.00% |
| **Drift** | -0.065 ppm |
| **Standard Error** | 0.00016 ppm |
| **Asymmetry Bound** | ±16.0 µs |
| **RTT (minimum)** | 32.0 µs |

* Drift of `-0.065 ppm` = accumulated error `< 0.25 ms` per hour.
* This is **80× smaller** than the PLC cycle time (20 ms).

---

## Documentation

| Document | Description |
| :--- | :--- |
| [`kuka/docs/PROJECT_DOCUMENTATION.txt`](kuka/docs/PROJECT_DOCUMENTATION.txt) | KUKA KR210 full docs (18 sections) |
| [`kuka/docs/commissioning.md`](kuka/docs/commissioning.md) | KUKA KR210 startup guide |
| [`yaskawa/docs/README_full.md`](yaskawa/docs/README_full.md) | Yaskawa GP110 full docs (Appendices A–F) |
| [`CHANGELOG.md`](CHANGELOG.md) | Version history |

### Appendices in Yaskawa Docs
* **Appendix A:** Frame Hierarchy (after setup)
* **Appendix B:** ROS 2 Topics
* **Appendix C:** Default Joint Names
* **Appendix D:** Zero-Pose Validation Results
* **Appendix E:** Debugging Journey: The Hidden A5 Problem
* **Appendix F:** Modular Refactoring: From Monolith to Modular Architecture

---

## Roadmap

### Completed
- [x] Gen 1 — Yaskawa GP110 prototype (OPC UA, modular)
- [x] Gen 1 — Hidden A5 diagnosed and fixed
- [x] Gen 2 — KUKA KR210 migration (TCP/JSON)
- [x] Gen 2 — Ruckig online trajectory generation
- [x] Gen 2 — Counter-based heartbeat protocol
- [x] Gen 2 — Sub-0.03° tracking accuracy verified

### Planned
- [ ] Cartesian motion (inverse kinematics layer)
- [ ] Trajectory queue + blending
- [ ] Real-time sync: bidirectional PLC ↔ Isaac
- [ ] Certified safety layer (F-CPU + PROFIsafe)
- [ ] Migration to OPC UA / PROFINET

---

## License

MIT License. See [LICENSE](LICENSE) for details.

---

## Contact

* **Developer:** Nebras — nebras4u@gmail.com
* **GitHub Profile:** [Nebras4u](https://github.com)
* **Repository:** [isaac-plc-digital-twin-v2](https://github.com/isaac-plc-digital-twin-v2)
* **YouTube Playlist:** [Videos & Demos](https://youtube.com)

***

Built for Industry 4.0 — GlassSync © 2026
