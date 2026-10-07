# Isaac Digital Twin — Multi-Robot Platform

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Siemens](https://img.shields.io/badge/Siemens-S7--1500T-orange)](https://www.siemens.com)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-5.0-green)](https://developer.nvidia.com/isaac-sim)
[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.11-yellow)](https://www.python.org)

An industrial-grade **Digital Twin platform** for robot arms driven by
**Siemens S7-1500T** PLCs, synchronized with **NVIDIA Isaac Sim** through
**ROS 2** and **OPC UA**.

This repository documents the **complete evolution** of the platform across
three generations — from an initial KUKA KR210 prototype to the current
modular Yaskawa GP110 deployment.

## Table of Contents

- [Current Generation (v2.0)](#current-generation-v20)
- [Project Generations](#project-generations)
- [Repository Structure](#repository-structure)
- [Quick Start](#quick-start)
- [Results](#results)
- [Documentation](#documentation)
- [Roadmap](#roadmap)
- [License](#license)
- [Contact](#contact)

## Current Generation (v2.0)

**Robot:** Yaskawa GP110 (5-joint articulation)
**PLC:** Siemens S7-1500T (TIA Portal, OPC UA Server)
**Simulator:** NVIDIA Isaac Sim 5.0
**Bridge:** Modular Python under `src/plc_bridge/`

Real-time bidirectional synchronization between:

- Siemens S7-1500T PLC (OPC UA Server, Articulated arm 3D with orientation TO)
- NVIDIA Isaac Sim 5.0 (Yaskawa GP110, URDF, 5-joint articulation)
- ROS 2 Humble (`/joint_states`, `/tf`, `/clock`)

Includes Isaac Home offset calibration and joint renaming (`joint_6 -> joint_4`).

**Flow:**

[S7-1500T PLCSIM Advanced + PLCopen Motion + TO 4-Axis]
|
| OPC UA :4840 / ns=3
v
[Bridge — asyncua + rclpy, Home Offset + Joint Remap]
^
| ROS 2 /DDS /joint_states (5 joints)
|
[Isaac Sim 5.0 — PhysX 5 — Yaskawa GP110 — 5-joint articulation]


## Project Generations

This repository contains **three generations** of the same platform.

| Generation | Robot         | Transport | Architecture         | Location                |
|------------|---------------|-----------|----------------------|-------------------------|
| v1.0       | KUKA KR210    | TCP/JSON  | 5 separate processes | `archive/v1.0-kuka/`    |
| v0.2       | Yaskawa GP110 | OPC UA    | Monolithic bridge    | `archive/v0.2-yaskawa/` |
| v2.0       | Yaskawa GP110 | OPC UA    | Modular (6 files)    | `src/plc_bridge/`       |

### v1.0 — KUKA KR210 (Archived)

The original prototype. Complete KUKA KR210 digital twin with:

- Siemens S7-1500 PLC in PLCSIM Advanced
- TCP/JSON transport between Windows and Linux
- Ruckig online trajectory generation (KRC Digital Twin)
- Counter-based heartbeat protocol
- 4 controlled joints + 1 held passively

**Status:** Fully operational. Tracking error < 0.03°.
**Documentation:** `archive/v1.0-kuka/docs/PROJECT_DOCUMENTATION.txt`

### v0.2 — Yaskawa GP110 (Archived)

Migration from KUKA to Yaskawa. Key changes:

- Switched to OPC UA transport
- Migrated to Yaskawa GP110 URDF
- Renamed `joint_6 -> joint_4` for 4-axis PLC kinematics
- Discovered and fixed the **Hidden A5** problem
- Modified URDF for full tool-down orientation

**Status:** Superseded by v2.0 modular refactoring.

### v2.0 — Yaskawa GP110 (Current)

Modular refactoring of the v0.2 monolith into **6 focused modules**:

- `config.py` — single source of truth
- `joint_map.py` — pure conversion functions
- `plc_client.py` — OPC UA isolation
- `ros_bridge.py` — ROS 2 node isolation
- `main.py` — orchestration loop
- `plc_gui.py` — standalone manual control GUI

**Status:** Production-ready for simulation deployment.

## Repository Structureisaac-plc-digital-twin-v2/
|
+-- README.md This file
+-- CHANGELOG.md Version history (v0.1 to v2.0)
+-- LICENSE MIT
+-- .gitignore
|
+-- aas/ Asset Administration Shell
| +-- KUKA_KR210_L150_L/
| +-- Main_Robot_L_Yaskawa_GP110/
|
+-- docs/
| +-- README_v2_full.md Full v2.0 documentation (Appendices A-F)
| +-- images/ Screenshots & diagrams
| +-- appendices/ (in progress)
| +-- tia_portal_dh_kinematics/ DH calibration tool
|
+-- isaac_sim/ URDF + USD + Isaac scripts
| +-- Yaskawa_GP110_v1/
| +-- GP110.urdf
| +-- meshes/
|
+-- PLC_1500T/ PLC program notes
|
+-- src/
| +-- plc_bridge/ v2.0 modular code
| +-- config.py Constants (PLC URL, NS, signs, offsets)
| +-- joint_map.py PLC deg <-> ROS rad conversion
| +-- plc_client.py OPC UA layer (Node IDs + opc_write)
| +-- ros_bridge.py ROS 2 node (publish-only)
| +-- main.py Async orchestration loop
| +-- plc_gui.py Standalone tkinter GUI
| +-- README.md Module docs
|
+-- archive/
+-- v0.2-yaskawa/ v0.2 prototype (README only)
| +-- README_v0.2.md
+-- v1.0-kuka/ KUKA KR210 v1.0 (full project)
+-- README.md
+-- requirements.txt
+-- docs/
| +-- PROJECT_DOCUMENTATION.txt
| +-- commissioning.md
+-- isaac/
| +-- physics_twin.py
| +-- Full_Control.usd
| +-- Main_Robot_L_URDF_tool.urdf
+-- linux/
| +-- plc_ros_bridge.py
| +-- krc_twin.py
| +-- check_accuracy.py
+-- plc/
| +-- DB.txt
| +-- SCL_heartbeat_OB30.txt
| +-- SCL_MAIN.txt
+-- tools/
| +-- srv.py
| +-- cli.py
+-- windows/
+-- plc_server_win.py


## Tech Stack

| Component    | Technology                                          |
|--------------|-----------------------------------------------------|
| PLC          | Siemens S7-1500T (TIA Portal, OPC UA Server)        |
| Robot        | Yaskawa GP110 (5-joint articulation)                |
| URDF         | `Main_Robot_L_URDF_tool.urdf`                       |
| Middleware   | ROS 2 Humble, rclpy                                 |
| OPC UA       | `asyncua` (Python)                                  |
| Simulation   | NVIDIA Isaac Sim 5.0 (PhysX 5, RTX)                 |
| Protocols    | OPC UA (ns=3, port 4840), ROS 2 topics              |
| GUI          | tkinter (standalone manual control)                 |
| Python       | 3.10 (ROS 2 bridge), 3.11 (Isaac Sim)               |

## Quick Start

### Prerequisites

**Windows (PLC host):**

- Windows 10/11 (64-bit)
- TIA Portal V15.1 or later
- PLCSIM Advanced V6.0
- Python 3.x with `asyncua`
- Network: `192.168.100.x`

**Linux (Bridge host):**

- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- Python 3.10
- `asyncua`, `rclpy`, `numpy`

**Isaac Sim host:**

- NVIDIA Isaac Sim 5.0
- ROS 2 Bridge extension enabled
- Yaskawa GP110 URDF loaded at `/World/Main_Robot_L_Yaskawa_GP110`

### Installation

```bash
# Clone the repository
git clone https://github.com/Nebras4u/isaac-plc-digital-twin-v2.git
cd isaac-plc-digital-twin-v2

# Install Python dependencies
pip install -r requirements.txt

## Quick Start

### Prerequisites

Windows (PLC host):
- Windows 10/11 (64-bit)
- TIA Portal V15.1 or later
- PLCSIM Advanced V6.0
- Python 3.x with asyncua
- Network: 192.168.100.x

Linux (Bridge host):
- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- Python 3.10
- asyncua, rclpy, numpy

Isaac Sim host:
- NVIDIA Isaac Sim 5.0
- ROS 2 Bridge extension enabled
- Yaskawa GP110 URDF loaded at /World/Main_Robot_L_Yaskawa_GP110

### Installation

git clone https://github.com/Nebras4u/isaac-plc-digital-twin-v2.git
cd isaac-plc-digital-twin-v2
pip install -r requirements.txt

### Startup Sequence

Run the components in this exact order.

Step 1 - Windows (PLC Simulation):
Open TIA Portal, download the project to PLCSIM Advanced instance ros2_.
Ensure the instance is in RUN state.

Step 2 - Isaac Sim (Physics Twin):
Open Isaac Sim. Load the Yaskawa GP110 scene. Ensure the ROS 2 Bridge
extension is enabled and the Action Graph is configured. Switch the
simulation to Play mode.

Step 3 - Linux (Bridge):
cd src/plc_bridge
python3 main.py

Wait for:
PLC connected.
[bridge] PLC <-> ROS 2 bridge started.

Step 4 - Linux (Optional Manual GUI):
cd src/plc_bridge
python3 plc_gui.py

This opens a tkinter window with tabs for manual control:
- Commands (Bool) - toggle PLC flags
- Faults - read-only indicators (red = FAULT, green = OK)
- Speed - write MainRobotLeft_Vlcty
- Cmd / Maint - write CmdAng and Pos4Maint arrays
- Actual (read-only) - live view of ActAng and ActPos
- RESET FAULT 801 button at top

## Verification

### Topic Rates

ros2 topic hz /joint_states           # 25-60 Hz
ros2 topic hz /tf                     # 25-60 Hz
ros2 topic echo /tf --once            # 6 frames expected

### Zero-Pose Validation

At mechanical zero, verify the Cartesian match between PLC and Isaac:

ros2 topic echo /joint_states --once

Expected results:

| Axis | PLC         | Isaac       | Delta      |
|------|-------------|-------------|------------|
| X    | 3100.000 mm | 3100.003 mm | -0.003 mm  |
| Y    | 0.000 mm    | -0.000 mm   | 0.000 mm   |
| Z    | 330.000 mm  | 329.896 mm  | 0.104 mm   |

X and Y effectively exact. Z within simulation physics noise.

### OPC UA Connection Test

From the Linux host, run a quick Python check:

python3 -c "
from asyncua import Client
import asyncio

async def test():
    async with Client(url='opc.tcp://192.168.100.50:4840') as client:
        node = client.get_node('ns=3;s=\"MAIN_KUKA_KR210_L150_L\".\"MainRobotLeft_ActAng\"')
        val = await node.read_value()
        print('ActAng =', val)

asyncio.run(test())
"

Expected: a list of 4 joint values (in degrees).

## Results

### Tracking Accuracy (v1.0 KUKA KR210)

Command: CmdAng = [30.0, -30.0, 60.0, 0.0]
Settling time: 5 seconds

| Joint    | Setpoint   | Actual     | Error      |
|----------|------------|------------|------------|
| joint_1  | 30.0002°   | 30.0002°   | 0.0001°    |
| joint_2  | -29.9603°  | -29.9349°  | 0.0254°    |
| joint_3  | 60.0097°   | 60.0133°   | 0.0036°    |
| joint_4  | -0.0025°   | -0.0033°   | 0.0008°    |
| joint_5  | -0.4668°   | -0.4668°   | 0.0000°    |

Maximum error: 0.0254° (joint_2, under full gravity load).
This is well below typical industrial robot tracking error (0.1° without closed-loop encoders).

### Zero-Pose Cartesian Match (v0.2 Yaskawa)

| Axis | PLC         | Isaac       | Delta      |
|------|-------------|-------------|------------|
| X    | 3100.000 mm | 3100.003 mm | -0.003 mm  |
| Y    | 0.000 mm    | -0.000 mm   | 0.000 mm   |
| Z    | 330.000 mm  | 329.896 mm  | 0.104 mm   |

X and Y are effectively exact. Z differs by only ~0.1 mm, which is well within simulation physics noise.

### Hidden A5 Diagnosis (v0.2.2)

After zero-pose Cartesian synchronization to < 0.1 mm, a residual 85.7° offset
in tool angle remained. Variable isolation methodology traced the root cause
to joint_5 (A5) being moved manually instead of programmatically.

Mathematical verification:
- Theoretical offset: LF × sin(A5) = 89.084 × sin(3°) = 4.663 mm
- Measured offset: 4.683 mm
- Error: 0.02 mm (conclusive proof)

Solution: A5 = -(A1 + A4) for tool-down orientation.
Full diagnostic journey in Appendix E of docs/README_v2_full.md.

### Clock Drift (v1.0 KUKA, 15-min measurement)

| Metric              | Value       |
|---------------------|-------------|
| Duration            | 900 s       |
| Samples             | 293,585     |
| Packet Loss         | 0.00%       |
| Drift               | -0.065 ppm  |
| Standard Error      | 0.00016 ppm |
| Asymmetry Bound     | ±16.0 µs    |
| RTT (minimum)       | 32.0 µs     |

Drift of -0.065 ppm = accumulated error < 0.25 ms per hour.
This is 80× smaller than PLC cycle time (20 ms).

## Documentation

| Document                                            | Description                                |
|-----------------------------------------------------|--------------------------------------------|
| docs/README_v2_full.md                              | Full v2.0 documentation (Appendices A-F)   |
| docs/PROJECT_DOCUMENTATION.txt (in archive/v1.0-kuka) | KUKA v1.0 complete docs (18 sections)     |
| docs/commissioning.md (in archive/v1.0-kuka)        | KUKA v1.0 startup guide                    |
| archive/v0.2-yaskawa/README_v0.2.md                 | Yaskawa v0.2 README                        |
| CHANGELOG.md                                        | Version history (v0.1 to v2.0)             |

### Appendices in docs/README_v2_full.md

| Appendix | Title                                                       |
|----------|-------------------------------------------------------------|
| A        | Frame Hierarchy (after setup)                               |
| B        | ROS 2 Topics                                                |
| C        | Default Joint Names                                         |
| D        | Zero-Pose Validation Results                                |
| E        | Debugging Journey: The Hidden A5 Problem                    |
| F        | Modular Refactoring: From Monolith to Modular Architecture  |

## Technical Contributions

1. 5-joint remap for 4-axis PLC kinematics: joint_6 renamed to joint_4,
   joint_5 Python-driven, exposing exactly the joint set expected by
   TIA Portal's Articulated arm 3D with orientation block.

2. Isaac Home offset calibration: normalizes URDF bias so Isaac's mechanical
   zero matches Yaskawa's mechanical zero.

3. Real-time PLC ↔ simulation validation: compares joint states with per-axis
   deviation output.

4. Publish-only ROS 2 Action Graph: On Playback Tick → ROS2 Publish Joint State
   + ROS2 Publish Transform Tree (no subscriber, static=false).

5. URDF modification for tool-down orientation: joint_5 limits were too
   restrictive in the original URDF; constraints were relaxed and the file
   re-exported as Main_Robot_L_URDF_tool.urdf.

6. Hidden A5 diagnosis: identified joint_5 as a dynamic offset source when
   not driven programmatically; established the tool-down kinematic relation
   A5 = -(A1+A4).

7. Modular refactoring: from monolith to six focused modules with clear
   separation of concerns.

8. Counter-based heartbeat protocol (v1.0): immune to clock drift, PROFINET-style.

## Roadmap

### Completed

- [x] v0.1 — KUKA KR210 prototype (TCP/JSON, monolithic)
- [x] v1.0 — KUKA KR210 full digital twin (5 processes, Ruckig, heartbeat)
- [x] v0.2 — Yaskawa GP110 migration (OPC UA, joint_6 -> joint_4)
- [x] v0.2.1 — URDF edited for joint_5 full tool-down orientation
- [x] v0.2.2 — Hidden A5 diagnosed and confirmed as root cause
- [x] v2.0 — Modular refactoring (6 files under src/plc_bridge/)

### Planned

- [ ] v0.3 — IK node with What-If feasibility checks + A5 = -(A1+A4) kinematics
- [ ] v0.4 — Pneumatic suction gripper + glass sheet handling
- [ ] v0.5 — Force loop + grasp stability monitoring
- [ ] v1.0 (RT) — Domain randomization + full digital twin validation
- [ ] v1.1 — Real-Time Sync: bidirectional PLC <-> Isaac with automatic A5

## References

- **RAAD 2025** — "Isaac Sim Integrated Digital Twin For Feasibility Checks In
  Skill-based Engineering"
  DOI: [10.1007/978-3-032-02106-9_46](https://doi.org/10.1007/978-3-032-02106-9_46)

- **Yaskawa GP110** Datasheet (link lengths L1–L4 confirmed)

- **KUKA KR 210 L150** Datasheet

- **NVIDIA Isaac Sim 5.0** Documentation

- **ROS 2 Humble** Documentation

- **TIA Portal** V17+ — Articulated arm 3D with orientation (4-Axis TO)

- **Keep a Changelog** — https://keepachangelog.com/

- **Semantic Versioning** — https://semver.org/

## License

MIT License. See [LICENSE](LICENSE) for details.

## Contact

**Nebras** — nebras4u@gmail.com

**GitHub:** https://github.com/Nebras4u

**Repository:** https://github.com/Nebras4u/isaac-plc-digital-twin-v2

**YouTube:** https://youtube.com/playlist?list=PLYLnoPt4fbu8

---

Built for Industry 4.0 — GlassSync (c) 2026
