# Isaac Digital Twin — Multi-Generation Platform

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen)]()

An industrial-grade **Digital Twin platform** for robot arms driven by
**Siemens S7-1500** PLCs, synchronized with **NVIDIA Isaac Sim** through
**ROS 2**.

The repository hosts three generations of the platform, side by side:

| Generation | Robot | Transport | Location | Status |
| :--- | :--- | :--- | :--- | :--- |
| **Gen 3 — Vendor-Neutral** | Any (config-driven) | tcp_json / opcua / plc_api | [`vendor-neutral/`](vendor-neutral/) | ✅ **Active** |
| Gen 2 — Yaskawa GP110 | Yaskawa GP110 | OPC UA | [`yaskawa/`](yaskawa/) | ⏸️ Reference |
| Gen 1 — KUKA KR210 | KUKA KR210 L150 L | TCP/JSON | [`kuka/`](kuka/) | ⏸️ Reference |

---

## Active Development: `vendor-neutral/`

The current generation removes hard-coded coupling to any specific robot
or PLC vendor. **Switch robots and PLCs by editing YAML — not by editing
Python.**

Highlights:

- 3 transports in one abstraction: `tcp_json`, `opcua`, `plc_api`
- Robot description in `config/robot.yaml`
- Signal mapping in `config/plc_ros_bridge.yaml`
- Ruckig-based online trajectory generation
- Counter-based heartbeat protocol
- Sub-0.03° tracking accuracy verified

**→ See [`vendor-neutral/README.md`](vendor-neutral/README.md) for full docs.**

---

## Future Vision

The vendor-neutral philosophy is being extended beyond robots and PLCs to the
**simulation layer** itself.

- **Isaac Sim** remains a first-class target, but will also be brought under the
  same vendor-neutral abstraction — so the digital twin is no longer locked to a
  single simulator.
- **CoppeliaSim** will be added as a **proof of concept** for multi-simulator
  support, demonstrating that the same PLC bridge, robot description, and
  trajectory stack can drive more than one physics engine through a thin,
  config-driven adapter.

The long-term goal is a fully portable digital twin: change the robot, the PLC,
or the simulator by editing configuration — not by rewriting the core platform.

---

## Repository Layout

```text
.
├── vendor-neutral/       # Active — Gen 3 (vendor-neutral platform)
├── archive/
│   ├── kuka/        # Gen 1 — KUKA KR210 (TCP/JSON)
│   └── yaskawa/     # Gen 2 — Yaskawa GP110 (OPC UA)
└── docs/                 # Cross-generation documentation
```

---

## Quick Start (Gen 3)

```bash
cd vendor-neutral

# 1. Windows: TCP/JSON server (only if driver: tcp_json)
cd windows && python plc_server_win.py

# 2. Isaac Sim Script Editor: run isaac/physics_twin.py

# 3. Linux: PLC bridge
cd linux
python3 plc_ros_bridge.py --ros-args -p config:=../config/plc_ros_bridge.yaml

# 4. Linux: KRC digital twin
python3 krc_twin.py
```

Full instructions: [`vendor-neutral/README.md`](vendor-neutral/README.md).

---

## Documentation

| Document | Description |
| :--- | :--- |
| [`vendor-neutral/README.md`](vendor-neutral/README.md) | Gen 3 (active) — vendor-neutral platform |
| [`kuka/README.md`](kuka/README.md) | Gen 1 — KUKA KR210 |
| [`yaskawa/README.md`](yaskawa/README.md) | Gen 2 — Yaskawa GP110 |
| [`CHANGELOG.md`](CHANGELOG.md) | All generations |
| [`docs/comparison.md`](docs/comparison.md) | Side-by-side comparison |

---

## Contact

* **Developer:** Nebras — nebras4u@gmail.com
* **Repository:** [isaac-plc-digital-twin](https://github.com/Nebras4u/isaac-plc-digital-twin)

***

Built for Industry 4.0 — GlassSync © 2026
