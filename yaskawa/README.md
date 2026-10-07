# KUKA KR210 Digital Twin — Industrial PLC Handshake

A **Digital Twin** of a KUKA KR210 industrial robot, built to demonstrate industrial-grade engineering: a real Siemens S7-1500 PLC drives the control loop through an OPC UA heartbeat protocol, while a physics simulation in NVIDIA Isaac Sim responds in real time.

The project is explicitly **not** a toy. It mirrors the architecture of real industrial cells: PLC as cycle master, KRC (KUKA Robot Controller) as trajectory generator, and the robot as the physical system. The only difference is that the robot lives in simulation rather than on the shop floor.

---

## Table of Contents

- Why this project exists
- Architecture
- Repository layout
- Components
- Running the system
- Design highlights
- Safety model
- Known limitations
- Roadmap
- License
- Author

---

## Why this project exists

Most "digital twin" demos online are either a single Python script that fakes both sides, or a MoveIt pipeline disconnected from any PLC logic. Neither proves that the author understands **industrial system design**: hardware/software boundaries, fail-safe behavior, cycle timing, and the difference between polling and event-driven control.

This project builds the whole chain, from PLC heartbeat to Ruckig trajectory generation to Isaac Sim physics, with **explicit safety layers** and a documented set of engineering decisions.

The goal is not to move a simulated robot. The goal is to show that the author can design a system the way a senior automation engineer would.

---

## Architecture

    ┌─────────────────────────────────────────────────────────────┐
    │ PLC (Siemens S7-1500)                                       │
    │   - Heartbeat master (OB30, 20 ms)                          │
    │   - Command source (CmdAng, CmdEnable)                      │
    │   - Feedback sink (ActAng, ActPos)                          │
    │   - Fail-safe logic (Comm_State machine)                    │
    └──────────────────────┬──────────────────────────────────────┘
                           │ OPC UA (Client/Server, 500 ms floor)
                           ▼
    ┌─────────────────────────────────────────────────────────────┐
    │ krc_twin.py (Linux / Python 3.10)                           │
    │   - OPC UA client with reconnect                            │
    │   - Heartbeat watchdog                                      │
    │   - Command validation                                      │
    │   - Ruckig Online Trajectory Generation (100 Hz)            │
    │   - Three-state machine: IDLE / RUNNING / STOPPING          │
    │   - Velocity-interface braking (IEC 60204-1 Stop Cat 1)     │
    └──────────────────────┬──────────────────────────────────────┘
                           │ ROS 2 (/servo_setpoints, /joint_states)
                           ▼
    ┌─────────────────────────────────────────────────────────────┐
    │ physics_twin.py (inside Isaac Sim, Python 3.11)             │
    │   - Articulation driver                                     │
    │   - 60 Hz physics feedback                                  │
    └─────────────────────────────────────────────────────────────┘

Detailed layer-by-layer view: docs/architecture.md.
The full reasoning behind every architectural choice: docs/engineering_decisions.md.

---

## Repository layout

    .
    ├── config.py              Central configuration (single source of truth)
    ├── joint_map.py           Pure unit-conversion functions
    ├── plc_client.py          OPC UA transport layer (S7-1500 compatible)
    ├── krc_twin.py            Digital twin of the KUKA Robot Controller
    ├── requirements.txt
    ├── LICENSE
    ├── .gitignore
    │
    ├── isaac/
    │   ├── physics_twin.py    Isaac Sim side of the digital twin
    │   └── README.md
    │
    ├── plc/
    │   ├── heartbeat.scl      OB30 code for the S7-1500 (heartbeat master)
    │   └── README.md
    │
    ├── gui/
    │   └── plc_gui.py         Standalone tkinter PLC control panel
    │
    ├── docs/
    │   ├── architecture.md    Layer-by-layer architecture
    │   ├── engineering_decisions.md   20 documented design decisions
    │   └── commissioning.md   Step-by-step startup guide
    │
    ├── tests/
    │   ├── test_joint_map.py
    │   └── test_planner.py
    │
    └── archive/               Historical versions, kept for reference
        ├── plc_ros_bridge4.py Monolithic ancestor
        ├── main.py            Simple PLC<->ROS bridge (pre-Ruckig)
        └── isaac_full_twin.py Single-process variant (KRC + physics merged)

---

## Components

### krc_twin.py — Digital twin of the KUKA Robot Controller

The industrial heart of the project. It runs two independent loops:

1. OPC UA loop — driven by the PLC heartbeat. Reads commands, validates them, writes actual angles and the Ack counter back. Reconnects on failure with a forced safe state.
2. Servo loop (100 Hz) — Ruckig online trajectory generation. Three-state machine (IDLE / RUNNING / STOPPING). Velocity-interface braking on any fault.

Key features:

- Feedback-driven trajectory — every cycle starts from the measured joint position, never from an open-loop model.
- Jump protection — large feedback discontinuities reset the derivative state instead of producing a violent correction.
- Command validation — NaN, inf, and out-of-range commands are rejected before they reach Ruckig.
- No fake zeros — if feedback is stale, the bridge does not write ActAng and does not acknowledge the heartbeat. The PLC faults out automatically.

### physics_twin.py — Isaac Sim side

A short script loaded into Isaac Sim's Script Editor. Subscribes to /servo_setpoints, drives the articulation at 60 Hz, and publishes /joint_states as feedback.

### plc/heartbeat.scl — PLC-side handshake

OB30 cyclic interrupt code for the S7-1500. Increments the heartbeat counter every 20 ms and runs a four-state communication state machine (Comm_State: OK / Late / Dead / BadAck). Forces CmdEnable = FALSE when ROS is dead.

### gui/plc_gui.py — Manual PLC control

A standalone tkinter application for testing PLC variables without launching ROS or Isaac Sim. Useful for field diagnostics:

- Commands (Bool) — toggle PLC flags
- Faults — read-only indicators (red = FAULT, green = OK)
- Speed — write the velocity setpoint
- Cmd / Maint — write command and maintenance arrays
- Actual — live view of actual angles and positions

### config.py — Single source of truth

All tunable parameters live here: PLC URL, node paths, heartbeat constants, joint mapping, unit conversion signs, control rates, and safety limits. Changing a sign or a limit is a one-line edit in one file.

### joint_map.py — Pure conversion functions

Two pure functions with no side effects:

- plc_deg_to_ros_rad(ax, deg) — PLC degrees → URDF radians
- ros_rad_to_plc_deg(ax, rad) — inverse

Plus bulk helpers for whole-vector conversion. Safe to unit-test with synthetic values.

### plc_client.py — OPC UA transport

Node ID definitions and a single low-level write helper compatible with the S7-1500 firmware. The opc_write() function avoids the BadWriteNotSupported error that occurs when a SourceTimestamp is included in the write request.

---

## Running the system

### Prerequisites

- TIA Portal V16+ for the full PLC project (V15.1 supports the base heartbeat SCL but not the optional LOpcUa library)
- ROS 2 Humble on Linux
- NVIDIA Isaac Sim (4.x or 2023.1+)
- Python packages: pip install -r requirements.txt
- ruckig must be installed in the ROS Python environment
- asyncua must be installed in both the ROS Python and Isaac Sim Python environments

See docs/commissioning.md for detailed setup.

### Startup order

1. Download the PLC project. Ensure Comm_State = 0 and the heartbeat counter is advancing.

2. In Isaac Sim Script Editor, run isaac/physics_twin.py. Confirm the articulation DOF list matches ROS_ORDER in config.py.

3. In a Linux terminal:

    source /opt/ros/humble/setup.bash
    cd kuka-digital-twin
    python3 krc_twin.py

   Expected output:

    KRC Twin connected to PLC.
    Subscribed to PLC heartbeat (500 ms).
    KRC Twin started (industrial version, Ruckig + jump protection + error recovery)

4. From TIA Portal (or gui/plc_gui.py), set CmdEnable = TRUE and write a target into CmdAng. The robot should move smoothly to the commanded pose.

### Verification

    ros2 topic hz /servo_setpoints    # ~100 Hz
    ros2 topic hz /joint_states       # ~60 Hz

If /servo_setpoints shows 0 Hz, the servo loop is blocked — check /joint_states freshness and the CmdEnable state.

---

## Design highlights

The full reasoning behind each architectural choice is documented in docs/engineering_decisions.md. Selected highlights:

| ID       | Decision                                       | Rationale                                              |
|----------|------------------------------------------------|--------------------------------------------------------|
| DEC-001  | PLC as heartbeat master, ROS as slave          | Mirrors PROFINET / EtherCAT practice                   |
| DEC-002  | Counter-based handshake                        | Immune to clock drift between PLC and PC               |
| DEC-004  | Ruckig for online trajectory generation        | Time-optimal, jerk-limited, production-proven          |
| DEC-005  | Jump protection with a 0.3 rad threshold       | Prevents violent corrections to noisy feedback         |
| DEC-006  | Velocity-interface braking                     | Matches IEC 60204-1 Stop Category 1                    |
| DEC-007  | Feedback injection every servo cycle           | Matches KRC interpolator behavior                      |
| DEC-010  | Command validation before use                  | Last line of defense against bad PLC data              |
| DEC-011  | No fake zeros on missing feedback              | Makes the fault visible at the PLC                     |
| DEC-013  | Transport isolated from control logic          | Enables future migration to PubSub / PROFINET / EtherCAT |
| DEC-019  | Isaac Sim as physics twin only                 | Keeps the KRC twin testable in isolation               |

---

## Safety model

The system is designed so that any fault produces a controlled stop, not an abrupt cut. This matches IEC 60204-1 Stop Category 1 (power retained during deceleration).

| Fault                                | Reaction                                              |
|--------------------------------------|-------------------------------------------------------|
| CmdEnable = FALSE                    | Velocity-interface braking, then IDLE                 |
| Heartbeat loss (> 2 s)               | force_disable = True, braking                         |
| /joint_states stale (> 0.5 s)        | Braking, no Ack written to the PLC                    |
| Command out of range or NaN          | Rejected; braking triggered                           |
| Feedback jump > 0.3 rad              | Derivatives reset; smooth recovery                    |
| 5 consecutive Ruckig errors          | Braking triggered                                     |
| OPC UA connection lost               | force_disable = True, retry every 2 s                 |

On the PLC side, the heartbeat protocol enforces a second layer of safety: if the Ack counter stops advancing, the PLC transitions to Comm_State = 2 (Dead) and forces CmdEnable = FALSE.

This is a simulation-grade controller. It is not certified for hard real-time control of physical hardware. Before connecting to a real robot, an independent safety layer (hardware E-stop, watchdog, safety relay) is required.

---

## Known limitations

- OPC UA Client/Server enforces a 500 ms publishing floor on this S7-1500 firmware. This is the dominant latency between the PLC and the KRC twin. It is documented, not hidden.
- PubSub over UDP would reduce the floor to ~1–10 ms but requires TIA Portal ≥ V16 and the Siemens LOpcUa library.
- EtherCAT is not supported natively by the S7-1500; it would need a separate communication processor.
- The bridge runs on standard Linux (not PREEMPT_RT), so timing jitter is bounded but not deterministic.
- Isaac Sim physics runs at 60 Hz; the servo loop at 100 Hz. The mismatch is handled by the queue-based handoff but adds a small latency.

---

## Roadmap

- [x] v0.1 — Monolithic PLC <-> ROS bridge
- [x] v0.2 — Modular architecture (config, joint_map, plc_client, gui)
- [x] v0.3 — Heartbeat handshake with the S7-1500
- [x] v0.4 — Ruckig trajectory generation and safety state machine
- [ ] v0.5 — OPC UA PubSub over UDP
- [ ] v0.6 — PROFINET IRT / EtherCAT path for hard real-time
- [ ] v0.7 — IK node and Cartesian motion
- [ ] v0.8 — Suction gripper integration

---

## License

MIT — see LICENSE.

---

## Author

Built as a portfolio project to demonstrate industrial system design: PLCs, industrial protocols, safety layers, robotics middleware, and physics simulation. Feedback and questions welcome.