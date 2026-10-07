# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] — 2026-10-07

### Changed (Breaking)

- **Architecture**: Monolithic bridge split into 6 focused modules under `src/plc_bridge/`:
  - `config.py` — single source of truth for constants
  - `joint_map.py` — pure PLC <-> ROS conversion functions
  - `plc_client.py` — OPC UA layer isolation
  - `ros_bridge.py` — ROS 2 node isolation
  - `main.py` — async orchestration loop
  - `plc_gui.py` — standalone tkinter GUI for manual control
- **Repository structure**: Historical versions moved to `archive/`
  - `archive/v1.0-kuka/` — KUKA KR210 L150 L (original prototype)
  - `archive/v0.2-yaskawa/` — Yaskawa GP110 migration README
- **Repository renamed**: `isaac-plc-digital-twin` -> `isaac-plc-digital-twin-v2`

### Added

- `src/plc_bridge/plc_gui.py` — standalone tkinter GUI with tabs:
  - Commands (Bool)
  - Faults (read-only)
  - Speed (LReal)
  - Cmd / Maint (arrays)
  - Actual (read-only)
  - RESET FAULT 801 button
- `docs/README_v2_full.md` — full v2.0 documentation (742 lines)
  - Appendix A — Frame Hierarchy
  - Appendix B — ROS 2 Topics
  - Appendix C — Default Joint Names
  - Appendix D — Zero-Pose Validation Results
  - Appendix E — Debugging Journey: The Hidden A5 Problem
  - Appendix F — Modular Refactoring

### Preserved

- v0.2 monolithic code in `archive/`
- v1.0 KUKA KR210 full project in `archive/v1.0-kuka/`
- All diagnostic appendices (A–F)

### Fixed

- **Hidden A5 problem** (diagnosed in v0.2.2, documented in Appendix E):
  - joint_5 was being moved manually, causing dynamic XY offsets (up to 11 mm)
  - Root cause confirmed: A5 != 0 introduces horizontal offset = LF * sin(A5)
  - Mathematical verification: 4.66 mm (theory) vs 4.68 mm (measurement)
  - Solution: A5 = -(A1 + A4) for tool-down orientation
  
## [0.2.2] — 2026-09-30

### Fixed

- **Hidden A5 axis diagnosed** as the root cause of residual XY/Z errors.
- After synchronizing Cartesian position to < 0.1 mm at zero pose, a residual
  85.7 deg offset in the tool angle remained, plus unexplained XY displacement
  in non-zero poses.
- Variable isolation methodology applied:
  - A1–A4 joint comparison: matched closely
  - TCP Cartesian comparison: Z constant ~6.8 mm offset
  - Frame analysis: link_4 correct, tool had unexplained XY offset
  - A4 test: offset does not rotate with A4
  - A1 test: offset rotates with A1, magnitude changed 8.2 -> 4.7 mm
  - /joint_states read: A5 = 3.000 deg confirmed
- Mathematical verification:
  - Horizontal offset = LF * sin(A5) = 89.084 * sin(3 deg) = 4.663 mm
  - Measured: 4.683 mm
  - Error: 0.02 mm — conclusive proof

### Added

- Appendix E in `docs/README_v2_full.md` — full diagnostic journey
- Tool-down kinematic relation: `A5 = -(A1 + A4)`
- Lessons learned table (7 items)

### Documented

- Hidden axes must be driven programmatically
- Manual jogging introduces dynamic offsets
- `/joint_states` is the primary diagnostic source

## [0.2.1] — 2026-09-25

### Changed

- **URDF modified**: `Main_Robot_L_URDF_tool.urdf` created.
- Original URDF imposed joint limits on `joint_5` that prevented full
  tool-down orientation.
- Constraints relaxed in the new URDF file.
- Tool axis can now align with world -Z direction (tool-down), required for
  glass sheet handling use case.
- `joint_5` remains hidden from PLC and is still driven by Python.

### Added

- Visual comparison: `docs/images/urdf_joint5_before_after.jpg`
- `docs/README_v2_full.md` Section 4.3 documents the URDF modification

### Documented

- Impact: joint_5 can rotate freely enough to align tool axis with -Z
- Note: joint_5 remains hidden from PLC

## [0.2.0] — 2026-09-20

### Changed (Breaking)

- **Robot migrated**: KUKA KR210 L150 L -> Yaskawa GP110
- **PLC**: Siemens S7-1500 -> Siemens S7-1500T
- **Transport**: TCP/JSON -> OPC UA (port 4840, namespace 3)
- **TO**: Switched to "Articulated arm 3D with orientation" (4-axis)
- **Joint renaming**: `joint_6 -> joint_4` to match 4-axis PLC TO
- **Joint count**: 5-joint articulation (4 controlled + 1 hidden)
- **`$MAMES` no longer needed** (replaced by Home Offset calibration)

### Added

- `Main_Robot_L_URDF_tool.urdf` — modified URDF
- `Yaskawa_GP110_v1/` — URDF + STL meshes
- Isaac Home Offset calibration
- ROS 2 Action Graph (publish-only, no subscribers)
- OPC UA bridge (`asyncua` + `rclpy`)
- Zero-pose validation:
  - X: 3100.000 mm (PLC) vs 3100.003 mm (Isaac) -> -0.003 mm
  - Y: 0.000 mm vs -0.000 mm -> 0.000 mm
  - Z: 330.000 mm vs 329.896 mm -> 0.104 mm
- Link parameter measurements (L1–L4 verified against datasheet)

### Fixed

- Action Graph configuration for TF publishing
- Duplicate frames (deleted stray `base_joint`)
- Articulation Root setup
- Joint damping / stiffness tuning

### Documented

- Appendix A — Frame Hierarchy
- Appendix B — ROS 2 Topics
- Appendix C — Default Joint Names
- Appendix D — Zero-Pose Validation Results

## [1.0.0] — 2026-09-15

### Added (Complete KUKA KR210 Digital Twin)

**PLC Layer (Siemens S7-1500 in PLCSIM Advanced):**

- `plc/heartbeat.scl` — OB30 heartbeat protocol, runs every 20 ms
  - Counter-based handshake (immune to clock drift)
  - State machine: OK / Late / Dead / BadAck
  - Forces `CmdEnable=FALSE` when ROS is dead
  - `HB_Timeout_Mult = 50` cycles before Late
- `plc/SCL_MAIN.txt` — OB1 motion logic
  - `MC_PickUp1..3` — approach and pickup sequence
  - `MC_Stkng_L1..L6` — left stacking
  - `MC_Stkng_R1..R6` — right stacking
  - `MC_Go2HomePos`, `MC_Pos4Maint` — manual routines
  - `MC_UpTheLinePos_Return` — return to home
  - Uses `TO_Kinematics` for inverse kinematics
- `plc/DB.txt` — data block reference

**Windows Layer:**

- `windows/plc_server_win.py` — TCP/JSON server bridging PLCSIM Advanced
  - Uses `pythonnet` to call PLCSIM API directly
  - Exposes ~30 tags via newline-delimited JSON
  - Commands: `read`, `write`, `list`
  - Listens on `0.0.0.0:5000`, one thread per client

**Linux Layer:**

- `linux/plc_ros_bridge.py` — bidirectional bridge between PLCSIM and ROS 2
  - Polls PLC @ 20 Hz
  - Publishes `/plc/cmd_ang`, `/plc/axis_enable`, `/plc/emergency_stop`,
    `/plc/velocity`, `/plc/ros_alive`
  - Subscribes to `/joint_states`
  - Writes `ROS_Ack_Counter` and `ActAng[1..4]` back to PLC
  - Unit conversion: degrees (PLC) <-> radians (ROS)
- `linux/krc_twin.py` — KRC Digital Twin with Ruckig online trajectory generation
  - `CONTROL_HZ = 60`
  - `MAX_VEL = 1.5 rad/s`, `MAX_ACC = 5.0 rad/s²`, `MAX_JERK = 30.0 rad/s³`
  - State machine: IDLE / RUNNING / STOPPING
  - Jump protection: 0.3 rad threshold
  - Braking: velocity-interface, Stop Category 1 (IEC 60204-1)
  - Feedback-driven: syncs from actual every cycle
- `linux/check_accuracy.py` — tracking accuracy verification tool
  - Subscribes to `/servo_setpoints` and `/joint_states`
  - Prints comparison table every second

**Isaac Sim Layer:**

- `isaac/physics_twin.py` — articulation driver for KR210 URDF
  - Subscribes to `/servo_setpoints`
  - Publishes `/joint_states` @ 60 Hz
  - Per-joint drive gains (stiffness up to 1,000,000)
  - Gravity compensation: `[0.0, 1200.0, 600.0, 0.0, 50.0]` N·m
  - Holds non-commanded joints at current position
- `isaac/Main_Robot_L_URDF_tool.urdf` — modified URDF
- `isaac/Full_Control.usd` — Isaac Sim scene

**Tools:**

- `tools/srv.py` — clock drift measurement (UDP responder, CLK1 protocol)
- `tools/cli.py` — clock drift measurement (client, same protocol)

**Documentation:**

- `docs/PROJECT_DOCUMENTATION.txt` — complete 18-section documentation
- `docs/commissioning.md` — startup procedure (verified working)

### Performance

**Tracking Accuracy (after commanding `CmdAng = [30, -30, 60, 0]`, 5 s settling):**

| Joint    | Setpoint   | Actual     | Error      |
|----------|------------|------------|------------|
| joint_1  | 30.0002°   | 30.0002°   | 0.0001°    |
| joint_2  | -29.9603°  | -29.9349°  | 0.0254°    |
| joint_3  | 60.0097°   | 60.0133°   | 0.0036°    |
| joint_4  | -0.0025°   | -0.0033°   | 0.0008°    |
| joint_5  | -0.4668°   | -0.4668°   | 0.0000°    |

Maximum error: **0.0254°** (joint_2, under full gravity load).

**Clock Drift (measured over 15 minutes):**

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

### Engineering Decisions

| ID      | Decision                                              |
|---------|-------------------------------------------------------|
| DEC-001 | PLC as heartbeat master, ROS as slave                 |
| DEC-002 | Counter-based handshake, not timestamp-based          |
| DEC-003 | Separate Python processes: 3.10 (ROS) + 3.11 (Isaac)  |
| DEC-004 | Ruckig for online trajectory generation               |
| DEC-005 | Jump protection with 0.3 rad threshold                |
| DEC-006 | Velocity-interface braking (Stop Category 1)          |
| DEC-007 | Feedback injection every servo cycle                  |
| DEC-008 | `pass_to_input()` after every Ruckig step             |
| DEC-009 | Three-state machine: IDLE / RUNNING / STOPPING        |
| DEC-010 | Command validation before use                         |
| DEC-011 | No fake zeros on missing feedback                     |
| DEC-012 | TCP/JSON transport (not OPC UA in this version)       |
| DEC-013 | Transport isolated from control logic                 |
| DEC-014 | Rate-limited logging                                  |
| DEC-015 | Reconnect loop with force-disable on failure          |
| DEC-016 | `__slots__` on hot-path objects                       |
| DEC-017 | Pre-allocated buffers in servo loop                   |
| DEC-018 | ROS 2 QoS left at defaults                            |
| DEC-019 | Isaac Sim as physics twin only                        |
| DEC-020 | No PubSub in this version                             |

### Known Limitations (v1.0)

| ID   | Limitation                                              |
|------|---------------------------------------------------------|
| L-01 | TCP/JSON instead of OPC UA                              |
| L-02 | joint_5 not controlled (only 4 axes from PLC)           |
| L-03 | Isaac physics runs at 25–60 Hz (target 60)              |
| L-04 | Bridge polls at 20 Hz (50 ms latency floor)             |
| L-05 | Linux process not real-time (no PREEMPT_RT)             |
| L-06 | No Cartesian motion or inverse kinematics in KRC twin   |
| L-07 | Single trajectory at a time (no queue, no blending)     |
| L-08 | No authentication on TCP protocol                       |

## [0.1.0] — 2026-09-10

### Added (Initial Prototype)

- First working proof-of-concept for a KUKA KR210 digital twin.
- Single monolithic Python file (`plc_ros_bridge4.py`) — merged bridge + KRC twin.
- Basic TCP/JSON transport between PLCSIM Advanced (Windows) and Linux.
- Simple joint-space command forwarding (degrees <-> radians).
- Verified:
  - PLC heartbeat protocol works end-to-end
  - ROS 2 topics publish/subscribe correctly
  - Isaac Sim receives joint commands
  - Feedback returns to PLC without corruption

### Notes

- Superseded by v0.2.0 (Yaskawa migration).
- Preserved as a historical reference for the initial architecture.
- The monolithic file was later refactored in v2.0.0 into six modules.

---

## Summary of Generations

| Version    | Robot            | Transport | Architecture         | Location                      |
|------------|------------------|-----------|----------------------|-------------------------------|
| v0.1.0     | KUKA KR210       | TCP/JSON  | Monolithic           | (superseded)                  |
| v1.0.0     | KUKA KR210       | TCP/JSON  | 5 separate processes | `archive/v1.0-kuka/`          |
| v0.2.0     | Yaskawa GP110    | OPC UA    | Monolithic bridge    | `archive/v0.2-yaskawa/`       |
| v0.2.1     | Yaskawa GP110    | OPC UA    | Monolithic bridge    | (URDF modification)           |
| v0.2.2     | Yaskawa GP110    | OPC UA    | Monolithic bridge    | (Hidden A5 diagnosis)         |
| v2.0.0     | Yaskawa GP110    | OPC UA    | Modular (6 files)    | `src/plc_bridge/`             |

---

## Versioning Policy

This project uses [Semantic Versioning](https://semver.org/):

- **MAJOR** (X.0.0) — incompatible API or architecture changes
- **MINOR** (0.X.0) — new features, backward compatible
- **PATCH** (0.0.X) — bug fixes, backward compatible

---

## Links

- **Repository**: https://github.com/Nebras4u/isaac-plc-digital-twin-v2
- **Full documentation**: [`docs/README_v2_full.md`](docs/README_v2_full.md)
- **KUKA v1.0 archive**: [`archive/v1.0-kuka/`](archive/v1.0-kuka/)
- **Yaskawa v0.2 archive**: [`archive/v0.2-yaskawa/`](archive/v0.2-yaskawa/)
- **Keep a Changelog**: https://keepachangelog.com/
- **Semantic Versioning**: https://semver.org/

---

*Maintained by [Nebras](https://github.com/Nebras4u) — nebras4u@gmail.com*
