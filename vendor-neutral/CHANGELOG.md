# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Changed

- `krc_twin.py` replaced by `trajectory_twin.py` — a vendor-neutral Ruckig
  trajectory generator and state machine that reads joints, limits, and
  Ruckig parameters from `config/robot.yaml` (closes **VL-01**).
- README Quick Start updated: `trajectory_twin.py --ros-args -p robot:=...`.

### Fixed

- **`trajectory_twin.py` — brake cancelled by a persistent bad command.**
  With a command that stayed out of limits, the state machine re-entered
  `RUNNING` from `STOPPING` on the next cycle and called `planner.reset()`,
  cancelling the brake profile (abrupt stop) and logging `bad_command` plus
  two `state_changed` lines every cycle. Now `RUNNING` can only be entered
  from `IDLE`, a brake always runs to completion, a bad command in `IDLE`
  keeps the twin idle, and `bad_command` is logged once.
  Stub-based test (200 cycles with a bad command): 200 `bad_command` and
  598 `state_changed` log lines before, 1 `bad_command` and 2
  `state_changed` after.
- **`plc_ros_bridge.py` — e-stop auto-cleared on reconnect.** The bridge
  published `/plc/emergency_stop = False` immediately on reconnect, before
  the real PLC e-stop value was read. When the PLC e-stop signal is
  published (`/plc/emergency_stop` in `publish`), the value now comes from
  the PLC on the next cycle. Configurations without that signal keep the
  previous behavior.

### Documentation

- Added a **Safety Notice** to the README: the e-stop, heartbeat, and
  watchdog logic is functional monitoring for the simulated robot and is
  **not a certified safety function** (IEC 61508 / ISO 13849-1 /
  IEC 62061). Real installations require an independent certified safety
  layer (F-CPU with PROFIsafe, hardware E-Stop).

### Known Limitations

| ID | Limitation |
| :--- | :--- |
| VL-01 | ~~KRC twin reads config inline~~ — resolved by `trajectory_twin.py` |
| VL-06 | Fixes above verified with stubbed ROS 2 / Ruckig only; not yet re-tested on Isaac Sim or PLCSIM Advanced |
| VL-07 | No automated tests yet for the planner state machine (the stub test is the starting point) |

---

## [3.1.2] — 2026-10-25 — Performance Fixes for VM Environments

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Changed

- **`plc_ros_bridge.py` — PLC I/O moved to a dedicated thread.**
  Previously, `tick()` (PLC I/O) and `on_joint_states()` (ROS callback)
  ran on the same `SingleThreadedExecutor`. A slow PLC read (6–7 ms)
  delayed the delivery of `/joint_states`, which inflated `ros_back`
  to ~4.7 ms. Now `tick()` runs in `_io_loop` on a separate thread,
  and `on_joint_states()` is delivered immediately.
  **Result:** `ros_back` −92% (4.66 → 0.37 ms), `total_loop` −30%
  (67.36 → 47.20 ms).

- **`plc_ros_bridge.py` — `_last_tick_end_ns` for real queue delay.**
  Previously, `origin_ns` was a duplicate of `read_start_ns`, so
  `queue_delay` was always 0.00 ms. Now `_last_tick_end_ns` is recorded
  at the end of each `tick()`, and `origin_ns` uses it. The queue delay
  (time between the end of the previous tick and the start of the current
  read) is now measurable.
  **Result:** `queue_delay` = 5.42 ms median (was 0.00).

- **`physics_twin.py` — `spin_once` before `step`/`publish_states`.**
  Previously, `spin_once` ran **after** `step()` and `publish_states()`,
  so a setpoint that arrived since the last frame waited one extra frame
  before being applied. Now `spin_once` runs first, and the setpoint is
  applied in the same frame.
  **Result:** `isaac_step` −62% (27.05 → 10.25 ms).

- **`trajectory_twin.py` — `SingleThreadedExecutor` (was `MultiThreaded`).**
  MultiThread added overhead on `ros_ruckig` (+26%) without improving
  `isaac_step`. Reverted to `SingleThreadedExecutor`.
  **Result:** `ros_ruckig` 8.14 ms (was 10.99 ms with MultiThread).

- **QoS — `BEST_EFFORT` + `depth=1` on high-rate streams.**
  `/plc/cmd_ang`, `/servo_setpoints`, `/joint_states` now use
  `BEST_EFFORT` with `depth=1`. Old samples are dropped instead of
  queued, which reduces `queue_delay` and `ros_ruckig` on slow consumers.

### Fixed

- **`latency_probe` `origin_ns`** — now reflects the end of the previous
  tick, not a duplicate of `read_start_ns`.

### Known Limitations

| ID | Limitation |
| :--- | :--- |
| VL-12 | `write_net` is ~12 ms on the VM (PLCSIM Advanced API on Windows guest). Reduce by switching to `plc_api` driver (PLCSIM direct) if the bridge runs on Windows. |
| VL-13 | `isaac_step` is ~10 ms on the VM (shared vCPU). Reduce by enabling GPU passthrough or running Isaac Sim on a native host. |
| VL-14 | One 103 s Isaac Sim stall observed (Play/Pause or re-init). Excluded from filtered statistics. |
| VL-15 | CycloneDDS is not supported on this branch's Isaac Sim setup (Python 3.11 vs 3.12 mismatch). Use FastDDS on all nodes. |

### Migration Notes

No migration is required. The config schema is unchanged (v2.1). The
only behavioral change is the QoS downgrade on high-rate streams, which
is transparent to existing subscribers (RELIABLE publishers remain
compatible with BEST_EFFORT subscribers).

---

## [3.1.0] — 2026-10-21 — Production-Grade Config

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Added

- **Pydantic v2 schema** (`bridge_config.py`) — replaces the hand-written
  `_validate_config`. Declarative, `extra="forbid"`, field ranges via
  `Field(gt=, le=)`, cross-reference validation with **field location**
  (`robot.cmd_ang[0]`), direction sanity, address safety (rejects
  `\x00`, `\n`, `\r`, `..`).
- **`schema_version` + migration chain** — v1.0 configs work unchanged.
- **Audit trail** — `sha256` hash + ISO-8601 timestamp logged to JSONL
  on every startup.
- **Fail-fast** — bad config rejected **before** any socket is opened.
- **`robot.yaml` validation** — `_validate_limits`, Ruckig parameter
  sanity.
- **26 unit tests** (`pytest tests/ -v` → 26 passed in 0.17 s):
  19 for config schema (`test_bridge_config.py`), 7 for trajectory
  planner (`test_trajectory_twin.py`).
- **`requirements-dev.txt` + `pytest.ini`** — repeatable test setup.
- **PEP 668 aware** — documented venv workflow for Python 3.11+.

### Changed

- `bridge_config.py` is now the canonical schema definition; the previous
  hand-written validator is removed.

---

## [3.0.0] — 2026-10-10 — Vendor-Neutral Platform

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Changed (Breaking)

- **New folder `vendor-neutral/`** — the platform is now decoupled from any
  specific robot or PLC vendor.
- **Repository reorganized by environment:** `windows/`, `linux/`, `isaac/`,
  `config/`, `tools/`, `plc/`, `docs/`.
- **Transport abstraction:** all three transports (`tcp_json`, `opcua`,
  `plc_api`) live in `linux/plc_drivers.py` behind a single `PlcDriver`
  interface.
- **Signal abstraction:** address mapping + value extraction moved to
  `linux/plc_config.py`.
- **`plc_api` transport added** — direct PLCSIM Advanced API (Windows-only),
  selectable via `driver: plc_api` in `plc_ros_bridge.yaml`.

### Added

- `vendor-neutral/linux/plc_drivers.py` — `PlcDriver` ABC + 3 implementations
- `vendor-neutral/linux/plc_config.py` — `make_addr` + `SignalExtractor`
- `vendor-neutral/config/robot.yaml` — placeholder for future migration of
  the KRC twin's inline config
- Reorganized `vendor-neutral/tools/` (latency, clock drift)

### Preserved

- All previous generations moved to `archive/`:
  - `archive/v1.0-kuka/` — KUKA KR210 (TCP/JSON)
  - `archive/v0.2-yaskawa/` — Yaskawa GP110 (OPC UA)

### Migration Notes

| Old (v2.0.0) | New (v3.0.0) |
| :--- | :--- |
| `linux/plc_ros_bridge.py` (inline drivers) | `vendor-neutral/linux/plc_drivers.py` + `vendor-neutral/linux/plc_ros_bridge.py` |
| `linux/plc_config` (implicit) | `vendor-neutral/linux/plc_config.py` |
| flat repository | folder-by-environment inside `vendor-neutral/` |

### Known Limitations (v3.0.0)

| ID | Limitation |
| :--- | :--- |
| VL-01 | KRC twin still reads config inline; `robot.yaml` is not consumed yet |
| VL-02 | Yaskawa GP110 end-to-end verification on this branch is pending |
| VL-03 | No Cartesian motion or inverse kinematics layer |
| VL-04 | No real-time guarantees (no PREEMPT_RT) |
| VL-05 | No authentication on `tcp_json` transport |

---

## [2.0.0] — 2026-10-07 — Yaskawa Modular

**Location:** [`archive/v0.2-yaskawa/`](archive/v0.2-yaskawa/) — Gen 2

### Changed (Breaking)

- Monolithic bridge split into 6 focused modules under `src/plc_bridge/`:
  - `config.py`, `joint_map.py`, `plc_client.py`, `ros_bridge.py`,
    `main.py`, `plc_gui.py`
- Historical versions moved to `archive/`

### Added

- `plc_gui.py` — standalone tkinter GUI for manual control
- `docs/README_v2_full.md` — 742-line documentation with Appendices A–F

### Fixed

- **Hidden A5 problem** — root cause: `A5 != 0` introduces horizontal offset
  = `LF * sin(A5)`. Mathematical verification: 4.66 mm (theory) vs 4.68 mm
  (measured). Solution: `A5 = -(A1 + A4)` for tool-down orientation.

---

## [1.0.0] — 2026-09-15 — KUKA KR210 Full Digital Twin

**Location:** [`archive/v1.0-kuka/`](archive/v1.0-kuka/) — Gen 1

### Added

- **PLC Layer:** OB30 heartbeat (20 ms), OB1 motion logic (MC_PickUp,
  MC_Stkng_L/R, MC_Go2HomePos), TO_Kinematics
- **Windows Layer:** `plc_server_win.py` — TCP/JSON bridge over PLCSIM
- **Linux Layer:** `plc_ros_bridge.py`, `krc_twin.py`, `check_accuracy.py`
- **Isaac Layer:** `physics_twin.py`, modified URDF, `Full_Control.usd`
- **Tools:** clock drift measurement (`srv.py`, `cli.py`)
- **Docs:** complete 18-section project documentation

### Performance

| Joint | Setpoint | Actual | Error |
| :--- | :--- | :--- | :--- |
| joint_1 | 30.0002° | 30.0002° | 0.0001° |
| joint_2 | -29.9603° | -29.9349° | 0.0254° |
| joint_3 | 60.0097° | 60.0133° | 0.0036° |
| joint_4 | -0.0025° | -0.0033° | 0.0008° |
| joint_5 | -0.4668° | -0.4668° | 0.0000° |

Maximum error: **0.0254°** (joint_2, full gravity load).

### Clock Drift (15 min)

- Drift: **-0.065 ppm**
- Standard Error: 0.00016 ppm
- Asymmetry Bound: ±16.0 µs
- RTT min: 32.0 µs

---

## [0.1.0] — 2026-09-10 — Initial Prototype

**Location:** archived

### Added

- First working proof-of-concept for a KUKA KR210 digital twin
- Single monolithic Python file (`plc_ros_bridge4.py`)
- Basic TCP/JSON transport between PLCSIM Advanced and Linux

### Notes

- Superseded by v0.2.0 (Yaskawa migration)
- Preserved as historical reference

---

## Summary of Generations

| Version | Robot | Transport | Architecture | Location |
| :--- | :--- | :--- | :--- | :--- |
| v0.1.0 | KUKA KR210 | TCP/JSON | Monolithic | (superseded) |
| v1.0.0 | KUKA KR210 | TCP/JSON | 5 processes | `archive/v1.0-kuka/` |
| v0.2.0 | Yaskawa GP110 | OPC UA | Modular (6 files) | `archive/v0.2-yaskawa/` |
| **v3.0.0** | **Any robot** | **Any transport** | **Vendor-neutral** | **`vendor-neutral/`** |
| **v3.1.0** | **Any robot** | **Any transport** | **Vendor-neutral + Pydantic v2 + 26 tests** | **`vendor-neutral/`** |
| **v3.1.1** | **Any robot** | **Any transport** | **+ Environment transparency + real VM results + 29 tests** | **`vendor-neutral/`** |

---

## Versioning Policy

This project uses [Semantic Versioning](https://semver.org/):

- **MAJOR** (X.0.0) — incompatible API or architecture changes
- **MINOR** (0.X.0) — new features, backward compatible
- **PATCH** (0.0.X) — bug fixes, backward compatible

---

## Links

- **Repository:** https://github.com/Nebras4u/isaac-plc-digital-twin
- **Gen 3 (active):** [`vendor-neutral/`](vendor-neutral/)
- **Gen 2 (reference):** [`archive/v0.2-yaskawa/`](archive/v0.2-yaskawa/)
- **Gen 1 (reference):** [`archive/v1.0-kuka/`](archive/v1.0-kuka/)

***

*Maintained by [Nebras](https://github.com/Nebras4u) — nebras4u@gmail.com*
