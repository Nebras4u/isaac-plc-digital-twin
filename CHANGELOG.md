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

## [3.1.1] — 2026-10-25 — Environment Transparency and Real VM Results

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Added

- **`Environment` section in README** — explains that the reference
  latency numbers (5.2 ms mean, 16.3 ms max) were measured on the
  author's native Ubuntu 24.04 machine and are **not reproducible on a
  VM or Docker environment**. Provides a table of the reference machine's
  characteristics and instructions for measuring on the reader's hardware.
- **`Results (this machine)` section in README** — 1,902 samples from a
  Docker / RTX 5060 / shared-vCPU environment with PLCSIM Advanced over
  TCP/JSON. Median full-loop latency **67.36 ms**, p95 **105.10 ms**,
  p99 **203.80 ms**. Filtered (top 1% removed): median **67.28 ms**,
  p95 **100.10 ms**, max **203.80 ms**.
- **`Articulation.initialize()` hang** documented in *Known Limitations*
  — on Isaac Sim 5.1, the call may hang if Play has not been pressed at
  least once. Workaround: press Play and re-run `physics_twin.py`.
- **ROS 2 Bridge environment** documented in *Quick Start* — Isaac Sim
  5.1 ships its own Python 3.11 and its own ROS 2 libraries. Users must
  set `ROS_DISTRO`, `RMW_IMPLEMENTATION`, and `LD_LIBRARY_PATH` before
  starting Isaac Sim, and must **not** `source /opt/ros/jazzy/setup.bash`
  (which would point Isaac Sim to the system `rclpy`, Python 3.12, and
  cause an ABI mismatch).
- **Single 6 s link-drop outlier** documented — one sample in the VM run
  produced a 9514 ms full-loop latency (a 6 s PLC link drop). Excluded
  from the filtered statistics; tracked as VL-08.
- **`latency_probe` `origin_ns` fix** documented in the *Roadmap* — the
  column now reflects the end of the previous tick (so queue delay is
  measurable), instead of being a duplicate of `read_start_ns`.

### Changed

- **Tests badge: 26 → 29.** Three new tests cover the E-stop forwarding
  rule (`test_estop_forwarding_required`, `test_estop_forwarding_optout`,
  `test_estop_forwarding_present`).
- **Schema badge: 2.0 → 2.1.** The config schema now enforces E-stop
  forwarding (opt-out for tests via `estop_forwarding_required: false`).
- **Latency badges renamed** to "Reference mean latency" and
  "Reference max latency" to make it explicit that the numbers depend on
  the environment.
- **`Supported Combinations`** table now labels reference numbers with
  `(ref)` to avoid implying they are universal.

### Fixed

- **README badge list** — no functional change; just clarifying labels
  for the reference numbers.

### Known Limitations

| ID | Limitation |
| :--- | :--- |
| VL-08 | VM environment produces a 9514 ms outlier (6 s PLC link drop) — a single sample, excluded from the filtered statistics |
| VL-09 | `isaac_step` on the VM environment is ~27 ms (vs 3–13 ms on the reference machine) — tied to shared CPU and Docker overhead |
| VL-10 | `write_net` on the VM environment is ~16 ms (vs 4.5 ms on the reference machine) — tied to the Docker-mediated PLC server |
| VL-11 | CycloneDDS is **not** supported on this branch's Isaac Sim setup, because Isaac Sim 5.1 uses its internal Python 3.11 ROS 2 libraries while `plc_ros_bridge` and `trajectory_twin` use system Python 3.12 |

### Migration Notes

No migration is required. The config schema version bumps from `2.0` to
`2.1`, but v2.0 configs are still accepted and migrated in-memory. The
only behavioral change is the E-stop forwarding rule, which is enforced
unless `estop_forwarding_required: false` is set (intended for tests).

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
