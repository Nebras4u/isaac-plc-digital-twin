# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

_Nothing yet._

---

## [3.7.0] — 2026-10-28 — Probe Observability + Production Path

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Added

- **`_LatencyProbe.probe_attempted` counter (`plc_ros_bridge.py`).**
  Every call to `record()` increments `attempted` before validation.
  This distinguishes "bridge didn't publish a tag" (`probe_sent == 0`)
  from "tag arrived but was rejected" (`probe_attempted > probe_rows`).
  In production with `cmd_ang: [cmd_ang]`, we observed
  `probe_attempted == probe_rows == 2691` (100% capture) over a 60 s
  motion run with `probe_rejected == 0`.

- **`joint_mapper.py` — standalone module.**
  `JointMapper` was extracted from `physics_twin.py` so it can be
  imported and tested outside Isaac Sim. `physics_twin.py` now does
  `from joint_mapper import JointMapper` with a `sys.path` bootstrap
  that adds `/workspace` when running inside Isaac Sim's Script Editor.

- **`test_physics_twin.py` — 4 pytest tests.**
  Covers `JointMapper` identity, reordering (`joint_4`/`joint_5` swap
  in `all_joints`), mismatch rejection, and roundtrip conversion.

- **`test_trajectory_twin.py` expanded from 7 to 25 tests.**
  New coverage:
  - Bug A regression: passive axis not interpolated in `_publish`
  - Bug B regression: `set_target` called on new command
  - Bug D regression: `STOPPING→IDLE` clears `_last_applied_cmd_deg`
  - State machine transitions (`bad command`, `e-stop`, `cmd timeout`)
  - `_publish` tag consumption (one-shot)
  - `set_target` rejects non-finite values
  - `sync_state` zeros velocity on jump
  - Per-axis array validation

- **`analyze_probe.py` — new options (backward compatible).**
  `--plot`, `--poll-hz HZ`, `--export-clean PATH`, `--window N`,
  `--bins N`. The default invocation is identical to v3.1.3.

- **`clean_run.sh` — helper script.**
  Commands: `start`, `motion`, `reset`, `verify`, `stop`.
  Uses `taskset` for CPU pinning (bridge on 8,9; traj on 10).

### Changed

- **`plc_ros_bridge.py`: same-tick `act_ang` write (v3.6 carryover).**
  `act_ang`, `cmd_ack`, and `feedback_valid` are prepared and sent in
  the same RTT as the read. `hb_ack` remains pipelined.
  **Result:** `write_net` p50 −24% (6.04 → 4.61 ms).

- **`trajectory_twin.py`: `_cmd_tag` consumed only by `_publish`**
  (v3.6 carryover). `_cmd_tag_t` tracks arrival time so a newer tag is
  not clobbered.

- **`physics_twin.py`: `_applied_tag` survives multiple publishes**
  (v3.6 carryover). Kept while fresh (`≤ TAG_VALID_S = 0.5s`).
  **Result:** 7-part `frame_id` in >90% of messages (was ~50%).

- **`physics_twin.py`: `DEFAULT_PUBLISH_HZ = 120.0` (was 60.0).**
  Aligns with traj's 120 Hz control loop.

- **`plc_ros_bridge.yaml`: `poll_hz: 200` (was 120 in v3.1.3).**
  With same-tick write + TCP tuning, 200 Hz is safe on the reference
  VM. `queue_delay` drops to ~3 ms (from ~5.87 ms at 120 Hz).

### Fixed

- **`plc_ros_bridge.py` — `_js_tag` starvation when `needs_write=False`.**
  Previously every tick with a fresh `/joint_states` cleared the tag,
  even if `act_ang` was skipped by the deadband. This made ~92% of
  probe tags vanish before they could be recorded (observed:
  `probe_rows=362/4800 = 7.5%`). Now the tag survives until an actual
  `act_ang` write occurs. **Result:** `probe_rows/probe_attempted`
  rose from 7.5% to **100%**.

- **`plc_ros_bridge.py` — `write_failed` on `CmdAck` / `ActAng`.**
  Same-tick write reduces batch-write contention with `hb_ack`.
  `write_failed` in the log is now zero over full 60 s runs.

- **`trajectory_twin.py` — `np.interp` on passive axes.**
  `_publish` no longer interpolates `joint_5` from `joint_3`/`joint_4`.
  Passive axes retain their last published value.

- **`trajectory_twin.py` — `set_target` never called after `reset()`.**
  `_last_cmd_deg` was set by `on_cmd` **before** `servo_tick`
  compared against it. Fixed by splitting `_last_received_cmd_deg`
  (log dedupe) from `_last_applied_cmd_deg` (target dedupe), updated
  only inside `servo_tick` after `set_target()` succeeds, cleared on
  `STOPPING→IDLE`.

- **`trajectory_twin.py` — `RuckigPlanner` parameter validation.**
  `max_vel`, `max_acc`, `max_jerk`, `jump_threshold`,
  `brake_acc_mult`, `brake_jerk_mult` validated at construction
  (per-axis > 0, length == dof).

- **`physics_twin.py` — `ModuleNotFoundError: joint_mapper` inside
  Isaac Sim.** `sys.path` bootstrap adds `_SCRIPT_DIR` and
  `/workspace` before the import.

### Performance (Docker/VM, 60 s motion, `cmd_ang: [cmd_ang]`)

Environment: Ubuntu 24.04 (Docker), RTX 5060, shared vCPU, PLCSIM
Advanced V6.0, FastDDS, 24 cores with `taskset` pinning.

**1663 rows, span 357 s (includes prior sessions), 2691 records
from current 60 s motion:**

| Metric                | p50          | p95          | p99          |
| :-------------------- | -----------: | -----------: | -----------: |
| `queue_delay`         | 3.08 ms      | 3.53 ms      | 3.61 ms      |
| `read_net`            | 1.72 ms      | 3.90 ms      | 17.11 ms     |
| `convert_pub`         | 0.05 ms      | 0.08 ms      | 0.11 ms      |
| `ros_ruckig`          | 3.08 ms      | 6.46 ms      | 8.31 ms      |
| `isaac_step`          | 5.15 ms      | 8.39 ms      | 9.50 ms      |
| `ros_back`            | 0.29 ms      | 0.44 ms      | 0.65 ms      |
| `write_net`           | 4.73 ms      | 13.42 ms     | 40.48 ms     |
| **`cmd_to_feedback`** | **14.48 ms** | **23.38 ms** | **45.91 ms** |
| **`total_loop`**      | **19.04 ms** | **29.25 ms** | **60.88 ms** |

**Latency Bound Probability (`cmd_to_feedback`):**

| Threshold   | LBP        |
| :---------- | ---------: |
| ≤ 10 ms     | 21.96%     |
| ≤ 15 ms     | 60.47%     |
| **≤ 20 ms** | **90.37%** |
| ≤ 30 ms     | 97.29%     |
| ≤ 40 ms     | 98.44%     |
| ≤ 50 ms     | 99.10%     |

**Probe health:** `probe_attempted == probe_rows == 2691`,
`probe_rejected == 0`, `probe_order_violations == 0`,
`write_failed == 0`.

### Comparison (Docker/VM, same environment)

| Metric (p50)      | v3.1.3 (VM) | **v3.7 (VM)** | Δ         |
| :---------------- | ----------: | ------------: | --------: |
| `queue_delay`     | 5.87 ms     | **3.08 ms**   | −48%      |
| `read_net`        | 1.36 ms     | 1.72 ms       | +26%      |
| `ros_ruckig`      | 4.32 ms     | **3.08 ms**   | −29%      |
| `isaac_step`      | 9.71 ms     | **5.15 ms**   | −47%      |
| `write_net`       | 5.28 ms     | **4.73 ms**   | −10%      |
| `cmd_to_feedback` | 18.55 ms    | **14.48 ms**  | **−22%**  |
| `total_loop`      | 25.72 ms    | **19.04 ms**  | **−26%**  |
| LBP ≤ 20 ms       | 59.17%      | **90.37%**    | **+31pp** |
| LBP ≤ 30 ms       | 96.33%      | **97.29%**    | +1pp      |

The reference machine (native Ubuntu) has **not been re-measured**
for v3.7.

### Known Limitations (added in v3.7)

| ID | Limitation |
| :--- | :--- |
| VL-20 | `probe_sent = 0` when `cmd_ang: []` in YAML. `probe_attempted` covers external tag sources. |
| VL-21 | `write_failed` on `CmdAck` if the PLC rejects the write. Verify `CmdEnable=TRUE` and `Emergency_Stop_Active=FALSE`. |
| VL-22 | `write_net` p99 remains ~40 ms on the VM (TCP tail on libvirt). Reduce with a direct network cable, `chrt -f 50`, or `poll_hz: 150`. |

### Migration Notes

- **No config schema change** (still v2.1).
- **No ROS graph change.**
- **`analyze_probe.py`** — existing invocations still work.
- **`joint_mapper.py`** — must be placed alongside `physics_twin.py`
  (or `/workspace` for Isaac Script Editor).
- **`poll_hz: 200`** requires the PLC server to handle 200 requests/s
  with `PLC_BATCH=1`.
- **`DEFAULT_PUBLISH_HZ = 120.0`** — revert to 60.0 if `isaac_step`
  jitter is a problem.

---

## [3.6.0] — 2026-10-28 — Latency-Probe Chain Fixes

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Fixed

- **`plc_ros_bridge.py` — `_js_tag` starvation when `needs_write=False`.**
  Full fix. See v3.7 for impact.

- **`trajectory_twin.py` — `_cmd_tag` consumed only by `_publish`.**
  Added `_cmd_tag_t` (monotonic timestamp).

- **`physics_twin.py` — `_applied_tag` survives multiple publishes.**
  Kept while fresh (`≤ TAG_VALID_S = 0.5s`).

### Changed

- **`plc_ros_bridge.py` — same-tick `act_ang` write.** `act_ang`,
  `cmd_ack`, `feedback_valid` in the same RTT as the read.
  **Result:** `write_net` p50 −24%.

### Migration Notes

Both `/servo_setpoints` and `/joint_states` now carry 6-part and
7-part `frame_id`s respectively.

---

## [3.5.0] — 2026-10-28 — Trajectory Twin Fixes

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Fixed

- **`trajectory_twin.py` — `RuckigPlanner` validates all parameters.**
- **`trajectory_twin.py` — `needs_target_update` compares against
  *applied*, not *received* command.**
- **`trajectory_twin.py` — `_publish` no longer uses `np.interp`.**
  Passive axes retain their last published value.

### Added

- **`test_trajectory_twin.py`** — initial expansion (later grew to 25
  tests in v3.7).

---

## [3.1.3] — 2026-10-26 — PLC Batch Mode + 120 Hz Tuning

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Added

- **`windows/plc_server_win.py` — batch PLC I/O enabled by default.**
  Uses `SDataValueByName` + `ReadSignals`/`WriteSignals` to read/write
  all tags in a single PLCSIM API call instead of one call per tag.
  Controlled by the environment variable `PLC_BATCH`:
  - `PLC_BATCH=1` (default) — batch mode
  - `PLC_BATCH=0` — per-tag mode (legacy)
  A startup self-test compares batch results with per-tag reads and
  falls back automatically if they disagree.

  **Fixed:** `EPrimitiveDataType` members are .NET primitive names
  (`Double`, `Int32`, `Int16`, `Bool`), **not** Siemens names (`LReal`,
  `DInt`, `Int`). The previous `_TYPE_NAMES` mapping failed its
  self-test and silently fell back to per-tag calls. Batch mode was
  never actually enabled until this fix. `_TYPE_NAMES` now maps:
  `LReal→Double`, `DInt→Int32`, `Int→Int16`, `Bool→Bool`.

- **`PLC_INSTANCE` environment variable** — overrides the default
  instance name (`"296"`) without editing the file.

- **`ros2/analyze_probe.py`** — offline CSV analysis tool. Reads
  `latency_probe.csv` and produces p50/p95/p99, LBP table, and a
  worst-5-rows table.

- **`ros2/test_probe_chain.py`** — 10 standalone tests for the
  latency-probe tag chain.

- **`windows/Run.txt`** — quick-start usage notes for the Windows
  server.

### Changed

- **`config/plc_ros_bridge.yaml`: `poll_hz: 60 → 120`.**
  At 120 Hz, `queue_delay` drops from 13.48 ms to 5.87 ms.

- **`PLC_TIMING` default is now `"1"`** in `windows/plc_server_win.py`.

### Performance (Docker/VM — **not** the reference machine)

| Metric (p50)      | batch OFF, 60 Hz | batch ON, 60 Hz | batch ON, 120 Hz |
| :---------------- | ---------------: | --------------: | ---------------: |
| `read_net`        | 6.04 ms          | 1.67 ms         | 1.36 ms          |
| `write_net`       | 13.93 ms         | 12.79 ms        | 5.28 ms          |
| `queue_delay`     | 6.33 ms          | 13.48 ms        | 5.87 ms          |
| `cmd_to_feedback` | 34.60 ms         | 34.34 ms        | **18.55 ms**     |
| `total_loop`      | 45.95 ms         | 49.69 ms        | **25.72 ms**     |

### Fixed

- **`windows/plc_server_win.py` batch self-test** — `_TYPE_NAMES`
  now correctly maps Siemens type names to `EPrimitiveDataType`
  members.

### Known Limitations (added)

| ID | Limitation |
| :--- | :--- |
| VL-17 | `isaac_step` (~9.71 ms p50) is now the largest single contributor on the Docker/VM. |
| VL-18 | `poll_hz: 120` requires batch mode in the PLC server. |
| VL-19 | The reference machine has **not** been re-measured with v3.1.3. |

### Migration Notes

- No config schema change (still v2.1).
- No ROS graph change.
- To disable batch: `PLC_BATCH=0` in the environment.

---

## [3.1.2] — 2026-10-25 — Performance Fixes for VM Environments

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Changed

- **`plc_ros_bridge.py` — PLC I/O moved to a dedicated thread.**
  **Result:** `ros_back` −92% (4.66 → 0.37 ms), `total_loop` −30%
  (67.36 → 47.20 ms).

- **`plc_ros_bridge.py` — `_last_tick_end_ns` for real queue delay.**
  **Result:** `queue_delay` = 5.42 ms median (was 0.00).

- **`physics_twin.py` — `spin_once` before `step`/`publish_states`.**
  **Result:** `isaac_step` −62% (27.05 → 10.25 ms).

- **`trajectory_twin.py` — `SingleThreadedExecutor` (was `MultiThreaded`).**
  **Note:** net change against v3.1.1 is +24% (`ros_ruckig`), see `VL-16`.

- **QoS — `BEST_EFFORT` + `depth=1` on high-rate streams.**

### Fixed

- **`latency_probe` `origin_ns`** — now reflects the end of the previous
  tick.

### Known Limitations

| ID | Limitation |
| :--- | :--- |
| VL-12 | `write_net` is ~12 ms on the VM. Reduce by switching to `plc_api` driver. |
| VL-13 | `isaac_step` is ~10 ms on the VM. Reduce by enabling GPU passthrough. |
| VL-14 | One 103 s Isaac Sim stall observed. Excluded from filtered statistics. |
| VL-15 | CycloneDDS is not supported on this branch's Isaac Sim setup. |
| VL-16 | `ros_ruckig` median rose from 6.57 ms (v3.1.1) to 8.14 ms (v3.1.2). Cause not verified. |

### Migration Notes

No migration is required. The config schema is unchanged (v2.1).

---

## [3.1.1] — 2026-10-23 — Environment Transparency and Real VM Results

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Changed

- `krc_twin.py` replaced by `trajectory_twin.py`.
- README Quick Start updated.
- Unit tests: 26 → 29.
- Config schema badge: 2.0 → 2.1.

### Fixed

- **`trajectory_twin.py` — brake cancelled by a persistent bad command.**
- **`plc_ros_bridge.py` — e-stop auto-cleared on reconnect.**

### Documentation

- Added an **Environment** section and **Results (this machine)**.
- Documented the `Articulation.initialize()` hang on Isaac Sim 5.1.
- Added a **Safety Notice** to the README.

### Known Limitations

| ID | Limitation |
| :--- | :--- |
| VL-01 | ~~KRC twin reads config inline~~ — resolved by `trajectory_twin.py` |
| VL-06 | Fixes above verified with stubbed ROS 2 / Ruckig only |
| VL-07 | No automated tests yet for the planner state machine |

---

## [3.1.0] — 2026-10-21 — Production-Grade Config

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Added

- **Pydantic v2 schema** (`bridge_config.py`).
- **`schema_version` + migration chain**.
- **Audit trail** — `sha256` hash + ISO-8601 timestamp logged to JSONL.
- **Fail-fast** — bad config rejected **before** any socket is opened.
- **`robot.yaml` validation**.
- **26 unit tests**.
- **`requirements-dev.txt` + `pytest.ini`**.
- **PEP 668 aware**.

### Changed

- `bridge_config.py` is now the canonical schema definition.

---

## [3.0.0] — 2026-10-10 — Vendor-Neutral Platform

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Changed (Breaking)

- **New folder `vendor-neutral/`** — platform decoupled from any
  specific robot or PLC vendor.
- **Repository reorganized by environment.**
- **Transport abstraction** in `linux/plc_drivers.py`.
- **Signal abstraction** in `linux/plc_config.py`.
- **`plc_api` transport added** — direct PLCSIM Advanced API
  (Windows-only).

### Added

- `vendor-neutral/linux/plc_drivers.py`
- `vendor-neutral/linux/plc_config.py`
- `vendor-neutral/config/robot.yaml`
- Reorganized `vendor-neutral/tools/`.

### Preserved

- All previous generations moved to `archive/`.

### Known Limitations (v3.0.0)

| ID    | Limitation                                                        |
| :---- | :---------------------------------------------------------------- |
| VL-01 | KRC twin still reads config inline; `robot.yaml` not consumed yet |
| VL-02 | Yaskawa GP110 end-to-end verification pending                     |
| VL-03 | No Cartesian motion or inverse kinematics layer                   |
| VL-04 | No real-time guarantees (no PREEMPT_RT)                           |
| VL-05 | No authentication on `tcp_json` transport                         |

---

## [2.0.0] — 2026-10-07 — Yaskawa Modular

**Location:** [`yaskawa/`](yaskawa/) — Gen 2

### Changed (Breaking)

- Monolithic bridge split into 6 focused modules under `src/plc_bridge/`:
  `config.py`, `joint_map.py`, `plc_client.py`, `ros_bridge.py`,
  `main.py`, `plc_gui.py`.

### Added

- `plc_gui.py` — standalone tkinter GUI for manual control.
- `docs/README_v2_full.md` — 742-line documentation.

### Fixed

- **Hidden A5 problem** — root cause: `A5 != 0` introduces horizontal
  offset = `LF * sin(A5)`. Solution: `A5 = -(A1 + A4)` for tool-down
  orientation.

---

## [1.0.0] — 2026-09-15 — KUKA KR210 Full Digital Twin

**Location:** [`kuka/`](kuka/) — Gen 1

### Added

- **PLC Layer:** OB30 heartbeat (20 ms), OB1 motion logic, TO_Kinematics.
- **Windows Layer:** `plc_server_win.py` — TCP/JSON bridge over PLCSIM.
- **Linux Layer:** `plc_ros_bridge.py`, `krc_twin.py`, `check_accuracy.py`.
- **Isaac Layer:** `physics_twin.py`, modified URDF, `Full_Control.usd`.
- **Tools:** clock drift measurement.
- **Docs:** complete 18-section project documentation.

### Performance

| Joint   | Setpoint  | Actual    | Error   |
| :------ | :-------- | :-------- | :------ |
| joint_1 | 30.0002°  | 30.0002°  | 0.0001° |
| joint_2 | -29.9603° | -29.9349° | 0.0254° |
| joint_3 | 60.0097°  | 60.0133°  | 0.0036° |
| joint_4 | -0.0025°  | -0.0033°  | 0.0008° |
| joint_5 | -0.4668°  | -0.4668°  | 0.0000° |

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

- First working proof-of-concept for a KUKA KR210 digital twin.
- Single monolithic Python file (`plc_ros_bridge4.py`).
- Basic TCP/JSON transport between PLCSIM Advanced and Linux.

### Notes

- Superseded by 1.0.0 (full KUKA digital twin).

---

## Summary of Generations

| Version    | Robot         | Transport         | Architecture                                                    | Location              |
| :--------- | :------------ | :---------------- | :-------------------------------------------------------------- | :-------------------- |
| v0.1.0     | KUKA KR210    | TCP/JSON          | Monolithic                                                      | (superseded)          |
| v1.0.0     | KUKA KR210    | TCP/JSON          | 5 processes                                                     | `kuka/`               |
| v2.0.0     | Yaskawa GP110 | OPC UA            | Modular (6 files)                                               | `yaskawa/`            |
| **v3.0.0** | **Any robot** | **Any transport** | **Vendor-neutral**                                              | **`vendor-neutral/`** |
| **v3.1.0** | **Any robot** | **Any transport** | **+ Pydantic v2 + 26 tests**                                    | **`vendor-neutral/`** |
| **v3.1.1** | **Any robot** | **Any transport** | **+ environment transparency + 29 tests**                       | **`vendor-neutral/`** |
| **v3.1.2** | **Any robot** | **Any transport** | **+ threaded PLC I/O + BEST_EFFORT QoS**                        | **`vendor-neutral/`** |
| **v3.1.3** | **Any robot** | **Any transport** | **+ batch PLC I/O + poll_hz 120 (18.55 ms)**                    | **`vendor-neutral/`** |
| **v3.5.0** | **Any robot** | **Any transport** | **+ trajectory_twin fixes**                                     | **`vendor-neutral/`** |
| **v3.6.0** | **Any robot** | **Any transport** | **+ probe chain fixes (100% capture)**                          | **`vendor-neutral/`** |
| **v3.7.0** | **Any robot** | **Any transport** | **+ probe_attempted + joint_mapper.py + 51+8 tests + 14.48 ms** | **`vendor-neutral/`** |

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
- **Gen 2 (reference):** [`yaskawa/`](yaskawa/)
- **Gen 1 (reference):** [`kuka/`](kuka/)

***

*Maintained by [Nebras](https://github.com/Nebras4u) — nebras4u@gmail.com*
