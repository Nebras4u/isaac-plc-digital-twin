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
  that adds `/workspace` when running inside Isaac Sim's Script Editor
  (which copies the script to `/tmp`).

- **`test_physics_twin.py` — 4 pytest tests.**
  Covers `JointMapper` identity, reordering (`joint_4`/`joint_5`
  swap in `all_joints`), mismatch rejection, and roundtrip conversion.

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

- **`analyze_probe.py` — new options.**
  - `--plot` — generate matplotlib curves (time series, histogram, ECDF)
  - `--poll-hz HZ` — warn if `write_net` p50 exceeds `1.8 × poll_period`
  - `--export-clean PATH` — normalized CSV with `time_s + stage_ms` columns
  - `--window N` — rolling window for smoothing (default 50)
  - `--bins N` — histogram bins (default 60)
  These options are backward compatible; the default invocation is
  identical to v3.1.3.

- **`clean_run.sh` — helper script.**
  Commands: `start`, `motion`, `reset`, `verify`, `stop`. Uses
  `taskset` for CPU pinning (bridge on cores 8,9; traj on 10).

### Changed

- **`plc_ros_bridge.py`: same-tick `act_ang` write (v3.6 carryover).**
  `act_ang`, `cmd_ack`, and `feedback_valid` are prepared and sent in
  the same RTT as the read. `hb_ack` remains pipelined.
  **Result:** `write_net` p50 −24% (6.04 → 4.61 ms baseline → v3.6).

- **`trajectory_twin.py`: `_cmd_tag` is not consumed in `servo_tick`
  (v3.6 carryover).** Consumed only by `_publish()` after the tag has
  been attached to an outgoing message. `_cmd_tag_t` tracks arrival
  time so a newer tag is not clobbered.

- **`physics_twin.py`: `_applied_tag` survives multiple publishes
  (v3.6 carryover).** The tag is kept while fresh (`≤ TAG_VALID_S`),
  so multiple `/joint_states` messages carry the same tag chain.
  **Result:** 7-part `frame_id` in >90% of messages (was ~50%).

- **`physics_twin.py`: `DEFAULT_PUBLISH_HZ = 120.0` (was 60.0).**
  Aligns with traj's 120 Hz control loop. **Result:** `isaac_step`
  captures shorter intervals; `cmd_to_feedback` p50 improved.

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
  Root causes identified and fixed:
  - Same-tick write means `act_ang` is no longer deferred by a tick,
    reducing batch-write contention with `hb_ack`.
  - `write_failed` in the log is now zero over full 60 s runs.
  If residual `write_failed` appears, verify PLC safety state
  (`CmdEnable=TRUE`, `Emergency_Stop_Active=FALSE`); see `VL-21`.

- **`trajectory_twin.py` — `np.interp` on passive axes.**
  Previously `_publish` used `np.interp` on the full joint vector,
  which filled `joint_5` (passive) by interpolating between `joint_3`
  and `joint_4`. This produced spurious motion on axes that were
  supposed to be idle. Now only controlled indices are updated;
  passive axes retain their last published value.

- **`trajectory_twin.py` — `set_target` never called after `reset()`.**
  `_last_cmd_deg` was set by `on_cmd` **before** `servo_tick`
  compared against it, so after `planner.reset()` no command ever
  triggered a fresh `set_target()`. The robot would not move on the
  first command after each state transition. Fixed by separating
  `_last_received_cmd_deg` (log dedupe) from `_last_applied_cmd_deg`
  (target dedupe). The applied variable is updated only inside
  `servo_tick` after `set_target()` succeeds, and cleared on
  `STOPPING→IDLE`.

- **`trajectory_twin.py` — `RuckigPlanner` parameter validation.**
  `max_vel`, `max_acc`, `max_jerk`, `jump_threshold`,
  `brake_acc_mult`, and `brake_jerk_mult` are validated at
  construction (per-axis > 0, length == dof). Typos in `robot.yaml`
  now fail loudly instead of producing silent no-motion.

- **`physics_twin.py` — `ModuleNotFoundError: joint_mapper` inside
  Isaac Sim.** Isaac's Script Editor copies the script to `/tmp`, so
  `__file__` does not point to `/workspace`. A `sys.path` bootstrap
  now adds both `_SCRIPT_DIR` and `/workspace` before the import.

### Performance (Docker/VM, 60 s motion, `cmd_ang: [cmd_ang]`)

Environment: Ubuntu 24.04 (Docker), RTX 5060, shared vCPU, PLCSIM
Advanced V6.0, FastDDS, 24 cores with `taskset` pinning.

**Run 1 — 1663 rows, span 357 s (includes prior sessions):**

| Metric             | p50      | p95      | p99      | p99.9    |
| :----------------- | -------: | -------: | -------: | -------: |
| `queue_delay`      | 3.08 ms  | 3.53 ms  | 3.61 ms  | 4.08 ms  |
| `read_net`         | 1.72 ms  | 3.90 ms  | 17.11 ms | 46.83 ms |
| `convert_pub`      | 0.05 ms  | 0.08 ms  | 0.11 ms  | 0.14 ms  |
| `ros_ruckig`       | 3.08 ms  | 6.46 ms  | 8.31 ms  | 9.06 ms  |
| `isaac_step`       | 5.15 ms  | 8.39 ms  | 9.50 ms  | 11.05 ms |
| `ros_back`         | 0.29 ms  | 0.44 ms  | 0.65 ms  | 3.12 ms  |
| `write_net`        | 4.73 ms  | 13.42 ms | 40.48 ms | 75.46 ms |
| **`cmd_to_feedback`** | **14.48 ms** | **23.38 ms** | **45.91 ms** | **86.45 ms** |
| **`total_loop`**   | **19.04 ms** | **29.25 ms** | **60.88 ms** | **89.61 ms** |

**Latency Bound Probability (`cmd_to_feedback`):**

| Threshold | LBP      |
| :-------- | -------: |
| ≤ 10 ms   | 21.96%   |
| ≤ 15 ms   | 60.47%   |
| **≤ 20 ms** | **90.37%** |
| ≤ 30 ms   | 97.29%   |
| ≤ 40 ms   | 98.44%   |
| ≤ 50 ms   | 99.10%   |

**Probe health:** `probe_attempted == probe_rows == 2691`,
`probe_rejected == 0`, `probe_order_violations == 0`,
`write_failed == 0`.

### Comparison (Docker/VM, same environment)

| Metric (p50)      | v3.1.3 (VM) | **v3.7 (VM)** | Δ |
| :---------------- | ----------: | ------------: | -: |
| `queue_delay`     | 5.87 ms     | **3.08 ms**   | −48% |
| `read_net`        | 1.36 ms     | 1.72 ms       | +26% |
| `ros_ruckig`      | 4.32 ms     | **3.08 ms**   | −29% |
| `isaac_step`      | 9.71 ms     | **5.15 ms**   | −47% |
| `write_net`       | 5.28 ms     | **4.73 ms**   | −10% |
| `cmd_to_feedback` | 18.55 ms    | **14.48 ms**  | **−22%** |
| `total_loop`      | 25.72 ms    | **19.04 ms**  | **−26%** |
| LBP ≤ 20 ms       | 59.17%      | **90.37%**    | **+31pp** |
| LBP ≤ 30 ms       | 96.33%      | **97.29%**    | +1pp |

The reference machine (native Ubuntu) has **not been re-measured**
for v3.7. The v3.1.3 reference numbers (5.2 ms mean, 16.3 ms max)
still refer to v3.1.2 and earlier.

### Known Limitations (added in v3.7)

| ID | Limitation |
| :--- | :--- |
| VL-20 | `probe_sent = 0` when `cmd_ang: []` in YAML. The bridge does not publish `/plc/cmd_ang`, so `_probe_sent` stays 0. `probe_attempted` covers this case: it counts every tag that arrives from any source. |
| VL-21 | `write_failed` on `CmdAck` if the PLC rejects the write. Verify PLC safety state (`CmdEnable=TRUE`, `Emergency_Stop_Active=FALSE`). If persistent, run `PLC_BATCH=0` on the server to isolate the failing tag. |
| VL-22 | `write_net` p99 remains ~40 ms on the VM (TCP tail on libvirt). Reduce with a direct network cable, `chrt -f 50 taskset -c 8,9`, or `poll_hz: 150`. |

### Migration Notes

- **No config schema change** (still v2.1).
- **No ROS graph change.**
- **`analyze_probe.py`** — existing invocations still work. New
  options are additive.
- **`joint_mapper.py`** — must be placed alongside `physics_twin.py`
  in the same directory (or `/workspace` for Isaac Script Editor).
  If placed elsewhere, set `sys.path` explicitly.
- **`clean_run.sh`** — optional helper, not required for operation.
- **`poll_hz: 200`** requires the PLC server to handle 200 requests/s.
  With batch mode enabled (`PLC_BATCH=1`), `plc_server_win.py`
  handles this comfortably. Do **not** use 200 Hz with `PLC_BATCH=0`.
- **`DEFAULT_PUBLISH_HZ = 120.0`** in `physics_twin.py` — Isaac must
  be able to sustain 120 Hz publish. On a heavily loaded VM this may
  cause `isaac_step` jitter. Revert to 60.0 if needed.

---

## [3.6.0] — 2026-10-28 — Latency-Probe Chain Fixes

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Fixed

- **`plc_ros_bridge.py` — `_js_tag` starvation when `needs_write=False`.**
  Full fix. See v3.7 for the impact (`probe_rows/probe_attempted`
  from 7.5% to 100%).

- **`trajectory_twin.py` — `_cmd_tag` consumed only by `_publish`.**
  Added `_cmd_tag_t` (monotonic timestamp) so `_publish` can verify
  no newer tag arrived before clearing the old one.

- **`physics_twin.py` — `_applied_tag` survives multiple publishes.**
  Tag kept while fresh (≤ `TAG_VALID_S = 0.5s`), not cleared on first
  publish.

### Changed

- **`plc_ros_bridge.py` — same-tick `act_ang` write.** `act_ang`,
  `cmd_ack`, `feedback_valid` are prepared and sent in the same RTT
  as the read. `hb_ack` remains pipelined. **Result:** `write_net`
  p50 −24%.

### Migration Notes

- No config schema change. Both `/servo_setpoints` and `/joint_states`
  now carry 6-part and 7-part `frame_id`s respectively. Probe tools
  that expected 7 parts should parse `/servo_setpoints` as 6 parts.

---

## [3.5.0] — 2026-10-28 — Trajectory Twin Fixes

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Fixed

- **`trajectory_twin.py` — `RuckigPlanner` validates all parameters.**
  See v3.7 for the full fix description.

- **`trajectory_twin.py` — `needs_target_update` compares against
  *applied*, not *received* command.** See v3.7 for the full fix
  (the split landed in v3.5; v3.6 added the tag tracking).

- **`trajectory_twin.py` — `_publish` no longer uses `np.interp`.**
  Passive axes (e.g. `joint_5`) retain their last published value.

### Added

- **`test_trajectory_twin.py`** — initial expansion (later grew to 25
  tests in v3.7).

---

## [3.1.3] — 2026-10-26 — PLC Batch Mode + 120 Hz Tuning

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Added

- **`windows/plc_server_win.py` — batch PLC I/O (default ON).**
  Uses `SDataValueByName` + `ReadSignals`/`WriteSignals` to read/write
  all tags in a single PLCSIM API call instead of one call per tag.
  Controlled by the environment variable `PLC_BATCH`:
  - `PLC_BATCH=1` (default) — batch mode
  - `PLC_BATCH=0` — per-tag mode (legacy)
  A startup self-test compares batch results with per-tag reads and
  falls back per-tag automatically if they disagree.

  **Fixed:** `EPrimitiveDataType` members are .NET primitive names
  (`Double`, `Int32`, `Int16`, `Bool`), not Siemens names (`LReal`,
  `DInt`, `Int`). The previous `_TYPE_NAMES` mapping failed its
  self-test and batch mode was never actually enabled.
  `_TYPE_NAMES` now maps `LReal→Double`, `DInt→Int32`, `Int→Int16`.

- **`PLC_INSTANCE` environment variable** — overrides the default
  instance name (`"296"`) without editing the file.

### Changed

- **`config/plc_ros_bridge.yaml`: `poll_hz: 60 → 120`.**
  With batch I/O the bridge finishes a tick in ~3 ms; the previous
  60 Hz cap left the CPU idle for ~13 ms per tick. At 120 Hz,
  `queue_delay` drops from 13.48 ms to 5.87 ms.
- **`PLC_TIMING` default is now `"1"`** in `plc_server_win.py` —
  timing report every 10 s without setting the env var.

### Performance (Docker/VM, same code, same YAML except `poll_hz`)

| Metric (p50)      | Before (batch OFF, 60 Hz) | After (batch ON, 120 Hz) | Δ |
| :---------------- | ------------------------: | -----------------------: | -: |
| `read_net`        | 6.04 ms                   | 1.36 ms                  | −77% |
| `write_net`       | 13.93 ms                  | 5.28 ms                  | −62% |
| `queue_delay`     | 6.33 ms                   | 5.87 ms                  | −7% |
| `ros_ruckig`      | 7.89 ms                   | 4.32 ms                  | −45% |
| `cmd_to_feedback` | 34.60 ms                  | **18.55 ms**             | **−46%** |
| `total_loop`      | 45.95 ms                  | **25.72 ms**             | **−44%** |
| LBP ≤ 30 ms       | 40.63%                    | **96.33%**               | +55.7pp |
| LBP ≤ 40 ms       | 83.73%                    | **99.85%**               | +16.1pp |
| LBP ≤ 50 ms       | 96.90%                    | **100.00%**              | +3.1pp |

### Fixed

- **`plc_server_win.py` batch self-test** — `_TYPE_NAMES` now
  correctly maps Siemens type names to `EPrimitiveDataType`
  members. Batch mode is actually enabled.

### Migration Notes

- No config schema change (still v2.1).
- No ROS graph change.
- If you use `plc_server_win3.py`, replace it with the v3.1.3 version.
- To disable batch: `PLC_BATCH=0` in the environment before starting
  `plc_server_win.py`.

### Known Limitations (added)

| ID | Limitation |
| :--- | :--- |
| VL-16 | `poll_hz: 120` requires the PLC server to handle 120 requests/s. With batch mode, `plc_server_win.py` handles this comfortably (~0.3 ms per batch request mean). With per-tag mode, do not exceed `poll_hz: 60`. |
| VL-17 | Isaac Sim physics step (`isaac_step`, ~10 ms p50) is now the largest single contributor on the VM. Reducing it requires GPU passthrough or headless mode. |

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
  at the end of each `tick()`, and `origin_ns` uses it.
  **Result:** `queue_delay` = 5.42 ms median (was 0.00).

- **`physics_twin.py` — `spin_once` before `step`/`publish_states`.**
  A setpoint that arrived since the last frame is applied in the same
  frame. **Result:** `isaac_step` −62% (27.05 → 10.25 ms).

- **`trajectory_twin.py` — `SingleThreadedExecutor` (was `MultiThreaded`).**
  MultiThread added overhead on `ros_ruckig` (+26%) without improving
  `isaac_step`. Reverted to `SingleThreadedExecutor`.

- **QoS — `BEST_EFFORT` + `depth=1` on high-rate streams.**
  `/plc/cmd_ang`, `/servo_setpoints`, `/joint_states` now use
  `BEST_EFFORT` with `depth=1`.

### Fixed

- **`latency_probe` `origin_ns`** — now reflects the end of the previous
  tick, not a duplicate of `read_start_ns`.

### Known Limitations

| ID | Limitation |
| :--- | :--- |
| VL-12 | `write_net` is ~12 ms on the VM. Reduce by switching to `plc_api` driver (PLCSIM direct) if the bridge runs on Windows. |
| VL-13 | `isaac_step` is ~10 ms on the VM. Reduce by enabling GPU passthrough or running Isaac Sim on a native host. |
| VL-14 | One 103 s Isaac Sim stall observed. Excluded from filtered statistics. |
| VL-15 | CycloneDDS is not supported on this branch's Isaac Sim setup (Python 3.11 vs 3.12 mismatch). |

### Migration Notes

No migration is required. The config schema is unchanged (v2.1).

---

## [3.1.1] — 2026-10-23 — Environment Transparency and Real VM Results

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Changed

- `krc_twin.py` replaced by `trajectory_twin.py` — a vendor-neutral Ruckig
  trajectory generator and state machine that reads joints, limits, and
  Ruckig parameters from `config/robot.yaml` (closes **VL-01**).
- README Quick Start updated: `trajectory_twin.py --ros-args -p robot:=...`.
- Unit tests: 26 → 29.
- Config schema badge: 2.0 → 2.1.

### Fixed

- **`trajectory_twin.py` — brake cancelled by a persistent bad command.**
- **`plc_ros_bridge.py` — e-stop auto-cleared on reconnect.**

### Documentation

- Added an **Environment** section and **Results (this machine)** with
  real Docker / RTX 5060 numbers.
- Documented the `Articulation.initialize()` hang on Isaac Sim 5.1 and
  the ROS 2 Bridge environment (Python 3.11 vs 3.12).
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

- **Pydantic v2 schema** (`bridge_config.py`) — replaces the hand-written
  `_validate_config`.
- **`schema_version` + migration chain** — v1.0 configs work unchanged.
- **Audit trail** — `sha256` hash + ISO-8601 timestamp logged to JSONL.
- **Fail-fast** — bad config rejected **before** any socket is opened.
- **`robot.yaml` validation** — `_validate_limits`, Ruckig parameter
  sanity.
- **26 unit tests** (19 for config schema, 7 for trajectory planner).
- **`requirements-dev.txt` + `pytest.ini`** — repeatable test setup.
- **PEP 668 aware** — documented venv workflow for Python 3.11+.

### Changed

- `bridge_config.py` is now the canonical schema definition.

---

## [3.0.0] — 2026-10-10 — Vendor-Neutral Platform

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Changed (Breaking)

- **New folder `vendor-neutral/`** — the platform is now decoupled from any
  specific robot or PLC vendor.
- **Repository reorganized by environment.**
- **Transport abstraction** in `linux/plc_drivers.py`.
- **Signal abstraction** in `linux/plc_config.py`.
- **`plc_api` transport added** — direct PLCSIM Advanced API (Windows-only).

### Added

- `vendor-neutral/linux/plc_drivers.py`
- `vendor-neutral/linux/plc_config.py`
- `vendor-neutral/config/robot.yaml`
- Reorganized `vendor-neutral/tools/`.

### Preserved

- All previous generations moved to `archive/`.

### Known Limitations (v3.0.0)

| ID | Limitation |
| :--- | :--- |
| VL-01 | KRC twin still reads config inline |
| VL-02 | Yaskawa GP110 end-to-end verification pending |
| VL-03 | No Cartesian motion or inverse kinematics layer |
| VL-04 | No real-time guarantees (no PREEMPT_RT) |
| VL-05 | No authentication on `tcp_json` transport |

---

## [2.0.0] — 2026-10-07 — Yaskawa Modular

**Location:** [`yaskawa/`](yaskawa/) — Gen 2

### Changed (Breaking)

- Monolithic bridge split into 6 focused modules under `src/plc_bridge/`.

### Added

- `plc_gui.py` — standalone tkinter GUI for manual control.
- `docs/README_v2_full.md` — 742-line documentation with Appendices A–F.

### Fixed

- **Hidden A5 problem** — root cause: `A5 != 0` introduces horizontal offset
  = `LF * sin(A5)`. Solution: `A5 = -(A1 + A4)` for tool-down orientation.

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

- First working proof-of-concept for a KUKA KR210 digital twin.
- Single monolithic Python file (`plc_ros_bridge4.py`).
- Basic TCP/JSON transport between PLCSIM Advanced and Linux.

### Notes

- Superseded by 1.0.0 (full KUKA digital twin).

---

## Summary of Generations

| Version | Robot | Transport | Architecture | Location |
| :--- | :--- | :--- | :--- | :--- |
| v0.1.0 | KUKA KR210 | TCP/JSON | Monolithic | (superseded) |
| v1.0.0 | KUKA KR210 | TCP/JSON | 5 processes | `archive/kuka/` |
| v2.0.0 | Yaskawa GP110 | OPC UA | Modular (6 files) | `archive/yaskawa/` |
| **v3.0.0** | **Any robot** | **Any transport** | **Vendor-neutral** | **`vendor-neutral/`** |
| **v3.1.0** | **Any robot** | **Any transport** | **+ Pydantic v2 + 26 tests** | **`vendor-neutral/`** |
| **v3.1.1** | **Any robot** | **Any transport** | **+ environment transparency + 29 tests** | **`vendor-neutral/`** |
| **v3.1.2** | **Any robot** | **Any transport** | **+ threaded PLC I/O + BEST_EFFORT QoS** | **`vendor-neutral/`** |
| **v3.1.3** | **Any robot** | **Any transport** | **+ batch PLC I/O + poll_hz 120 (18.55 ms)** | **`vendor-neutral/`** |
| **v3.5.0** | **Any robot** | **Any transport** | **+ trajectory_twin fixes (validation, applied dedupe, passive axis)** | **`vendor-neutral/`** |
| **v3.6.0** | **Any robot** | **Any transport** | **+ probe chain fixes (100% capture), same-tick write** | **`vendor-neutral/`** |
| **v3.7.0** | **Any robot** | **Any transport** | **+ probe_attempted counter, joint_mapper.py, 51+8 tests, poll_hz 200 (14.48 ms)** | **`vendor-neutral/`** |

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
