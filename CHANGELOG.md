# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 4

_Nothing yet._

---

## [4.0.0] — 2026-10-29 — Industrial Type-Safety Overhaul

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 4

### Added

- **`bridge_config.py` — Schema v2.2: per-signal type/range/coercion policy.**
  New optional fields on `SignalConfig`:
  - `min`, `max` — numeric range check (`below_min` / `above_max`)
  - `strict` — per-signal override of `BridgeConfig.strict_types`
  - `reject_non_finite` (default `True`) — NaN/Inf rejection
  - `allow_int_for_float` (default `True`) — widening rule for `float`
  New fields on `BridgeConfig`:
  - `strict_types` (default `True`) — validator mode vs legacy coercer
  - `strict_type_handshake` (default `False`) — abort connect on type mismatch
  New cross-check: `publish[].type` must equal `signals[].type` (catches
  YAML type drift at load). Migration chain accepts v2.0 and v2.1
  in-memory, no manual edit required.

- **`plc_ros_bridge.py` — `_wire_type_check` at the driver boundary.**
  `TcpJsonDriver`, `OpcUaDriver`, and `PlcApiDriver` now call
  `_wire_type_check(expected, value)` on every read. Type mismatches
  surface as `"ERR:TYPE:..."` strings, not silent coercions. `bool` is
  never accepted as `int` (Python subclass trap).

- **`plc_ros_bridge.py` — `TcpJsonDriver.parse_constant` rejects NaN/Infinity.**
  `json.loads(..., parse_constant=_reject_constant)` raises on `NaN`,
  `Infinity`, `-Infinity` at the wire boundary. `json.dumps` uses
  `allow_nan=False` on the request side.

- **`plc_ros_bridge.py` — `_get` is now a validator, not a coercer.**
  In `strict_types=True`, `_validate` rejects:
  - `str` → `int`/`float` coercion (no more `int("5")`)
  - `float` → `int` truncation (`3.9` is rejected, not truncated to `3`)
  - `int` → `bool` (`2` is rejected, not coerced to `True`)
  - `NaN`/`Infinity` on `float` signals (unless `reject_non_finite: false`)
  - Out-of-range values when `min`/`max` are set
  `strict_types=False` activates `_coerce_legacy`, which still applies
  `_to_int_checked` (no silent truncation) and `_to_finite_float`
  (no silent NaN) — so the two critical bugs are closed even in
  transitional mode.

- **`plc_ros_bridge.py` — per-signal rejection counters (`_stats`).**
  `_get` increments `_stats[name][reason]` on every rejection. The
  `tick_stats` log now carries `signal_rejections_total=0` (or, on
  rejection, the per-signal breakdown). `_diag()` reports the same
  total.

- **`plc_ros_bridge.py` — boot-time type handshake.**
  `TcpJsonDriver.get_server_types()` sends `{"cmd":"types"}` and compares
  the declared types against the config. A mismatch is logged as
  `config_type_mismatch`; with `strict_type_handshake: true`, the
  connection is aborted. Catches YAML↔PLC type drift at first
  connect, not after hours of silent miscasts.

- **`plc_server_win.py` — `types` command + NaN/Inf sanitation.**
  New `cmd: "types"` returns the declared type per tag/array. All JSON
  replies use `allow_nan=False`; `_sanitize()` replaces any `NaN`/`Inf`
  with `"ERR:non_finite"` per tag, per array element.

- **`physics_twin.py` — NaN/Inf rejection at all boundaries.**
  - `on_setpoint` rejects non-finite positions per-joint; a
    name/position length mismatch rejects the whole message.
  - `_seed_from_current` / `_refresh_current` refuse non-finite state
    from Isaac (prevents contaminating the E-stop hold position).
  - `publish_states` refuses to publish `/joint_states` if any position
    or velocity component is non-finite.
  - Per-joint `_stats` counters and throttled `_warn`.

- **`trajectory_twin.py` — NaN/Inf rejection at all boundaries.**
  - `on_js` rejects non-finite joint states; a length mismatch rejects
    the whole message.
  - `on_cmd` rejects non-finite commands; a bad message does not update
    `_cmd_deg` or `_last_cmd_t` (the 0.5 s timeout then brakes).
  - `_publish` refuses to publish `/servo_setpoints` with non-finite
    positions (defense in depth on Ruckig output).
  - Per-joint `_stats` counters and throttled `_warn`.

- **Tests — 86 pytest + 9 standalone probe tests.**
  Total grew from 51+8 (v3.7) to **86 + 9** (v4.0):
  - `test_bridge_config.py` — **33 tests** (was 22): +9 v2.2 schema
    tests, +2 corrections
  - `test_trajectory_twin.py` — **27 tests** (was 25): +2 non-finite
    rejection tests
  - `test_physics_twin.py` — **4 tests** (unchanged)
  - `test_industrial_patch.py` — **12 tests** (new): `act_ang` write
    policy, `_pub_locked` estop lock, `io_loop` deadline counting,
    physics estop latch (5 scenarios)
  - `test_probe_chain.py` — **9 tests** (was 8): + `_bind_physics_v4_helpers`

### Changed

- **`plc_ros_bridge.py` — `_js_tag` NaN guard before `act_ang` write.**
  `/joint_states` with non-finite positions is never written to the
  PLC; `cmd_ack` and `feedback_valid` go `False` instead. Prevents a
  `NaN` from silently freezing a joint (the `deadband` comparison would
  return `False` forever, so `needs_write` would stay `False`).

- **`plc_ros_bridge.py` — probe rows use `-1` for unavailable HB.**
  `_LatencyProbe.record(..., hb_counter_val)` is `-1` when the value is
  missing/rejected (real counters are never negative), instead of `0`
  (a plausible counter value).

- **`plc_ros_bridge.py` — `PlcApiDriver` default instance `"296"`**
  (was `"ros2_"`). Matches `plc_server_win.py`'s default.

- **`plc_ros_bridge.yaml` — `schema_version: "2.2"`.** New fields:
  `min`/`max` on `cmd_ang`, `act_ang`, `velocity_scale`, `hb_counter`,
  `hb_ack`; `strict_types: true`; `strict_type_handshake: true`.
  `velocity_scale.max` raised from `100.0` (initial guess) to `1000.0`
  after observing the PLC sends `500.0`.

- **`robot.yaml` — `warn_cooldown: 5.0`.** Throttles per-joint
  rejection warnings in `trajectory_twin.py` and `physics_twin.py`.

### Performance (VM, 363 s motion, `cmd_ang: [cmd_ang]`, mostly stationary joints)

**v4.0 measured:**

| Metric                | p50          | p95          | p99          | p99.9         |
| :-------------------- | -----------: | -----------: | -----------: | ------------: |
| `queue_delay`         | 3.10 ms      | 3.51 ms      | 3.60 ms      | 3.70 ms       |
| `read_net`            | 1.80 ms      | 3.90 ms      | 12.29 ms     | 39.32 ms      |
| **`convert_pub`**     | **0.05 ms**  | **0.10 ms**  | **0.13 ms**  | **0.15 ms**   |
| `ros_ruckig`          | 3.09 ms      | 6.32 ms      | 7.52 ms      | 8.61 ms       |
| `isaac_step`          | 5.09 ms      | 8.35 ms      | 8.70 ms      | 8.92 ms       |
| `ros_back`            | 0.35 ms      | 0.48 ms      | 0.55 ms      | 0.69 ms       |
| `write_net`           | 7.86 ms      | 21.10 ms     | 41.16 ms     | 92.84 ms      |
| **`cmd_to_feedback`** | **16.07 ms** | **30.73 ms** | **50.18 ms** | **106.18 ms** |

**v3.7 reference (88.7 s, 15.7 rows/s):** `convert_pub` p99 = 0.12 ms,
`cmd_to_feedback` p50 = 14.62 ms.

**Interpretation — critical:** the only stage that v4.0 actually
modifies is `convert_pub` (where `_validate` runs, after `read_done`).
Observed cost: **+0.01 ms p99** (0.12 → 0.13 ms), i.e. **~10 µs**.
Every other stage is unchanged or better:
- `queue_delay` p99: 3.62 → 3.60 ms (−1%)
- `ros_ruckig` p99: 8.16 → 7.52 ms (−8%)
- `isaac_step` p99: 9.33 → 8.70 ms (−7%)
- `ros_back` p99: 0.73 → 0.55 ms (−25%)

`write_net` p50 differs from v3.7 (7.86 vs 4.69 ms) because the motion
pattern differs (3.0 rows/s here vs 15.7 rows/s in v3.7, i.e. more
`queue_delay` before each write). **v4.0 does not modify
`TcpJsonDriver.sync()`** — the path from `js_recv_ns` to
`write_done_ns` contains no v4.0 line.

**Server-side timing** (`plc_server_win.py` v4.0, `PLC_TIMING=1`):

| Phase             | mean         | max       |
| :---------------- | -----------: | --------: |
| `api_read_batch`  | 1.10–1.4 ms  | 11–51 ms  |
| `api_write_batch` | 0.60–0.84 ms | 8–56 ms   |
| `cmd_sync` (total)| 1.8–2.3 ms   | 15–67 ms  |

Confirms the server is fast (~1.8 ms per sync); the `write_net` tails
are environmental (Windows/VM scheduling, Defender).

**Probe health:** `probe_attempted == probe_rows` (100% capture),
`probe_rejected == 0`, `probe_order_violations == 0`,
`write_failed == 0`, `signal_rejections_total == 0`.

### Fixed

- **`test_bridge_config.py::test_estop_forwarding_present`** — the
  estop signal was declared without `type: bool`, so the new
  `publish[].type == signals[].type` check would have rejected it. The
  test config now declares `type: bool` on the estop signal.

- **`plc_ros_bridge.yaml` — `velocity_scale.max` was `100.0`.**
  The PLC sends `500.0`; every tick was rejected with `above_max` until
  the limit was raised to `1000.0`.

### Migration Notes

- **Schema v2.1 → v2.2:** `load_bridge_config` migrates in-memory.
  No YAML edit required for load. New fields default to safe values.
- **`strict_types=True` is the default.** During migration, set
  `strict_types: false` in YAML to keep v3.7 behaviour (with
  `_to_int_checked` and `_to_finite_float` guards still active).
- **`strict_type_handshake=False` is the default.** Set to `true` once
  the server is upgraded (`plc_server_win.py` v4.0 supports the `types`
  command). A mismatch then aborts the connection at boot.
- **`PlcApiDriver` default instance changed** from `"ros2_"` to `"296"`.
- **Test count** — the `pytest` suite is now **86 tests** (up from 51).

### Known Limitations (added in v4.0)

| ID | Limitation |
| :--- | :--- |
| VL-23 | `signal_rejections_total` on the `tick_stats` line is cumulative since process start. A non-zero value after startup may indicate a persistent bad signal; check the JSONL log for the per-signal breakdown. |
| VL-24 | `strict_type_handshake: true` requires `plc_server_win.py` v4.0. Against a v3.3 server the bridge logs `type_handshake_unavailable` and does **not** abort. |
| VL-25 | In `strict_types=True`, a string-formatted number (`"5.0"`) sent to an `int` signal is rejected, not parsed. |
| VL-26 | The `_wire_type_check` widening rule (`float` accepts `int`) is controlled by `allow_int_for_float` (default `True`). |
| VL-27 | `nan` in `/joint_states` position now triggers `joint_angles_non_finite` and skips the write. May delay `act_ang` update by one tick. |

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

| Version    | Robot         | Transport         | Architecture                                                       | Location              |
| :--------- | :------------ | :---------------- | :----------------------------------------------------------------- | :-------------------- |
| v0.1.0     | KUKA KR210    | TCP/JSON          | Monolithic                                                         | (superseded)          |
| v1.0.0     | KUKA KR210    | TCP/JSON          | 5 processes                                                        | `kuka/`               |
| v2.0.0     | Yaskawa GP110 | OPC UA            | Modular (6 files)                                                  | `yaskawa/`            |
| **v3.0.0** | **Any robot** | **Any transport** | **Vendor-neutral**                                                 | **`vendor-neutral/`** |
| **v3.1.0** | **Any robot** | **Any transport** | **+ Pydantic v2 + 26 tests**                                       | **`vendor-neutral/`** |
| **v3.1.1** | **Any robot** | **Any transport** | **+ environment transparency + 29 tests**                          | **`vendor-neutral/`** |
| **v3.1.2** | **Any robot** | **Any transport** | **+ threaded PLC I/O + BEST_EFFORT QoS**                           | **`vendor-neutral/`** |
| **v3.1.3** | **Any robot** | **Any transport** | **+ batch PLC I/O + poll_hz 120 (18.55 ms)**                       | **`vendor-neutral/`** |
| **v3.5.0** | **Any robot** | **Any transport** | **+ trajectory_twin fixes**                                        | **`vendor-neutral/`** |
| **v3.6.0** | **Any robot** | **Any transport** | **+ probe chain fixes (100% capture)**                             | **`vendor-neutral/`** |
| **v3.7.0** | **Any robot** | **Any transport** | **+ probe_attempted + joint_mapper.py + 51+8 tests + 14.48 ms**    | **`vendor-neutral/`** |
| **v4.0.0** | **Any robot** | **Any transport** | **+ industrial type-safety (schema v2.2, min/max/strict, type **   | **`vendor-neutral/`** |
|            |               |                   |  ** handshake, NaN guards), 86+9 tests, +0.01 ms p99 vs v3.7**     |                       |

---

## Versioning Policy

This project uses [Semantic Versioning](https://semver.org/):

- **MAJOR** (X.0.0) — incompatible API or architecture changes
- **MINOR** (0.X.0) — new features, backward compatible
- **PATCH** (0.0.X) — bug fixes, backward compatible

---

## Links

- **Repository:** https://github.com/Nebras4u/isaac-plc-digital-twin
- **Gen 4 (active):** [`vendor-neutral/`](vendor-neutral/)
- **Gen 3 (reference):** [`vendor-neutral/`](vendor-neutral/)
- **Gen 2 (reference):** [`yaskawa/`](yaskawa/)
- **Gen 1 (reference):** [`kuka/`](kuka/)

***

*Maintained by [Nebras](https://github.com/Nebras4u) — nebras4u@gmail.com*
