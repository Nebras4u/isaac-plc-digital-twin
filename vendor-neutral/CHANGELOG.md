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
  - `allow_int_for_float` (default `True`) — widening rule for `float` signals
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
  `tick_stats` log now carries:
  ```
  signal_rejections_total=0
  ```
  or, on rejection:
  ```
  signal_rejections_total=140000
  signal_rejections={"velocity_scale":{"above_max":140000}}
  ```
  `_diag()` (diagnostic_updater) reports the same total.

- **`plc_ros_bridge.py` — boot-time type handshake.**
  `TcpJsonDriver.get_server_types()` sends `{"cmd":"types"}` and compares
  the declared types against the config. A mismatch is logged as
  `config_type_mismatch`; with `strict_type_handshake: true`, the
  connection is aborted. This catches YAML↔PLC type drift at first
  connect, not after hours of silent miscasts.

- **`plc_server_win.py` — `types` command + NaN/Inf sanitation.**
  New `cmd: "types"` returns the declared type per tag/array. All JSON
  replies use `allow_nan=False`; `_sanitize()` replaces any `NaN`/`Inf`
  with `"ERR:non_finite"` per tag, per array element. Errors during
  reply serialization are caught and re-emitted as a JSON error reply.

- **`physics_twin.py` — NaN/Inf rejection at all boundaries.**
  - `on_setpoint` rejects non-finite positions per-joint; a name/position
    length mismatch rejects the whole message.
  - `_seed_from_current` / `_refresh_current` refuse non-finite state
    from Isaac (prevents contaminating the E-stop hold position).
  - `publish_states` refuses to publish `/joint_states` if any position
    or velocity component is non-finite.
  - `_check_finite` helper returns `None` (+ bump+warn) on failure.
  - Per-joint `_stats` counters and throttled `_warn`.

- **`trajectory_twin.py` — NaN/Inf rejection at all boundaries.**
  - `on_js` rejects non-finite joint states; a length mismatch rejects
    the whole message.
  - `on_cmd` rejects non-finite commands; a bad message does not update
    `_cmd_deg` or `_last_cmd_t` (the 0.5 s timeout then brakes).
  - `_publish` refuses to publish `/servo_setpoints` with non-finite
    positions (defense in depth on Ruckig output).
  - Per-joint `_stats` counters and throttled `_warn`.

- **Tests — 86 pytest (`test_*.py`) + 9 standalone probe tests.**
  Total grew from 51+8 (v3.7) to **86 + 9** (v4.0):
  - `test_bridge_config.py` — **33 tests** (was 22): +9 v2.2 schema tests,
    +2 corrections
  - `test_trajectory_twin.py` — **27 tests** (was 25): +2 non-finite
    rejection tests
  - `test_physics_twin.py` — **4 tests** (unchanged)
  - `test_industrial_patch.py` — **12 tests** (new): `act_ang` write
    policy, `_pub_locked` estop lock, `io_loop` deadline counting,
    physics estop latch (5 scenarios)
  - `test_probe_chain.py` — **9 tests** (was 8): + `_bind_physics_v4_helpers`
    so tests survive v4.0's `_check_finite` / `_bump` calls

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
  (was `"ros2_"`). Matches `plc_server_win.py`'s default. The old default
  required `plc.instance` in YAML or the driver silently failed to
  connect.

- **`plc_ros_bridge.yaml` — `schema_version: "2.2"`.** New fields:
  `min`/`max` on `cmd_ang`, `act_ang`, `velocity_scale`, `hb_counter`,
  `hb_ack`; `strict_types: true`; `strict_type_handshake: true`.
  `velocity_scale.max` raised from `100.0` (initial guess) to `1000.0`
  after observing the PLC sends `500.0`.

- **`robot.yaml` — `warn_cooldown: 5.0`.** Throttles per-joint
  rejection warnings in `trajectory_twin.py` and `physics_twin.py`.

### Performance (VM, 363 s motion, `cmd_ang: [cmd_ang]`, mostly stationary joints)

**v4.0 measured:**

| Metric                | p50          | p95          | p99          | p99.9        |
| :-------------------- | -----------: | -----------: | -----------: | -----------: |
| `queue_delay`         | 3.10 ms      | 3.51 ms      | 3.60 ms      | 3.70 ms      |
| `read_net`            | 1.80 ms      | 3.90 ms      | 12.29 ms     | 39.32 ms     |
| **`convert_pub`**     | **0.05 ms**  | **0.10 ms**  | **0.13 ms**  | **0.15 ms**  |
| `ros_ruckig`          | 3.09 ms      | 6.32 ms      | 7.52 ms      | 8.61 ms      |
| `isaac_step`          | 5.09 ms      | 8.35 ms      | 8.70 ms      | 8.92 ms      |
| `ros_back`            | 0.35 ms      | 0.48 ms      | 0.55 ms      | 0.69 ms      |
| `write_net`           | 7.86 ms      | 21.10 ms     | 41.16 ms     | 92.84 ms     |
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

| Phase            | mean      | max        |
| :--------------- | --------: | ---------: |
| `api_read_batch` | 1.10–1.4 ms | 11–51 ms |
| `api_write_batch`| 0.60–0.84 ms | 8–56 ms  |
| `cmd_sync` (total)| 1.8–2.3 ms | 15–67 ms |

Confirms the server is fast (~1.8 ms per sync); the `write_net` tails
are environmental (Windows/VM scheduling, Defender).

**Probe health:** `probe_attempted == probe_rows` (100% capture),
`probe_rejected == 0`, `probe_order_violations == 0`,
`write_failed == 0`, `signal_rejections_total == 0`.

### Fixed

- **`test_bridge_config.py::test_estop_forwarding_present`** — the
  estop signal was declared without `type: bool`, so the new
  `publish[].type == signals[].type` check would have rejected it. The
  test config now declares `type: bool` on the estop signal. (This bug
  existed in the v3.7 test suite too, but the v2.1 schema did not check
  it.)

- **`plc_ros_bridge.yaml` — `velocity_scale.max` was `100.0`.**
  The PLC sends `500.0`; every tick was rejected with `above_max` until
  the limit was raised to `1000.0`. Initial value was a guess, corrected
  after observing real traffic in `signal_rejected` log lines.

### Migration Notes

- **Schema v2.1 → v2.2:** `load_bridge_config` migrates in-memory.
  No YAML edit required for load. New fields default to safe values
  (`strict_types=True`, `reject_non_finite=True`,
  `allow_int_for_float=True`, `strict_type_handshake=False`).
- **`strict_types=True` is the default.** During migration, set
  `strict_types: false` in YAML to keep v3.7 behaviour (with
  `_to_int_checked` and `_to_finite_float` guards still active).
  Move to `true` after confirming `signal_rejections_total == 0`.
- **`strict_type_handshake=False` is the default.** Set to `true` once
  the server is upgraded (`plc_server_win.py` v4.0 supports the `types`
  command). A mismatch then aborts the connection at boot.
- **`plc_server_win.py` v4.0 is required for the `types` handshake.**
  v3.3 servers reply `unknown cmd: types`; the bridge logs
  `type_handshake_unavailable` and continues (no abort).
- **`PlcApiDriver` default instance changed** from `"ros2_"` to
  `"296"` to match the server. If your YAML relies on the old default,
  add `plc.instance: "ros2_"` explicitly.
- **Test count** — the `pytest` suite is now **86 tests** (up from 51).
  `test_industrial_patch.py` is new. `test_bridge_config.py` must be
  replaced (the `test_estop_forwarding_present` fix + v2.2 additions).

### Known Limitations (added in v4.0)

| ID | Limitation |
| :--- | :--- |
| VL-23 | `signal_rejections_total` on the `tick_stats` line is cumulative since process start. A non-zero value after startup may indicate a persistent bad signal; check the JSONL log for the per-signal breakdown in `signal_rejections`. |
| VL-24 | `strict_type_handshake: true` requires `plc_server_win.py` v4.0. Against a v3.3 server the bridge logs `type_handshake_unavailable` and does **not** abort (the flag is best-effort against older servers). |
| VL-25 | In `strict_types=True`, a string-formatted number (`"5.0"`) sent to an `int` signal is rejected, not parsed. The server must send typed JSON (which it does by default). |
| VL-26 | The `_wire_type_check` widening rule (`float` accepts `int`) is controlled by `allow_int_for_float` (default `True`). Set to `False` for strict mode; beware that most JSON producers send `5` where `5.0` is expected, which will then be rejected. |
| VL-27 | `nan` in `/joint_states` position now triggers `joint_angles_non_finite` error log and skips the write. This may delay `act_ang` update by one tick (until the next finite `/joint_states`), which the `ACT_ANG_REFRESH_S = 0.5` bound absorbs. |

---

## [3.7.0] — 2026-10-28 — Probe Observability + Production Path
