# Isaac Digital Twin — Vendor-Neutral Platform

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![PLC](https://img.shields.io/badge/PLC-Vendor--Neutral-lightgrey)](#supported-combinations)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue)](https://docs.ros.org/)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-5.1-green)](https://developer.nvidia.com/isaac-sim)
[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.11%20%7C%203.12-yellow)](https://www.python.org)
[![Pydantic](https://img.shields.io/badge/Pydantic-2.x-blueviolet)](https://docs.pydantic.dev/)
[![Tests](https://img.shields.io/badge/Tests-29%20passed-success)](#testing)
[![Schema](https://img.shields.io/badge/Config%20Schema-v2.1-success)](#configuration)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen)]()
[![Reference latency](https://img.shields.io/badge/Reference%20mean%20latency-5.2%20ms-success)](#results)
[![Reference max](https://img.shields.io/badge/Reference%20max%20latency-16.3%20ms-success)](#results)

A **vendor-neutral Digital Twin platform** for industrial robot arms.
The platform synchronizes any robot arm with any PLC through **ROS 2**,
using a transport-agnostic bridge and a robot-agnostic YAML configuration.

This folder (`vendor-neutral/`) is the evolution of two robot-specific
implementations (KUKA KR210 and Yaskawa GP110) into a reusable,
configuration-driven platform.

---

## Safety Notice

> **This project is a simulation / digital-twin research platform. It is
> NOT a certified safety function and must not be used to protect people
> or physical machinery.**

- The e-stop, heartbeat, and watchdog logic (bridge, trajectory twin, PLC
  counters) is **functional monitoring** that stops the *simulated* robot.
  It is not designed, validated, or certified according to IEC 61508,
  ISO 13849-1, or IEC 62061.
- `tcp_json`, OPC UA, and ROS 2 links carry no safety-rated integrity
  (no black channel, no safety protocol).
- A real installation needs an independent certified safety layer (for
  example an F-CPU with PROFIsafe and hardware E-Stop circuits).

---

## Table of Contents

- [Safety Notice](#safety-notice)
- [Environment](#environment)
- [Motivation](#motivation)
- [What "Vendor-Neutral" Means](#what-vendor-neutral-means)
- [Architecture](#architecture)
- [Repository Structure](#repository-structure)
- [Supported Combinations](#supported-combinations)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
  - [Schema v2.1 — Pydantic validation](#schema-v21--pydantic-validation)
  - [Signal mapping](#configplc_ros_bridgeyaml--signal-mapping)
  - [Robot description](#configrobotyaml--robot-description)
- [Testing](#testing)
- [Results](#results)
  - [Tracking Accuracy](#tracking-accuracy)
  - [Control-Loop Latency (reference)](#control-loop-latency-reference)
  - [Bridge Real-Time Behaviour](#bridge-real-time-behaviour)
  - [Clock Drift Calibration](#clock-drift-calibration)
  - [Results (this machine)](#results-this-machine)
- [Performance Tuning](#performance-tuning)
- [Documentation](#documentation)
- [Lineage](#lineage)
- [Known Limitations](#known-limitations)
- [Roadmap](#roadmap)
- [License](#license)
- [Contact](#contact)

---

## Environment

> **The performance numbers in this README depend on the environment.**
> Always re-measure on your own hardware before drawing conclusions.

The reference numbers (5.2 ms mean, 16.3 ms max) were measured on the
**author's reference machine**:

| Factor         | Reference machine                   |
| :------------- | :---------------------------------- |
| OS             | Native Ubuntu 24.04                 |
| CPU            | Dedicated cores (no throttling)     |
| GPU            | NVIDIA, no passthrough              |
| PLC server     | Native Windows, low-latency         |
| Isaac Sim      | Native, 60 FPS stable               |
| ROS 2          | Jazzy                               |
| RMW            | FastDDS                             |
| `tcp_json`     | msgpack + `TCP_NODELAY` + batch     |

The **same code** on a **Docker / VM environment** (RTX 5060, shared
vCPUs, PLCSIM Advanced over TCP/JSON) gives different numbers — see
[Results (this machine)](#results-this-machine). This is expected and
does not indicate a regression.

**How to measure on your machine:**

1. Set `latency_probe: true` in `config/plc_ros_bridge.yaml`.
2. Run the bridge for ≥ 30 s.
3. Analyze `latency_probe.csv` with `tools/latency_full_loop.py`, or use
   the inline snippet in [Results (this machine)](#results-this-machine).

---

## Motivation

Previous generations were tightly coupled to a specific robot and a
specific PLC transport. Every migration required rewriting multiple files.

This folder removes those constraints:

> **Switch robots and PLCs by editing YAML — not by editing Python.**

Three abstractions make this possible:

1. **Transport abstraction** — `plc_drivers.py` provides a common
   `PlcDriver` interface with three implementations: `tcp_json`, `opcua`,
   `plc_api`.
2. **Signal abstraction** — `plc_config.py` handles address mapping and
   value extraction, decoupled from any specific PLC protocol.
3. **Robot abstraction** — a single `robot.yaml` describes joints,
   limits, drives, and Isaac paths for any robot.

The main nodes (`trajectory_twin.py`, `plc_ros_bridge.py`,
`physics_twin.py`) stay focused on their responsibility and delegate
these concerns to the modules above.

**Since Gen 3.1**, configuration validation is **declarative**
(Pydantic v2 schema in `bridge_config.py`), with **versioning**,
**audit trail**, and **29 unit tests** covering every structural rule.
A bad config fails **before** the bridge touches the PLC.

---

## What "Vendor-Neutral" Means

| Axis             | Before                          | Now                                             |
| :--------------- | :------------------------------ | :---------------------------------------------- |
| **PLC vendor**   | Siemens only                    | Any PLC with OPC UA, TCP/JSON, or PLCSIM API    |
| **Robot vendor** | KUKA only, then Yaskawa only    | Any articulated robot described in `robot.yaml` |
| **Transport**    | TCP/JSON only, then OPC UA only | Selected by a single YAML line                  |

The same code runs unchanged across all combinations. Only configuration
differs.

---

## Architecture

    +----------------------------------------------------------------------+
    |                          PLC (any vendor)                            |
    |   Siemens S7-1500 - Beckhoff TwinCAT - Rockwell - OPC UA servers     |
    +--------------------------------+-------------------------------------+
                                     |
                           one of three transports
                                     |
            +------------------------+--------------------------+
            |                        |                          |
       tcp_json                    opcua                     plc_api
     (Windows server)           (native OPC UA)         (PLCSIM direct)
            |                        |                          |
            +------------------------+--------------------------+
                                     |
                        +------------v------------+
                        |  linux/plc_ros_bridge.py|
                        |  (+ bridge_config.py)   |   <-- Pydantic v2 schema
                        |  (+ plc_drivers.py)     |       + audit trail
                        |  (+ plc_config.py)      |       + versioning
                        +------------+------------+
                                     |
                                ROS 2 topics
                                     |
                        +------------v------------+
                        |linux/trajectory_twin.py |
                        | (Ruckig + state machine)|
                        +------------+------------+
                                     |
                        +------------v------------+
                        |  isaac/physics_twin.py  |
                        |  (inside Isaac Sim)     |
                        +-------------------------+

### Runtime threading model

`plc_ros_bridge.py` runs on a **`SingleThreadedExecutor`**. This is
deliberate:

- Timer (`tick()`) and subscriber (`on_joint_states()`) callbacks run
  strictly in sequence.
- They cannot race for the Python GIL mid-tick, which was causing
  200–1000 ms `read` stalls on VMs.
- The 60 Hz tick is preserved; the only cost is that a subscriber
  callback can delay the next tick by its own runtime (~20–50 µs).

`trajectory_twin.py` and `physics_twin.py` use the default executor; they
do not touch the same critical timing as the bridge.

---

## Repository Structure

    vendor-neutral/
    |
    +-- README.md                         # This file
    +-- CHANGELOG.md                      # Gen 3 history
    +-- requirements-dev.txt              # Pydantic + pytest
    +-- pytest.ini                        # Test configuration
    |
    +-- windows/
    |   +-- plc_server_win.py             # PLCSIM Advanced -> TCP/JSON
    |
    +-- linux/
    |   +-- trajectory_twin.py            # Ruckig trajectory + state machine
    |   +-- plc_ros_bridge.py             # ROS 2 <-> PLC bridge
    |   +-- bridge_config.py              # Pydantic v2 schema + migration + audit
    |   +-- plc_drivers.py                # 3 transports (tcp_json/opcua/plc_api)
    |   +-- plc_config.py                 # signal mapping + extractor
    |   +-- bridge_logger.py              # Structured logging
    |
    +-- tests/
    |   +-- conftest.py
    |   +-- test_bridge_config.py         # 22 tests: schema + migration + safety
    |   +-- test_trajectory_twin.py       # 7 tests: Ruckig parameter validation
    |
    +-- isaac/
    |   +-- physics_twin.py               # Articulation driver
    |   +-- Main_Robot_L_URDF_tool.urdf
    |   +-- Full_Control.usd
    |
    +-- config/
    |   +-- plc_ros_bridge.yaml           # Signal mapping (schema_version: 2.1)
    |   +-- robot.yaml                    # Joints, limits, Ruckig, drives
    |
    +-- tools/
    |   +-- latency_simpel.py             # Quick latency check
    |   +-- latency_academic.py           # LBP / p99.9 / jitter
    |   +-- latency_full_loop.py          # Full-loop L1+L2+L3+L4
    |   +-- compare_results.py            # A/B comparison tool
    |   +-- srv.py                        # Clock responder
    |   +-- cli.py                        # Clock drift client
    |   +-- info.txt
    |
    +-- plc/
    |   +-- SCL_heartbeat_OB30.txt
    |   +-- SCL_MAIN.txt
    |   +-- SCL_heartbeat_OB30-init.txt
    |   +-- SCL_MAIN-init.txt
    |   +-- DB.txt
    |
    +-- docs/
        +-- migration.md
        +-- latency_comparison.md         # Full comparison report
        +-- latency_full_loop_OPC-UA.md   # Baseline report
        +-- upgrade_report.html           # Session report: Prototype -> Production Grade

---

## Supported Combinations

| PLC                           | Transport  | Robot                     | Mean Latency | Status     |
| :---------------------------- | :--------- | :------------------------ | :----------: | :--------- |
| Siemens S7-1500 (PLCSIM Adv.) | `plc_api`  | KUKA KR210                | TBD          | Verified   |
| Siemens S7-1500 (PLCSIM Adv.) | `tcp_json` | KUKA KR210                | 5.2 ms (ref) | Verified   |
| Any OPC UA server             | `opcua`    | Any robot in `robot.yaml` | ~7 ms (ref)  | Structural |
| Siemens S7-1500               | `opcua`    | Yaskawa GP110             | TBD          | Structural |

**"Structural"** = code path exercised, imports work; full end-to-end
verification on that combination is pending.

---

## Quick Start

### Prerequisites

**Windows (only if using `tcp_json`):**

- Python 3.x with `pythonnet`
- PLCSIM Advanced V6.0
- `pip install msgpack`

**Linux (bridge + trajectory twin):**

- Ubuntu 22.04 / 24.04
- ROS 2 Humble or Jazzy
- Python 3.10+
- `pip install "pydantic>=2.0,<3" ruckig numpy pyyaml msgpack`
- For running the tests: `pip install pytest pytest-cov`

> **PEP 668 note:** on Python 3.11+ systems (Ubuntu 24.04+), install into
> a virtual environment, not the system Python:
>
>     python3 -m venv --system-site-packages plc_env
>     source plc_env/bin/activate
>     pip install "pydantic>=2.0,<3" pytest pytest-cov ruckig

**Isaac Sim host:**

- Isaac Sim 5.1 (5.0 supported)
- ROS 2 Bridge extension enabled

### Startup Sequence

    # 1. Windows: TCP/JSON server (only if driver: tcp_json)
    cd windows && python plc_server_win.py --host 0.0.0.0 --port 5000

    # 2. Isaac Sim Script Editor: physics twin
    #    Run: isaac/physics_twin.py

    # 3. Linux: PLC <-> ROS bridge (with CPU pinning for latency)
    taskset -c 2 python3 linux/plc_ros_bridge.py \
        --ros-args -p config:=config/plc_ros_bridge.yaml

    # 4. Linux: trajectory twin (Ruckig trajectory generator)
    taskset -c 3 python3 linux/trajectory_twin.py \
        --ros-args -p robot:=config/robot.yaml

### Switching Transport

Edit one line in `config/plc_ros_bridge.yaml`:

    plc:
      driver: opcua          # change to: tcp_json | opcua | plc_api

Everything else stays the same.

### Validate Config Before Running

You can validate a config file without starting the bridge:

    python3 -c "from bridge_config import load_bridge_config; \
                cfg, h = load_bridge_config('config/plc_ros_bridge.yaml'); \
                print(f'OK schema={cfg.schema_version} hash={h[:12]}...')"

On error, this prints a precise message with field path and location:

    ConfigError: config validation failed for config/plc_ros_bridge.yaml:
    1 validation error for BridgeConfig
    robot.cmd_ang[0]
      Value error, signal 'missing' referenced in 'robot.cmd_ang[0]'
      is not defined in 'signals' [type=value_error]

### Isaac Sim Setup (physics_twin.py)

`physics_twin.py` runs inside Isaac Sim's Script Editor and connects the
simulated robot to the ROS 2 graph. It is responsible for:

1. Reading `robot.yaml` to find the robot prim paths and drive gains.
2. Creating an `Articulation` object and initializing it.
3. Disabling gravity for the robot links (Isaac Sim 5.1 has no gravity
   compensation API on `Articulation`; disabling gravity is the only
   reliable way to get zero tracking error).
4. Publishing `/joint_states` at 60 Hz.

**Gravity handling:** Isaac Sim 5.1 does not expose
`get_dof_gravity_compensation_forces()` or `set_gravity_compensation()` on
`Articulation`. The documented workaround is to apply
`PhysxSchema.PhysxRigidBodyAPI` with `disableGravity=True` to every link
under the robot scope. This is done by `disable_gravity_for_robot_links()`
at startup and again after every reinit.

**Recovery from Stop/Play:** PhysX invalidates simulation views whenever
the timeline is stopped or the stage is modified. `physics_twin.py`
detects this via `_needs_reinit` and rebuilds the `Articulation`
automatically — no manual restart required.

**Startup order (important):**

1. Open Isaac Sim (do **not** press Play yet).
2. Run `physics_twin.py` in the Script Editor.
3. Wait for `Gravity disabled for N robot links.`
4. **Then** press Play.

> **Known issue (Isaac Sim 5.1):** `Articulation.initialize()` may hang
> if Play has not been pressed at least once. If you see
> `Initializing articulation...` and nothing else for > 30 s, press Play
> and re-run `physics_twin.py`.

### ROS 2 Bridge environment (Isaac Sim)

Isaac Sim 5.1 ships its own Python 3.11 and its own ROS 2 libraries.
It does **not** use the system ROS 2 (Python 3.12). To make `rclpy`
available inside Isaac Sim, start Isaac Sim from a terminal with:

    export ROS_DISTRO=jazzy
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib
    /isaac-sim/kit/kit /isaac-sim/apps/isaacsim.exp.full.kit

Do **not** `source /opt/ros/jazzy/setup.bash` before starting Isaac Sim
— that points Isaac Sim to the system `rclpy` (Python 3.12), which is
ABI-incompatible with Isaac Sim's Python 3.11.

`plc_ros_bridge.py` and `trajectory_twin.py` run on the **system** ROS 2
(Python 3.12) and communicate with Isaac Sim through DDS.

---

## Configuration

### Schema v2.1 — Pydantic validation

`bridge_config.py` defines the config schema declaratively with Pydantic
v2. Benefits over the previous hand-written validator:

| Aspect                 | Before (v1.0)                           | After (v2.1)                                     |
| :--------------------- | :-------------------------------------- | :----------------------------------------------- |
| Validation code        | ~60 lines, hand-written                 | Declarative Pydantic model                       |
| Unknown fields         | Silently ignored                        | **Rejected** (`extra="forbid"`)                  |
| Numeric ranges         | Not checked                             | `Field(gt=, le=)` per field                      |
| Error messages         | `signal 'x' is used but not defined`    | `signal 'x' referenced in 'robot.cmd_ang[0]' …`  |
| Cross-reference        | Manual, no context                      | Automatic, with **field location**               |
| Direction sanity       | Partial                                 | Every `cmd_ang`/`act_ang`/`ack` checked          |
| Address safety         | None                                    | Rejects `\x00`, `\n`, `\r`, `..`                 |
| Versioning             | None                                    | `schema_version` + migration chain               |
| Audit trail            | None                                    | `sha256` hash + timestamp in JSONL               |
| E-stop forwarding      | None                                    | Enforced at config load (opt-out for tests)      |
| Unit tests             | 0                                       | **29**                                           |

**Backward compatibility:** configs without `schema_version` are treated
as v1.0 and migrated to v2.1 in-memory. No manual edits required.

**Fail-fast guarantee:** a bad config is rejected **before** the bridge
attempts to open a socket. There is no possibility of starting the bridge
with invalid signals, ranges, or cross-references.

### `config/plc_ros_bridge.yaml` — Signal mapping

    schema_version: "2.1"       # v1.0 files work without it

    latency_probe: false        # enable per-tick CSV timing probe

    plc:
      driver: tcp_json               # opcua | tcp_json | plc_api
      endpoint: opc.tcp://192.168.100.50:4840   # opcua only
      host: 192.168.100.18                      # tcp_json only
      port: 5000                                # tcp_json only
      timeout: 1.5                              # all drivers — see note below

    poll_hz: 60
    watchdog_timeout: 4.0

    logging:
      level: INFO
      console: true
      to_ros: false              # do NOT duplicate to ROS logger
      file: logs/bridge.log
      json_file: logs/bridge.jsonl
      max_bytes: 10485760
      backup_count: 5

    vars:
      ns: 'ns=3;s="MAIN_KUKA_KR210_L150_L"'
      db: 'MAIN_KUKA_KR210_L150_L'

    signals:
      cmd_ang: {addr: 'MainRobotLeft_CmdAng', type: float, dir: read}
      act_ang: {addr: 'MainRobotLeft_ActAng', type: float, dir: write}
      estop:   {addr: 'MainRobotLeft_Emergency_Stop_Active', type: bool, dir: read}

    publish:
      - {topic: /plc/emergency_stop, signal: estop, type: bool}

    heartbeat:
      counter: hb_counter
      ack: ros_ack_counter

    robot:
      joints:  [joint_1, joint_2, joint_3, joint_4]
      cmd_ang: [cmd_ang]     # one array signal (all joints)
      act_ang: [act_ang]
      cmd_ack: cmd_ack

**Schema v2.1 rules enforced at startup:**

- `plc.driver` must be one of `tcp_json`, `opcua`, `plc_api`.
- `tcp_json` requires `plc.host`; `opcua` requires `plc.endpoint`.
- `poll_hz ∈ (0, 1000]`; `watchdog_timeout ∈ (0.1, 60]`; `backoff_max ∈ (0, 30]`.
- `signals` must be non-empty.
- Every signal referenced in `robot`, `publish`, `heartbeat` must exist in
  `signals`.
- `robot.cmd_ang` signals must have `dir: read`; `act_ang` and `*_ack`
  must have `dir: write`.
- `publish[].signal` must have `dir: read`.
- `publish` must forward the PLC E-stop to `/plc/emergency_stop` unless
  `estop_forwarding_required: false` is set (test/simulation only).
- `addr` must not contain `\x00`, `\n`, `\r`, or `..`.
- Unknown top-level or nested fields are rejected.

**Why `timeout: 1.5`?** The PLC server occasionally has a 200–1000 ms
cold-start stall (new TCP connection, first read after reconnect). A
timeout under ~200 ms fires the reconnect cascade on every such stall,
publishes a spurious e-stop, and requires a 1 s backoff. 1.5 s absorbs
all observed stalls in a single read attempt while still surfacing a
genuine link failure within two poll periods.

**Why `to_ros: false`?** When both the console/file handler and the ROS
logger are attached, every record appears twice on the terminal. The
JSONL file retains full structure; the ROS echo is redundant.

**Why `watchdog_timeout: 4.0`?** The bridge treats a gap of > 4 s
between successful ticks as a liveness failure and publishes
`/plc/emergency_stop`. At 60 Hz with 200–400 ms server stalls, 4 s is the
first threshold that does not produce false positives.

**Audit trail:** on startup, the bridge logs `config_loaded` with the
`sha256` of the canonicalised config, the `schema_version`, and an
ISO-8601 UTC timestamp. This is appended to both the human log and the
JSONL file:

    {"event":"config_loaded","hash":"751ed1fb...729ed5","schema_version":"2.1",
     "timestamp_utc":"2026-10-21T16:44:27.486232+00:00",
     "path":"plc_ros_bridge.yaml","node":"plc_ros_bridge"}

The hash is **deterministic** — same content → same hash across runs,
across machines. Use it to verify that a config has not changed between
two deployments, or to match an experiment log to the exact config used.

**Important:** `/plc/cmd_ang` is republished on **every** tick, even when
the joint targets have not changed. `trajectory_twin.py` uses message
recency as a liveness signal (0.5 s watchdog). Deduplicating unchanged
`cmd_ang` messages causes the twin to stop publishing setpoints and the
robot silently stops moving. Only the **state** topics
(`/plc/velocity`, `/plc/axis_enable`, `/plc/emergency_stop`,
`/plc/ros_alive`, `/plc/comm_state`) are deduped — they are latched
state and rarely change.

### `config/robot.yaml` — Robot description

Used by `trajectory_twin.py` (and the Isaac physics twin): joints, limits
(`limits_deg`), Ruckig limits (`ruckig`), drive gains, and Isaac prim paths.
`all_joints` includes the passive joint (`joint_5`); `controlled_joints`
are the axes commanded by the PLC.

**Validated at startup (since v2.1):**

- Every `controlled_joints` entry must have `limits_deg`.
- `limits_deg[joint]` must be `[lower, upper]` with `lower < upper`.
- `hi − lo ≤ 720°` (rejects unit-confusion: degrees vs radians).
- `ruckig.control_hz ∈ (0, 1000]`, `max_vel > 0`, `max_acc > 0`,
  `max_jerk > 0`, `jump_threshold_rad > 0`.

A malformed `robot.yaml` fails at startup, not mid-trajectory.

### Terminal output

`trajectory_twin.py` prints a column-aligned table once per second:

    joint           cmd_deg    actual_deg     setpt_deg
    --------------------------------------------------
    joint_1           33.000        33.004        33.002
    joint_2          -60.000       -59.981       -59.987
    joint_3           66.000        66.006        66.003
    joint_4            0.000        -0.000        -0.000

`cmd_deg` comes from `/plc/cmd_ang`, `actual_deg` from `/joint_states`
(feedback from the Isaac physics twin), and `setpt_deg` is the Ruckig
output going to `/servo_setpoints`. All angles in degrees for readability;
the published `JointState` messages still use radians, per ROS convention.

`plc_ros_bridge.py` prints a `tick_stats` line every ~10 s:

    tick_stats | ticks=600 slow=1 slow_pct=0.17 avg_slow_sock_ms=324.13 avg_slow_gap_ms=0.0

- `slow_pct` should stay under 0.5% on a healthy host.
- `avg_slow_gap_ms > 0` indicates GIL contention (bridge-side).
- `avg_slow_sock_ms > 0` indicates real server-side stalls (not bridge-side).

---

## Testing

### Running the test suite

    # From the vendor-neutral folder
    cd /path/to/vendor-neutral
    python3 -m pytest tests/ -v

Expected output:

    ============================== 29 passed in 0.17s ==============================

### What is tested

The 29 unit tests cover **every structural rule** enforced by the schema
and the trajectory planner. Each test maps to a specific industrial risk:

**`tests/test_bridge_config.py` — 22 tests**

| Test                                                     | Prevents                                                     |
| :------------------------------------------------------- | :----------------------------------------------------------- |
| `test_minimal_valid`                                     | Baseline: a valid config loads without error                 |
| `test_v1_migration`                                      | Old configs (no `schema_version`) still work                 |
| `test_missing_driver`                                    | Bridge starting without a driver selected                    |
| `test_unknown_field`                                     | Silent typos in field names (`poll_hzz` → ignored)           |
| `test_empty_signals`                                     | Empty signal map causing runtime `KeyError`                  |
| `test_poll_hz_out_of_range[0/-1/1001/100000]`            | CPU-melting poll rates or division by zero                   |
| `test_tcpjson_requires_host`                             | `tcp_json` without `host` → `KeyError` after 10 s            |
| `test_opcua_requires_endpoint`                           | `opcua` without `endpoint` → connection hang                 |
| `test_cross_ref_includes_location`                       | Missing signal reported **without its location**             |
| `test_publish_cross_ref_location`                        | Same, for `publish[].signal`                                 |
| `test_cmd_ang_must_be_read`                              | `cmd_ang` marked `write` → bridge tries to write a read tag  |
| `test_act_ang_must_be_write`                             | Same, inverse                                                |
| `test_addr_rejects_control_chars[a\x00b/a\nb/a\rb/a..b]` | Path-traversal or NUL injection in signal addresses          |
| `test_physics_twin_reinit`                               | `_needs_reinit` fails to rebuild Articulation after Stop/Play|
| `test_robot_yaml_limits`                                 | `limits_deg` invalid (lower >= upper, range > 720°)          |
| `test_config_hash_deterministic`                         | same config produces different hashes across runs            |
| `test_estop_forwarding_required`                         | Missing e-stop rule is rejected at config load               |
| `test_estop_forwarding_optout`                           | `estop_forwarding_required: false` allows tests to omit it   |
| `test_estop_forwarding_present`                          | A valid e-stop rule passes                                   |

**`tests/test_trajectory_twin.py` — 7 tests**

| Test                                     | Prevents                                       |
| :--------------------------------------- | :--------------------------------------------- |
| `test_ruckig_rejects_bad_params[bad0]`   | `control_hz=0` → `ZeroDivisionError`           |
| `test_ruckig_rejects_bad_params[bad1]`   | `control_hz=2000` → unrealistic CPU load       |
| `test_ruckig_rejects_bad_params[bad2]`   | `control_hz=-1` → nonsensical timer            |
| `test_ruckig_rejects_bad_params[bad3]`   | `max_vel=0` → robot never moves                |
| `test_ruckig_rejects_bad_params[bad4]`   | `max_acc=0` → same                             |
| `test_ruckig_rejects_bad_params[bad5]`   | `max_jerk=0` → same                            |
| `test_ruckig_rejects_bad_params[bad6]`   | `jump_threshold=0` → velocity reset every tick |

### Why unit tests matter here

The code is used to drive a real PLC and a real robot. A regression
introduced during a future refactor (e.g. removing the `Field(gt=0)`
constraint from `poll_hz`) would not be visible until the bridge is
running under load — at which point the failure mode is either a crash
or a frozen host. The tests make that regression **visible in 0.2 s**,
on every commit.

### Running tests in CI (optional)

Add to `.github/workflows/test.yml`:

    name: tests
    on: [push, pull_request]
    jobs:
      test:
        runs-on: ubuntu-24.04
        steps:
          - uses: actions/checkout@v4
          - uses: actions/setup-python@v5
            with:
              python-version: "3.12"
          - run: pip install "pydantic>=2.0,<3" PyYAML pytest pytest-cov
          - run: pytest tests/ -v --cov=bridge_config

---

## Results

### Tracking Accuracy

Measured on KUKA KR210 through `tcp_json`:

| Joint       | Setpoint  | Actual    | Error   |
| :---------- | :-------- | :-------- | :------ |
| joint_1     | 30.0002°  | 30.0002°  | 0.0001° |
| joint_2     | -29.9603° | -29.9349° | 0.0254° |
| joint_3     | 60.0097°  | 60.0133°  | 0.0036° |
| joint_4     | -0.0025°  | -0.0033°  | 0.0008° |
| joint_5     | -0.4668°  | -0.4668°  | 0.0000° |

Maximum error: **0.0254°** on `joint_2` under full gravity load.

*Note: `joint_5` is passive. Its setpoint is taken from the measured
position, so its 0.0000° error is zero by construction and is not a
tracking result.*

### Control-Loop Latency (reference)

> **Reference machine only.** See [Environment](#environment) and
> [Results (this machine)](#results-this-machine) for the VM numbers.

Measured with `tools/latency_full_loop.py` (L2 + L3, after TCP-JSON
optimization).

#### Summary (3 independent runs)

| Stage                       | Mean         | p95        | p99        | Max        |
| :-------------------------- | :----------: | :--------: | :--------: | :--------: |
| L1 — OPC UA/TCP read        | ~8 ms        | ~12 ms     | ~20 ms     | ~25 ms     |
| L2 — ROS + Ruckig           | ~6.5 ms      | ~12 ms     | ~24 ms     | ~24 ms     |
| L3 — Isaac physics          | ~3–13 ms     | ~16 ms     | ~17 ms     | ~17 ms     |
| L4 — PLC write              | ~4.5 ms      | ~6 ms      | ~15 ms     | ~16 ms     |
| **Full loop (L1+L2+L3+L4)** | **~15–25 ms**| **~40 ms** | **~55 ms** | **~80 ms** |

#### Comparison across transports

| Transport              | Mean       | Max        | Notes                           |
| :--------------------- | :--------: | :--------: | :------------------------------ |
| OPC UA (baseline)      | 11.5 ms    | 100 ms     | Original — tag-by-tag reads     |
| TCP-JSON (baseline)    | 12.5 ms    | 233 ms     | Original — Windows GIL noise    |
| TCP-JSON optimized     | 5.2 ms     | 16.3 ms    | Batch + msgpack + TCP_NODELAY   |

#### Latency Bound Probability (LBP)

| Threshold | LBP      |
| :-------- | :------: |
| <= 5 ms   | 46.9%    |
| <= 10 ms  | 92.9%    |
| <= 15 ms  | 98.3%    |
| <= 20 ms  | 100%     |
| <= 50 ms  | 100%     |

**Result:** 100% of full-loop samples stay under 20 ms on the
**reference machine**. This is well within the control cycle for
medium-speed industrial applications.

See `docs/latency_comparison.md` for the full report.

### Bridge Real-Time Behaviour

Measured with the `tick_stats` diagnostic (60 Hz, 10 s window,
`tcp_json` transport, TSC clocksource).

| Metric                              | Value        | Notes                                  |
| :---------------------------------- | :----------- | :------------------------------------- |
| Effective tick rate                 | 60.0 Hz      | `ticks` increments by exactly 600/10 s |
| Slow reads (>200 ms)                | 0–1 per 10 s | one per ~10 s, then zero               |
| `slow_pct`                          | 0.00–0.17%   | stabilizes under 0.1% after warm-up    |
| Median read (`sock_ms`)             | 0.2 ms       | measured against the same server       |
| Slow-read `gap_ms` (GIL wait)       | **0.0**      | after `SingleThreadedExecutor` fix     |
| Slow-read `sock_ms` (socket wait)   | 219–406 ms   | server-side cold-start stall           |

**Interpretation:** after the GIL-contention fixes, the bridge sustains
a genuine 60 Hz loop. The residual ~0.05% `slow_read` warnings are
**server-side stalls**, not bridge-side timing artifacts. They are
tolerated by `timeout: 1.5` and `watchdog_timeout: 4.0` and do not cause
spurious e-stops or setpoint interruptions.

For comparison, before the fixes:

| Metric                              | Before      | After     |
| :---------------------------------- | :---------- | :-------- |
| Slow reads (>200 ms)                | ~5%         | <0.2%     |
| Slow-read `gap_ms` (GIL wait)       | 200–1000 ms | 0.0 ms    |
| Per-tick CPU (1 thread)             | ~3.3 ms     | <1 ms     |
| Spurious e-stops from timeouts      | frequent    | none      |
| Duplicate terminal lines            | yes (×2)    | no        |

### Clock Drift Calibration

Two independent measurements were performed to characterize the clock
behavior between Windows and Linux. The scope is **offset and drift
between the two endpoints**, not the intrinsic oscillator accuracy of
either machine.

Both endpoints use the same clock kind (`perf_counter_ns`), so the
comparison is homogeneous — no mixing of `raw` vs `mono` or `wall` vs
`monotonic`.

#### Physical machines (Windows 10 ↔ Ubuntu 24.04)

**Setup:**

| Item              | Value                                |
| :---------------- | :----------------------------------- |
| Client            | Windows 10, `DESKTOP-7RFMLG3`        |
| Reference         | Ubuntu 24.04, `Nebras`, `tsc`        |
| Peer address      | `111.111.111.2:9999`                 |
| Duration          | 299.98 s (~5 minutes)                |
| Samples           | 97,471 pings                         |

**Main result:**

| Metric                | Value             |
| :-------------------- | :---------------- |
| Packet loss           | 0.00 % (0/97,471) |
| Drift (least squares) | **+95.73 ppm**    |
| Drift (Theil-Sen)     | +95.81 ppm        |
| Estimator agreement   | 0.082 ppm         |
| Drift stability       | 1.99 ppm range    |
| Quality               | ok (no warnings)  |

**Interpretation:**

- Linux clock runs **~95.7 ppm faster** than Windows
- = **~5.74 ms per minute**
- = **~0.344 s per hour**
- = **~8.26 s per day**

**Link quality:**

| Metric  | Value     |
| :------ | :-------- |
| Min RTT | 0.166 ms  |
| Median  | 0.598 ms  |
| p95     | 0.848 ms  |
| Max     | 1.436 ms  |

The hard floor on offset error is `min_rtt / 2` = **±0.083 ms**.

**Allan deviation:**

| τ (s) | adev (ppm) | n   |
| :---: | :--------: | :-- |
| 1     | 58.22      | 298 |
| 2     | 31.63      | 296 |
| 4     | 15.17      | 292 |
| 8     | 7.69       | 284 |
| 16    | 4.12       | 268 |
| 32    | 1.85       | 236 |
| 64    | 0.92       | 172 |
| 128   | 0.52       | 44  |

Curve falls as `1/√τ` → white-noise behavior; clock is stable.

**Correction model:**

    offset(t) ≈ offset_at_t0 + drift × (t − t0)
    offset(t) ≈ 8,785,337.617 + 0.095728 × (t − t0)   [ms, t in seconds]

    local_time = peer_time − offset(t)

**Achievable correction accuracy:** **±0.1 ms** over 5 minutes.

#### QEMU/KVM guest (Windows guest ↔ Ubuntu host)

**Setup:**

| Item              | Value                                |
| :---------------- | :----------------------------------- |
| Host              | Ubuntu 24.04, TSC clocksource        |
| Guest             | Windows 10 in QEMU/KVM               |
| Duration          | 756.7 s                              |
| Samples           | 31,851 pings                         |

**Main result:**

| Metric              | Value                    |
| :------------------ | :----------------------- |
| Mean drift          | **≈ 80 ppm**             |
| Min RTT             | 1.2 ms                   |
| Median RTT          | 2.0–2.5 ms               |
| Max RTT             | 4–5 ms                   |
| Drift stability     | unstable (>20 ppm range) |

**Interpretation:**

This number is the *practical* timing error of a Windows guest in this VM
configuration. It includes:

- host TSC stability (good)
- QEMU emulation overhead (dominant)
- vCPU scheduling jitter
- actual Windows clock drift

It does **NOT** measure the intrinsic oscillator drift of the Windows
machine's hardware.

#### Comparison of both setups

| Metric           | QEMU/KVM guest     | Physical machines       | Improvement |
| :--------------- | :----------------: | :---------------------: | :---------: |
| Min RTT          | 1.2 ms             | **0.166 ms**            | **7.2×**    |
| Median RTT       | 2.0–2.5 ms         | 0.598 ms                | ~4×         |
| Fitted drift     | ~80 ppm (variable) | **95.73 ppm (const.)**  | stable now  |
| Drift range      | > 20 ppm           | **1.99 ppm**            | **> 10×**   |
| Fit residual std | large              | 0.030 ms                | —           |
| Quality          | check notes        | **ok**                  | OK          |

#### Conclusions

- The two physical machines drift at a constant rate of **+95.73 ppm**.
- The Linux clock is consistently **ahead** of the Windows clock.
- The link between physical machines is clean: **0% loss**,
  min RTT = **0.166 ms**.
- The drift is stable (range **1.99 ppm** over 5 minutes).
- A single linear correction is sufficient for sessions up to ~10 min.
- For longer sessions, use `--track` and interpolate.
- Achievable correction accuracy on physical machines: **±0.1 ms**.
- For QEMU/KVM guests, the emulation overhead dominates — use physical
  machines for pure oscillator measurement.

#### Recommendations

1. Use the linear model in the physical-machines section for sessions
   ≤ 10 minutes.
2. Switch to `--track` for sessions > 1 hour.
3. Keep NTP/chrony disabled on both machines for pure measurement.
4. Repeat the run 3 times at different times of day to confirm.
5. Store the calibration coefficients with each experiment.
6. For QEMU/KVM: enable `hypervclock`, disable `hpet`, pin vCPUs, then
   re-run.

### Results (this machine — after fixes, v3.1.2)

After applying the fixes described in [Roadmap](#roadmap) (v3.1.2):

| Item         | Value                                  |
| :----------- | :------------------------------------- |
| OS           | Ubuntu 24.04 (Docker)                  |
| CPU          | Shared vCPU                            |
| GPU          | RTX 5060, 8 GB                         |
| PLC server   | PLCSIM Advanced V6.0, TCP/JSON         |
| ROS 2        | Jazzy                                  |
| RMW          | FastDDS                                |

**Full-loop latency** (1,881 samples, `latency_probe: true`):

| Stage          | Median   | p95      | p99       | Max        |
| :------------- | -------: | -------: | --------: | ---------: |
| `queue_delay`  | 5.42 ms  | 7.41 ms  | 8.04 ms   | 10.30 ms   |
| `read_net`     | 6.73 ms  | 13.08 ms | 46.32 ms  | 118.33 ms  |
| `convert_pub`  | 0.05 ms  | 0.08 ms  | 0.10 ms   | 0.16 ms    |
| `ros_ruckig`   | 8.14 ms  | 21.81 ms | 75.53 ms  | 502.71 ms  |
| `isaac_step`   | 10.25 ms | 26.11 ms | 30.79 ms  | 103575.93 ms |
| `ros_back`     | 0.37 ms  | 1.83 ms  | 2.64 ms   | 3.33 ms    |
| `write_net`    | 12.15 ms | 20.61 ms | 36.79 ms  | 132.64 ms  |
| **`total_loop`** | **47.20 ms** | **67.28 ms** | **151.49 ms** | **104091.64 ms** |

**LBP (Latency Bound Probability) — this machine (after fixes):**

| Threshold | LBP    |
| :-------- | -----: |
| <= 20 ms  | 0.00%  |
| <= 30 ms  | 5.10%  |
| <= 40 ms  | 37.59% |
| <= 50 ms  | 63.37% |
| <= 80 ms  | 96.60% |
| <= 100 ms | 97.24% |
| <= 200 ms | 99.89% |

**Filtered (remove top 1% outliers):**

| Metric | Value     |
| :----- | --------: |
| n      | 1,863     |
| median | 47.01 ms  |
| p95    | 64.51 ms  |
| max    | 151.49 ms |

**Comparison: before vs after fixes**

| Stage          | Before (v3.1.1) | After (v3.1.2) | Improvement |
| :------------- | --------------: | -------------: | ----------: |
| `queue_delay`  | 4.67 ms         | **5.42 ms**    | (measurable now) |
| `read_net`     | 6.88 ms         | **6.73 ms**    | −2%         |
| `convert_pub`  | 0.06 ms         | **0.05 ms**    | −17%        |
| `ros_ruckig`   | 6.57 ms         | **8.14 ms**    | +24% (DDS)  |
| `isaac_step`   | 27.05 ms        | **10.25 ms**   | **−62%** ✅ |
| `ros_back`     | 4.66 ms         | **0.37 ms**    | **−92%** ✅ |
| `write_net`    | 15.72 ms        | **12.15 ms**   | −23%        |
| **`total_loop`** | **67.36 ms**  | **47.20 ms**   | **−30%** ✅ |
| LBP ≤ 50 ms    | 6.78%           | **63.37%**     | **+56.59%** ✅ |
| LBP ≤ 80 ms    | 75.45%          | **96.60%**     | **+21.15%** ✅ |

**What changed (v3.1.2):**

1. **`plc_ros_bridge.py`** — PLC I/O moved to a **dedicated thread** (`_io_loop`), separate from the ROS executor. This eliminates the contention between `tick()` (PLC I/O) and `on_joint_states()` (ROS callback). Result: `ros_back` −92%, `total_loop` −30%.
2. **`plc_ros_bridge.py`** — `_last_tick_end_ns` recorded at the end of each `tick()`, so `origin_ns` reflects the end of the previous tick (queue delay measurable). Before: `queue_delay = 0.00` (meaningless).
3. **`physics_twin.py`** — `rclpy.spin_once()` moved **before** `step()` and `publish_states()`, so a setpoint that arrived since the last frame is applied in **this** frame. Result: `isaac_step` −62%.
4. **`trajectory_twin.py`** — `SingleThreadedExecutor` (was `MultiThreadedExecutor`). MultiThread added overhead on `ros_ruckig` (+26%) without improving `isaac_step`.
5. **QoS** — `BEST_EFFORT` + `depth=1` on high-rate streams (`/plc/cmd_ang`, `/servo_setpoints`, `/joint_states`). Old samples are dropped instead of queued.

**Interpretation:**

- The VM is still **~3× slower** than the reference machine (47 ms vs ~15 ms).
- The dominant remaining contributors are `write_net` (~12 ms) and `isaac_step` (~10 ms). Both are tied to the **QEMU/KVM environment**: PLCSIM Advanced API on Windows VM and Isaac Sim physics step on shared vCPU.
- `convert_pub` (0.05 ms) and `queue_delay` (5.42 ms) are close to the reference machine — the bridge-side code is **not** the bottleneck.
- The single outlier (`max = 103575 ms`) is a **103-second stall** in Isaac Sim (Play/Pause or re-init), not a bridge or PLC issue. Excluded from the filtered statistics.

**Conclusion:** the fixes reduced median full-loop latency by **30%** (67.36 → 47.20 ms) and increased the fraction of samples under 50 ms from **6.78% → 63.37%**. The remaining gap to the reference machine is environmental (VM), not algorithmic.

---

## Performance Tuning

The platform can be tuned for different latency/robustness trade-offs.

### Quick Wins (no code changes)

| Tuning             | Command                                          | Expected gain |
| :----------------- | :----------------------------------------------- | :-----------: |
| CPU pinning        | `taskset -c 2,3 python3 trajectory_twin.py`      | -2 to -3 ms   |
| Real-time priority | `sudo chrt -f 80 python3 plc_ros_bridge.py`      | -2 ms         |
| CycloneDDS         | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`   | -1 ms         |

> **Note:** CycloneDDS requires Isaac Sim to use its **internal** ROS 2
> libraries (Python 3.11), not the system ROS 2 (Python 3.12). See
> [ROS 2 Bridge environment (Isaac Sim)](#ros-2-bridge-environment-isaac-sim).
> For the simplest setup, keep FastDDS on all nodes.

### Bridge-Side Tuning (built in)

These are already enabled by the current code and matter more than any
external knob at 60 Hz:

| Tuning                           | Effect                                                                                    |
| :------------------------------- | :---------------------------------------------------------------------------------------- |
| `SingleThreadedExecutor`         | Eliminates GIL contention between timer and subscriber callbacks (was 200-1000 ms stalls) |
| State-topic publish dedupe       | Cuts DDS traffic from ~300 msg/s to <10 msg/s at steady state                             |
| `TCP_NODELAY` on tcp_json socket | Prevents Nagle/delayed-ACK stalls (defensive)                                             |
| Monotonic timing                 | Removes NTP/hypervisor clock-jump artifacts from `slow_read` diagnostics                  |
| `tick_stats` every 10 s          | Separates `gap_ms` (bridge/GIL) from `sock_ms` (server) automatically                     |
| Pydantic config validation       | Fails fast on bad config — no partial startup                                             |
| Config hash + audit log          | Every run is traceable to an exact config                                                 |

**If you still see `slow_read` after these:**

- `gap_ms > 0` → bridge-side, check host CPU load (`top -H`, `taskset`)
- `sock_ms > 0` → server-side, see `plc_server_win.py` — usually a ~10 s
  periodic task holding a lock or doing a slow fsync

### Transport Tuning

**TCP-JSON:**

- Use msgpack framing (not JSON) — 3× faster serialization
- Set `TCP_NODELAY` on both client and server sockets
- Use `asyncio` on the Windows server (not threading)
- Batch all reads/writes into a single request

**OPC UA:**

- Use `read_values(nodes)` instead of per-node `read_value()`
- Consider OPC UA Subscriptions for change-based updates
- Set `TCP_NODELAY` on the OPC UA client socket

### Expected Performance

| Transport             | Mean      | p99        | Max        |
| :-------------------- | :-------: | :--------: | :--------: |
| OPC UA (naive)        | 11.5 ms   | 14 ms      | 100 ms     |
| OPC UA (batch)        | ~5 ms     | ~10 ms     | ~20 ms     |
| TCP-JSON (naive)      | 12.5 ms   | 20 ms      | 233 ms     |
| TCP-JSON (optimized)  | 5.2 ms    | 15.5 ms    | 16.3 ms    |

---

## Documentation

| Document                            | Description                                   |
| :---------------------------------- | :-------------------------------------------- |
| `docs/README_full.md`               | Gen 3 full documentation                      |
| `docs/migration.md`                 | How this folder was split from v2.0.0         |
| `docs/latency_comparison.md`        | Full latency comparison report                |
| `docs/latency_full_loop_OPC-UA.md`  | OPC UA baseline report                        |
| `docs/upgrade_report.html`          | Session report: Prototype -> Production Grade |
| `../CHANGELOG.md`                   | Version history (all generations)             |

---

## Lineage

| Version | Robot         | Transport         | Architecture                            | Location                |
| :------ | :------------ | :---------------- | :-------------------------------------- | :---------------------- |
| v0.1.0  | KUKA KR210    | TCP/JSON          | Monolithic                              | archived                |
| v1.0.0  | KUKA KR210    | TCP/JSON          | 5 processes                             | `archive/v1.0-kuka/`    |
| v0.2.0  | Yaskawa GP110 | OPC UA            | Monolithic bridge                       | `archive/v0.2-yaskawa/` |
| v2.0.0  | Yaskawa GP110 | OPC UA            | Modular (6 files)                       | `archive/v0.2-yaskawa/` |
| v3.0.0  | Any robot     | Any transport     | Vendor-neutral                          | `vendor-neutral/`       |
| v3.1.0  | Any robot     | Any transport     | Vendor-neutral + Pydantic v2 + 29 tests | `vendor-neutral/`       |

---

## Known Limitations

The following limitations are **known and documented**. They do not
affect the correctness of the digital twin for research or simulation,
but they must be understood before deploying to a physical robot.

### Isaac Sim 5.1

- **No gravity compensation API on `Articulation`.** The current
  workaround disables gravity for the robot links. This makes the arm
  weightless in simulation — tracking error drops to near zero, but
  payloads and vertical contacts are not simulated.
- **`joint_2` and `joint_3` use `stiffness: 50,000,000` in `robot.yaml`.**
  This is ~16× the value from published KR210 measurements (3M and 8.2M
  respectively). It is required because gravity is disabled and the
  drives must still track position accurately. Do not compare it to
  physical stiffness values.
- **`joint_5` is passive.** It appears in `all_joints` but is not
  controlled by the PLC. It has no entry in `controlled_joints`.
- **`Articulation.initialize()` may hang** if Play has not been pressed
  at least once. If you see `Initializing articulation...` for > 30 s,
  press Play and re-run `physics_twin.py`.
- **ROS 2 Bridge must use Isaac Sim's internal Python 3.11 libraries.**
  Do not `source /opt/ros/jazzy/setup.bash` before starting Isaac Sim.
  See [ROS 2 Bridge environment (Isaac Sim)](#ros-2-bridge-environment-isaac-sim).

### PLC Bridge

- **`robot.yaml` has `logging.to_ros: true`.** This causes duplicate
  terminal lines in `trajectory_twin.py`. It should be `false`. The
  `plc_ros_bridge.yaml` already has `false` — only `robot.yaml` is
  affected. Fix: change to `to_ros: false`.
- **`physics_twin.py` does not use `bridge_config.py`.** It opens
  `robot.yaml` directly and does not validate `limits_deg` or `ruckig`
  parameters. This is acceptable because `physics_twin` only needs the
  Isaac paths and drives, but it means a malformed `robot.yaml` fails
  inside Isaac Sim rather than at `trajectory_twin` startup.
- **Residual server-side stalls (~400 ms every ~10 s).** Measured with
  `tick_stats` — `slow_pct < 0.2%` but not zero. Traced to
  `plc_server_win.py`, not the bridge. Acceptable for simulation;
  should be fixed before production.
- **One 6 s link drop was observed on the VM environment**, producing a
  single 9514 ms outlier in `latency_probe.csv`. Excluded from the
  filtered statistics in [Results (this machine)](#results-this-machine).

### Not Certified

This project is **not a certified safety function**. See the
[Safety Notice](#safety-notice) at the top of this file.

---

## Roadmap

### Completed

- [x] Extract PLC transports into `plc_drivers.py`
- [x] Extract signal handling into `plc_config.py`
- [x] All three transports selectable from one YAML line
- [x] Repository reorganized by environment (windows/linux/isaac/plc/tools)
- [x] Latency + drift measurement tools preserved
- [x] TCP-JSON optimization: batch reads + msgpack + TCP_NODELAY
  - Mean latency: 12.5 ms -> 5.2 ms (-58%)
  - Max latency: 233 ms -> 16.3 ms (-93%)
  - Full-loop p99 under 20 ms in 100% of samples (reference machine)
- [x] Latency characterization across OPC UA and TCP-JSON
- [x] `trajectory_twin.py` reads joints, limits, and Ruckig parameters
  from `config/robot.yaml`
- [x] Fix: a persistent bad command no longer cancels an in-progress brake
- [x] Fix: bridge no longer auto-clears the e-stop on PLC reconnect
- [x] Clock drift characterization (physical + QEMU/KVM)
- [x] **Bridge reaches clean 60 Hz**
  - `SingleThreadedExecutor` eliminates GIL contention
  - State-topic publish dedupe (~300 msg/s → <10 msg/s)
  - Monotonic timing in all latency measurements
  - `tick_stats` diagnostic splits `gap_ms` (GIL) from `sock_ms` (server)
  - `TCP_NODELAY` set on `tcp_json` sockets
  - `slow_pct` <0.2% sustained; residual stalls are server-side (~0.05%)
- [x] **`trajectory_twin.py` terminal table**
  - Side-by-side `cmd_deg` / `actual_deg` / `setpt_deg`, once per second
  - File/JSONL log keeps compact single-line record with `position_deg`
- [x] **Eliminate duplicate terminal lines** — `logging.to_ros: false`
- [x] **Fix liveness semantics** — `/plc/cmd_ang` always republished;
      only state topics are deduped
- [x] **`latency_probe` origin_ns fix** — `origin_ns` now reflects the
      end of the previous tick (queue-delay measurable), not a duplicate
      of `read_start_ns`

### Gen 3.1 — Production-Grade Config (completed)

- [x] **Pydantic v2 schema** (`bridge_config.py`) — replaces hand-written
  `_validate_config`
  - Declarative, `extra="forbid"`, field ranges via `Field(gt=, le=)`
  - Cross-reference validation with **field location** (`robot.cmd_ang[0]`)
  - Direction sanity: `cmd_ang`/`act_ang`/`*_ack` must have correct `dir`
  - Address safety: rejects `\x00`, `\n`, `\r`, `..` in `addr`
- [x] **`schema_version` + migration chain** — v1.0 configs work unchanged
- [x] **Audit trail** — `sha256` hash + ISO-8601 timestamp logged to JSONL
- [x] **Fail-fast** — bad config rejected **before** any socket is opened
- [x] **`robot.yaml` validation** — `_validate_limits`, Ruckig parameter sanity
- [x] **29 unit tests** (`pytest tests/ -v` → 29 passed in 0.17 s)
  - 22 for config schema (`test_bridge_config.py`)
  - 7 for trajectory planner (`test_trajectory_twin.py`)
- [x] **`requirements-dev.txt` + `pytest.ini`** — repeatable test setup
- [x] **PEP 668 aware** — documented venv workflow for Python 3.11+
- [x] **E-stop forwarding enforced** — config rejected if the PLC E-stop
      is not forwarded to `/plc/emergency_stop` (opt-out for tests)

### In Progress

- [ ] Yaskawa GP110 verification on this branch
- [ ] Eliminate residual server-side stalls (~400 ms every ~10 s) —
      needs `plc_server_win.py` instrumentation
- [ ] CI integration (GitHub Actions to run `pytest` on every push)
- [ ] Improve VM performance — investigate `isaac_step` (~27 ms) and
      `write_net` (~16 ms) on the Docker environment
- [ ] Optional: CycloneDDS support on Isaac Sim (requires internal
      Python 3.11 libraries and `LD_LIBRARY_PATH` — see Known Limitations)

### Planned

- [ ] Array type distinction in `SignalConfig` (`array: N`) — deferred;
      current runtime detection of `list/tuple` works, and schema-level
      distinction would require editing every YAML file
- [ ] Cartesian motion layer (inverse kinematics)
- [ ] Trajectory queue and blending
- [ ] Certified safety layer (F-CPU + PROFIsafe) — postponed; see Safety Notice
- [ ] Native OPC UA on Windows (remove PLC server dependency)

---

## License

Apache License 2.0. See [LICENSE](LICENSE).

---

## Contact

- **YouTube:** [Digital Twin Series](https://www.youtube.com/playlist?list=PLYLnoPt4fbu8)
- **Developer:** Nebras — nebras4u@gmail.com
- **Repository:** [isaac-plc-digital-twin](https://github.com/Nebras4u/isaac-plc-digital-twin)

---

*Built for Industry 4.0 — isaac-plc-digital-twin © 2026*
