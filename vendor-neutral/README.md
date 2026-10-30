# Isaac Digital Twin — Vendor-Neutral Platform

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23312010.svg)](https://doi.org/10.5281/zenodo.23312010)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen)](CHANGELOG.md)
[![Tests](https://img.shields.io/badge/Tests-86%20pytest%20%2B%209%20standalone-success)](#testing)
[![ORCID](https://img.shields.io/badge/ORCID-0009--0001--2333--1502-a6ce39?logo=orcid&logoColor=white)](https://orcid.org/0009-0001-2333-1502)

A **vendor-neutral Digital Twin platform** for industrial robot arms.
The platform synchronizes any robot arm with any PLC through **ROS 2**,
using a transport-agnostic bridge and a robot-agnostic YAML configuration.

This folder (`vendor-neutral/`) is the evolution of two robot-specific
implementations — KUKA KR210 ([`../kuka/`](../kuka/)) and Yaskawa GP110
([`../yaskawa/`](../yaskawa/)) — into a reusable, configuration-driven
platform.

---

![S7-1500T ↔ ROS2 ↔ Isaac Sim Architecture and latency budget](https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/vendor-neutral/docs/architecture_and_latency_budget.png)

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
  - [Schema v2.2 — Pydantic validation](#schema-v22--pydantic-validation-with-per-signal-typerange-policy)
  - [Signal mapping](#configplc_ros_bridgeyaml--signal-mapping)
  - [Robot description](#configrobotyaml--robot-description)
- [Testing](#testing)
- [Results](#results)
  - [Tracking Accuracy](#tracking-accuracy)
  - [Control-Loop Latency (reference)](#control-loop-latency-reference)
  - [Bridge Real-Time Behaviour](#bridge-real-time-behaviour)
  - [Clock Drift Calibration](#clock-drift-calibration)
  - [Results (this machine — detailed, v3.1.2)](#results-this-machine--detailed-v312)
  - [v3.1.3 — PLC batch mode + 120 Hz tuning](#v313--plc-batch-mode--120-hz-tuning-this-machine)
  - [v3.7 — Probe observability + production path](#v37--probe-observability--production-path-this-machine)
  - [v4.0 — Industrial Type-Safety](#v40--industrial-type-safety-this-machine)
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

| Factor         | Reference machine                          |
| :------------- | :----------------------------------------- |
| OS             | Native Ubuntu 24.04                        |
| CPU            | Dedicated cores (no throttling)            |
| GPU            | NVIDIA, no passthrough                     |
| PLC server     | Native Windows, low-latency                |
| Isaac Sim      | Native, 60 FPS stable, **gravity ENABLED** |
| ROS 2          | Jazzy                                      |
| RMW            | FastDDS                                    |
| `tcp_json`     | msgpack + `TCP_NODELAY` + batch            |

> **Note on v3.1.3 and later:** The reference numbers above were
> measured on the reference machine with batch enabled at **60 Hz**.
> v3.1.3 fixed batch mode on the Windows server (`_TYPE_NAMES` mapping)
> and raised `poll_hz` to 120. Those changes were measured on the
> **Docker/VM**, **not** on the reference machine. The reference
> machine has **not been re-measured** for v3.1.3 or later. See
> [v3.1.3](#v313--plc-batch-mode--120-hz-tuning-this-machine),
> [v3.7](#v37--probe-observability--production-path-this-machine), and
> [v4.0](#v40--industrial-type-safety-this-machine) for VM results,
> which use a different metric (`total_loop`/`cmd_to_feedback` p50)
> and are not comparable to the mean/max shown above.

The **same code** on a **Docker / VM environment** (RTX 5060, shared
vCPUs, PLCSIM Advanced over TCP/JSON) gives different numbers — see
the historical sections. This is expected and does not indicate a
regression.

**How to measure on your machine:**

1. Set `latency_probe: true` in `config/plc_ros_bridge.yaml`.
2. Run the bridge for ≥ 30 s.
3. Analyze `latency_probe.csv` with `ros2/analyze_probe.py`, or use
   the inline snippet in
   [Results (this machine — detailed, v3.1.2)](#results-this-machine--detailed-v312).

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
**audit trail**, and a pytest suite covering the main structural rules.
A bad config fails **before** the bridge touches the PLC.

**Since Gen 4.0 (v4.0.0)**, the platform enforces **industrial type
safety**: per-signal `min`/`max` range, `strict_types` validator mode,
wire-level type checks in every driver, a boot-time type handshake,
per-signal rejection counters, and NaN/Inf rejection across all four
nodes. This closes the "silent coercion" class of bugs (`int(3.9) → 3`,
`float("nan")` passing through, `int → bool`, etc.) at the cost of
**+0.01 ms p99** on the one stage that actually validates
(`convert_pub`).

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
                        |  (+ bridge_config.py)   |   <-- Pydantic v2 schema (v2.2)
                        |  (+ plc_drivers.py)     |       + audit trail
                        |  (+ plc_config.py)      |       + versioning
                        |                         |       + type handshake (v4.0)
                        +------------+------------+
                                     |
                                ROS 2 topics
                                     |
                        +------------v------------+
                        |linux/trajectory_twin.py |
                        | (Ruckig + state machine)|
                        | + NaN/Inf guards (v4.0) |
                        +------------+------------+
                                     |
                        +------------v------------+
                        |  isaac/physics_twin.py  |
                        |  (inside Isaac Sim)     |
                        | + NaN/Inf guards (v4.0) |
                        +-------------------------+

### Runtime threading model

**Since v3.1.2**, `plc_ros_bridge.py` separates PLC I/O from ROS
callbacks:

- `tick()` (PLC read/write) runs in `_io_loop` on a **dedicated thread**.
- `on_joint_states()` (ROS subscriber) runs on a
  **`SingleThreadedExecutor`** and is delivered immediately, without
  waiting for a PLC read to finish.
- `_last_tick_end_ns` is recorded at the end of each `tick()` so the
  probe can measure the real `queue_delay`.

**Why it changed:** up to v3.1.1, `tick()` and `on_joint_states()` shared
the same `SingleThreadedExecutor`. That serialized the two callbacks and
avoided the GIL races that had caused 200–1000 ms `read` stalls on VMs,
but a slow PLC read (6–7 ms) delayed `/joint_states` delivery and
inflated `ros_back` to ~4.7 ms. Moving the I/O out of the executor
reduced `ros_back` by 92% (see
[Results (this machine — detailed, v3.1.2)](#results-this-machine--detailed-v312)).

**What to watch:** the two threads now share the GIL again, so keep an
eye on `avg_slow_gap_ms` in the `tick_stats` line (see
[Terminal output](#terminal-output)). A non-zero value means bridge-side
GIL contention.

`trajectory_twin.py` also uses a **`SingleThreadedExecutor`** (since
v3.1.2). `physics_twin.py` calls `rclpy.spin_once()` at the **start** of
each frame, before `step()` and `publish_states()`, so a setpoint that
arrived since the last frame is applied in the same frame.

**Since v4.0**, all four nodes reject non-finite values at their
boundaries. A `NaN` in `/joint_states` no longer silently freezes a
joint (its `deadband` comparison would return `False` forever); it
triggers `joint_angles_non_finite` and skips the `act_ang` write for
that tick.

---

## Repository Structure

    vendor-neutral/
    |
    +-- README.md                         # This file
    +-- CHANGELOG.md                      # Gen 4 history
    +-- requirements-dev.txt              # Pydantic + pytest
    +-- pytest.ini                        # Test configuration
    |
    +-- windows/
    |   +-- plc_server_win.py             # PLCSIM Advanced -> TCP/JSON (v4.0: batch + types + NaN guard)
    |   +-- Run.txt                       # Quick-start usage notes
    |
    +-- linux/
    |   +-- trajectory_twin.py            # Ruckig trajectory + state machine (v4.0: NaN/Inf guards)
    |   +-- plc_ros_bridge.py             # ROS 2 <-> PLC bridge (v4.0: validator-mode _get, type handshake, counters)
    |   +-- bridge_config.py              # Pydantic v2 schema v2.2 + migration + audit
    |   +-- plc_drivers.py                # 3 transports (tcp_json/opcua/plc_api) with _wire_type_check
    |   +-- plc_config.py                 # signal mapping + extractor
    |   +-- bridge_logger.py              # Structured logging
    |
    +-- ros2/
    |   +-- analyze_probe.py              # Offline CSV analysis
    |   +-- test_probe_chain.py           # 9 standalone probe tests (v4.0)
    |
    +-- tests/
    |   +-- conftest.py
    |   +-- test_bridge_config.py         # 33 tests: schema v2.2 + migration + safety
    |   +-- test_physics_twin.py          # 4 tests: JointMapper
    |   +-- test_trajectory_twin.py       # 27 tests: Ruckig + NaN guards
    |   +-- test_industrial_patch.py      # 12 tests: act_ang policy, estop lock, deadline, physics latch
    |
    +-- isaac/
    |   +-- physics_twin.py               # Articulation driver (v4.0: NaN/Inf guards)
    |   +-- Main_Robot_L_URDF_tool.urdf
    |   +-- Full_Control.usd
    |
    +-- config/
    |   +-- plc_ros_bridge.yaml           # Signal mapping (schema_version: 2.2)
    |   +-- robot.yaml                    # Joints, limits, Ruckig, drives, warn_cooldown
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
3. **Keeping gravity ENABLED** (real physics). Isaac Sim 5.1 does not
   expose a gravity-compensation API on `Articulation`, so compensation
   is achieved implicitly through the high drive stiffness configured in
   `robot.yaml`. The arm is **not** weightless — it holds position under
   real gravity because the drives are stiff enough to counteract it.
4. Publishing `/joint_states` at 60 Hz (default; `DEFAULT_PUBLISH_HZ = 120.0`
   in v3.7+ for tighter latency).

**Gravity handling:** gravity is **kept enabled** throughout the
simulation. There is no `disableGravity=True` and no
`disable_gravity_for_robot_links()` call. Tracking accuracy measured
under full gravity load is reported in
[Tracking Accuracy](#tracking-accuracy) (~0.0254° max on `joint_2`).
This reflects real tracking performance, not a weightless simulation.

**Recovery from Stop/Play:** PhysX invalidates simulation views whenever
the timeline is stopped or the stage is modified. `physics_twin.py`
detects this via `_needs_reinit` and rebuilds the `Articulation`
automatically — no manual restart required.

**Startup order (important):**

1. Open Isaac Sim (do **not** press Play yet).
2. Run `physics_twin.py` in the Script Editor.
3. Wait for `PhysicsTwin ready. DOF=... gravity=ENABLED`.
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

### Schema v2.2 — Pydantic validation with per-signal type/range policy

`bridge_config.py` defines the config schema declaratively with Pydantic
v2. Version history:

| Schema   | Introduced | Highlights |
| :------- | :--------- | :--------- |
| v1.0     | v3.0.0     | Hand-written validator |
| v2.1     | v3.1.0     | Pydantic v2, `extra="forbid"`, `Field(gt=, le=)`, sha256 audit trail, versioning |
| **v2.2** | **v4.0.0** | **Per-signal `min`/`max`/`strict`/`reject_non_finite`/`allow_int_for_float`, `strict_types`, `strict_type_handshake`, `publish[].type` cross-check** |

Benefits over the hand-written v1.0 validator:

| Aspect                 | Before (v1.0)                           | After (v2.2)                                                   |
| :--------------------- | :-------------------------------------- | :------------------------------------------------------------- |
| Validation code        | ~60 lines, hand-written                 | Declarative Pydantic model                                     |
| Unknown fields         | Silently ignored                        | **Rejected** (`extra="forbid"`)                                |
| Numeric ranges         | Not checked                             | `Field(gt=, le=)` + per-signal `min`/`max`                     |
| Error messages         | `signal 'x' is used but not defined`    | `signal 'x' referenced in 'robot.cmd_ang[0]' …`                |
| Cross-reference        | Manual, no context                      | Automatic, with **field location**                             |
| Direction sanity       | Partial                                 | Every `cmd_ang`/`act_ang`/`ack` checked                        |
| Address safety         | None                                    | Rejects `\x00`, `\n`, `\r`, `..`                               |
| Versioning             | None                                    | `schema_version` + migration chain (v1.0/v2.0/v2.1 → v2.2)     |
| Audit trail            | None                                    | `sha256` hash + timestamp in JSONL                             |
| E-stop forwarding      | None                                    | Enforced at config load (opt-out for tests)                    |
| Signal type policy     | None                                    | `min`/`max`/`strict`/`reject_non_finite`/`allow_int_for_float` |
| Publish type check     | None                                    | `publish[].type` must equal `signals[].type`                   |
| Unit tests             | 0                                       | **86** (plus 9 standalone probe tests)                         |

**Backward compatibility:** configs without `schema_version` are treated
as v1.0 and migrated to v2.2 in-memory. v2.0 and v2.1 configs are also
migrated in-memory. No manual edits required.

**Fail-fast guarantee:** a bad config is rejected **before** the bridge
attempts to open a socket. There is no possibility of starting the bridge
with invalid signals, ranges, or cross-references.

### `config/plc_ros_bridge.yaml` — Signal mapping

    schema_version: "2.2"       # v1.0/v2.0/v2.1 files migrate in-memory

    latency_probe: true         # enable per-tick CSV timing probe

    plc:
      driver: tcp_json               # opcua | tcp_json | plc_api
      endpoint: opc.tcp://192.168.100.50:4840   # opcua only
      host: 192.168.100.18                      # tcp_json only
      port: 5000                                # tcp_json only
      timeout: 1.5                              # all drivers — see note below

    poll_hz: 200
    watchdog_timeout: 4.0

    strict_types: true              # v4.0: validator mode (industrial default)
    strict_type_handshake: true     # v4.0: abort connect on type mismatch

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
      cmd_ang:
        addr: 'MainRobotLeft_CmdAng'
        type: float
        dir: read
        min: -360.0
        max:  360.0

      act_ang:
        addr: 'MainRobotLeft_ActAng'
        type: float
        dir: write
        min: -360.0
        max:  360.0

      hb_counter:
        addr: 'Heartbeat_Counter'
        type: int
        dir: read
        min: 0
        max: 2147483647

      hb_ack:
        addr: 'ROS_Ack_Counter'
        type: int
        dir: write
        min: 0
        max: 2147483647

      cmd_ack:
        addr: 'MainRobotLeft_CmdAck'
        type: bool
        dir: write

      axis_enable:
        addr: 'MainRobotLeft_CmdEnable'
        type: bool
        dir: read

      estop:
        addr: 'MainRobotLeft_Emergency_Stop_Active'
        type: bool
        dir: read

      velocity_scale:
        addr: 'MainRobotLeft_Vlcty'
        type: float
        dir: read
        min: 0.0
        max: 1000.0

    publish:
      - {topic: /plc/emergency_stop, signal: estop,          type: bool}
      - {topic: /plc/axis_enable,    signal: axis_enable,    type: bool}
      - {topic: /plc/velocity_scale, signal: velocity_scale, type: float}

    heartbeat:
      counter: hb_counter
      ack: ros_ack_counter

    robot:
      joints:  [joint_1, joint_2, joint_3, joint_4]
      cmd_ang: [cmd_ang]     # one array signal (all joints)
      act_ang: [act_ang]
      cmd_ack: cmd_ack

**Schema v2.2 rules enforced at startup:**

- `plc.driver` must be one of `tcp_json`, `opcua`, `plc_api`.
- `tcp_json` requires `plc.host`; `opcua` requires `plc.endpoint`.
- `poll_hz ∈ (0, 1000]`; `watchdog_timeout ∈ (0.1, 60]`; `backoff_max ∈ (0, 600]`.
- `signals` must be non-empty.
- Every signal referenced in `robot`, `publish`, `heartbeat` must exist in
  `signals`.
- `robot.cmd_ang` signals must have `dir: read`; `act_ang` and `*_ack`
  must have `dir: write`.
- `publish[].signal` must have `dir: read`.
- **`publish[].type` must equal `signals[].type`** (v2.2).
- `publish` must forward the PLC E-stop to `/plc/emergency_stop` unless
  `estop_forwarding_required: false` is set (test/simulation only).
- `addr` must not contain `\x00`, `\n`, `\r`, or `..`.
- **`min`/`max` must be finite, `min < max`, and only on `int`/`float`
  signals** (v2.2).
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
`/plc/emergency_stop`. At 200 Hz with 200–400 ms server stalls, 4 s is
the first threshold that does not produce false positives.

**Why `strict_types: true`?** (v4.0) In this mode, `_get` is a
**validator**: it rejects `str → int` (`int("5")`), `float → int`
truncation (`int(3.9) → 3`), `int → bool` (`2 → True`), `NaN`/`Inf` on
`float` signals, and out-of-range values. Rejections surface as
`signal_rejected` log entries and increment
`signal_rejections_total` on the `tick_stats` line. Set to `false` for
transitional deployment: `_coerce_legacy` restores v3.7 behaviour but
still applies `_to_int_checked` (no silent truncation) and
`_to_finite_float` (no silent NaN).

**Why `strict_type_handshake: true`?** (v4.0) On connect, the bridge
sends `{"cmd":"types"}` to `plc_server_win.py` v4.0 and compares the
declared types against the config. A mismatch logs
`config_type_mismatch` and, with the flag set, aborts the connection.
Requires a v4.0 server; against v3.3 the bridge logs
`type_handshake_unavailable` and continues.

**Audit trail:** on startup, the bridge logs `config_loaded` with the
`sha256` of the canonicalised config, the `schema_version`, and an
ISO-8601 UTC timestamp. This is appended to both the human log and the
JSONL file:

    {"event":"config_loaded","hash":"751ed1fb...729ed5","schema_version":"2.2",
     "timestamp_utc":"2026-10-29T11:57:02.810316+00:00",
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

**QoS (since v3.1.2):** the high-rate streams `/plc/cmd_ang`,
`/servo_setpoints`, and `/joint_states` use `BEST_EFFORT` with
`depth=1`. Old samples are dropped instead of queued. `RELIABLE`
publishers remain compatible with `BEST_EFFORT` subscribers, so no
migration is needed.

### `config/robot.yaml` — Robot description

> **Note:** The Isaac Sim URDF (`isaac/Main_Robot_L_URDF_tool.urdf`) is a
> **4-axis approximation** of a KUKA KR210 L150 (6-axis). Links L3+L4 and
> L5+L6 are merged, masses are scaled to a realistic total of **1410 kg**,
> and `joint_4` origin matches the PLC tool length (`LF = 89.084 mm`).
> `joint_5` is present in the URDF (passive physics) but not in
> `controlled_joints` (not commanded by the PLC). The same code works on
> the original 6-axis URDF by editing `robot.yaml` only.

Used by `trajectory_twin.py` (and the Isaac physics twin): joints, limits
(`limits_deg`), Ruckig limits (`ruckig`), drive gains, and Isaac prim paths.
`all_joints` includes the passive joint (`joint_5`); `controlled_joints`
are the axes commanded by the PLC. Since v4.0, `warn_cooldown` also lives
here (throttles per-joint rejection warnings in `trajectory_twin.py` and
`physics_twin.py`).

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

**Probe stats (since v3.1.3, extended v3.7/v4.0):** if
`latency_probe: true`, the same `tick_stats` line also carries:

    probe_sent=1800 probe_attempted=136 probe_rows=136
    probe_rejected=0 probe_order_violations=0 signal_rejections_total=0

- `probe_sent` — number of probe tags emitted by the bridge (`/plc/cmd_ang`)
- `probe_attempted` — total `record()` calls (all sources; `>= probe_sent`)
- `probe_rows` — number of CSV rows successfully written
- `probe_rejected` — malformed tags (should stay 0)
- `probe_order_violations` — timestamps not monotonic along the chain
  (should stay 0)
- `signal_rejections_total` (v4.0) — cumulative count of rejected signal
  values. On any rejection the line also carries
  `signal_rejections={...}` with the per-signal breakdown.

---

## Testing

### Running the test suite

    # From the vendor-neutral folder
    cd /path/to/vendor-neutral
    python3 -m pytest test_bridge_config.py test_physics_twin.py \
                     test_probe_chain.py test_trajectory_twin.py \
                     test_industrial_patch.py -q

Expected output:

    ============================== 86 passed in ~1.1s ==============================

Breakdown (approximate):

- `test_bridge_config.py` — 33 tests (schema v2.2: `min`/`max`, `strict`,
  publish type cross-check, estop rule)
- `test_physics_twin.py` — 4 tests (JointMapper)
- `test_probe_chain.py` — 9 tests (probe + `_bind_physics_v4_helpers`)
- `test_trajectory_twin.py` — 27 tests (Ruckig + non-finite rejection)
- `test_industrial_patch.py` — 12 tests (act_ang write policy, estop lock,
  io_loop deadline, physics estop latch)

To list the collected tests without running them:

    python3 -m pytest test_*.py --collect-only -q

### Standalone probe tests (not pytest)

`test_probe_chain.py` runs standalone as well (its `__main__` block):

    python3 test_probe_chain.py
    # Expected: 9/9 passed

### What is tested

The 86 pytest tests cover the **main structural rules** enforced by
the schema, the Ruckig planner, and the industrial patch (act_ang
write policy, estop lock, deadline counting, physics estop latch).
Each test maps to a specific industrial risk.

**`test_bridge_config.py` — 33 tests**

Covers: minimal valid config, v1.0 migration, missing driver, unknown
field rejection, empty signals, `poll_hz` range, driver-specific
requirements (host/endpoint), cross-reference with location, direction
sanity, address safety (`\x00`/`\n`/`\r`/`..`), estop forwarding rule
(required/opt-out/present), and the v2.2 additions:
`signal_min_max_accepted`, `signal_min_gt_max_rejected`,
`signal_min_on_non_numeric_rejected[bool|string]`,
`signal_min_non_finite_rejected`, `publish_type_must_match_signal`,
`strict_types_default_true`, `strict_type_handshake_opt_in`,
`signal_strict_override`, `reject_non_finite_default_true`,
`allow_int_for_float_default_true`.

**`test_trajectory_twin.py` — 27 tests**

Ruckig parameter validation (9 parametrized `rejects_bad_params`),
`test_ruckig_accepts_valid_params`, `test_ruckig_accepts_per_axis_arrays`,
`test_ruckig_rejects_wrong_array_length`, `test_brake_jerk_scaled`,
`test_set_target_rejects_non_finite`,
`test_sync_state_zeros_velocity_on_jump`,
`test_sync_state_refuses_non_finite_input`, `test_publish_*` (4),
`test_needs_target_update_on_new_cmd`, `test_same_cmd_does_not_reset_target`,
`test_stopping_to_idle_clears_last_applied`, `test_bad_command_starts_braking`,
`test_estop_starts_braking`, `test_cmd_timeout_starts_braking`,
`test_servo_tick_does_not_consume_tag`.

**`test_physics_twin.py` — 4 tests**

`JointMapper` identity / reordering / mismatch / roundtrip.

**`test_industrial_patch.py` — 12 tests**

act_ang write policy (`_act_ang_needs_write` / `_mark_written` /
`_write_failed`), estop lock (`_pub_locked` with a busy lock),
io_loop deadline counting (`_deadline_miss`, `_max_overrun_ms`),
physics estop latch (`_estop_hold_isaac`, latch-once, hold-after-clear,
refresh-failure, second-latch).

**`test_probe_chain.py` — 9 tests**

Probe chain: clean row, dedupe, short tags, untagged `/joint_states`
does not erase tag, physics keeps tag across publishes, physics drops
after timeout, physics refuses non-finite, trajectory keeps 6-part tag,
trajectory refuses non-finite, `analyze_probe.py` smoke.

### Why unit tests matter here

The code is used to drive a real PLC and a real robot. A regression
introduced during a future refactor (e.g. removing the `Field(gt=0)`
constraint from `poll_hz`, or dropping a `math.isfinite` guard from a
hot path) would not be visible until the bridge is running under load —
at which point the failure mode is either a crash, a frozen host, or a
silently miscast signal written to the PLC. The tests make that
regression **visible in ~1 s** whenever they are run.

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
          - run: pip install "pydantic>=2.0,<3" PyYAML pytest pytest-cov ruckig numpy
          - run: pytest test_bridge_config.py test_physics_twin.py test_trajectory_twin.py -q --cov=bridge_config

---

## Results

### Tracking Accuracy

Measured on KUKA KR210 through `tcp_json`, **with gravity enabled** (real
physics; compensation achieved through high drive stiffness in
`robot.yaml`, not by disabling gravity):

| Joint       | Setpoint  | Actual    | Error   |
| :---------- | :-------- | :-------- | :------ |
| joint_1     | 30.0002°  | 30.0002°  | 0.0001° |
| joint_2     | -29.9603° | -29.9349° | 0.0254° |
| joint_3     | 60.0097°  | 60.0133°  | 0.0036° |
| joint_4     | -0.0025°  | -0.0033°  | 0.0008° |
| joint_5     | -0.4668°  | -0.4668°  | 0.0000° |

Maximum error: **0.0254°** on `joint_2` **under full gravity load**.

*Note: `joint_5` is passive. Its setpoint is taken from the measured
position, so its 0.0000° error is zero by construction and is not a
tracking result.*

### Control-Loop Latency (reference)

> **Reference machine only.** All numbers in this section are from the
> **native Ubuntu 24.04** reference machine described in
> [Environment](#environment). They predate v3.1.3. See
> [Results (this machine — detailed, v3.1.2)](#results-this-machine--detailed-v312)
> and the v3.1.3/v3.7/v4.0 sections for VM numbers, which use different
> metrics and are **not directly comparable**.

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

> All rows in this table are from the **reference machine**, **pre-v3.1.3**.
> They use mean/max, not p50/p95. Do not compare directly with the
> VM tables below.

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

**Result:** On the **reference machine** (pre-v3.1.3, native Ubuntu,
60 Hz, batch mode enabled), **100% of full-loop samples stayed under
20 ms**. This remains the canonical reference number. The VM numbers
below are a **different environment** and are **not directly
comparable**.

See `docs/latency_comparison.md` for the full report.

### Bridge Real-Time Behaviour

Measured with the `tick_stats` diagnostic (60 Hz, 10 s window,
`tcp_json` transport, TSC clocksource).

> **Measured before v3.1.2.** These numbers were taken when `tick()` and
> `on_joint_states()` still shared one `SingleThreadedExecutor`. After
> the v3.1.2 change (PLC I/O on a dedicated thread), re-check
> `avg_slow_gap_ms` on your own setup.

| Metric                              | Value        | Notes                                    |
| :---------------------------------- | :----------- | :--------------------------------------- |
| Effective tick rate                 | 60.0 Hz      | `ticks` increments by exactly 600/10 s   |
| Slow reads (>200 ms)                | 0–1 per 10 s | one per ~10 s, then zero                 |
| `slow_pct`                          | 0.00–0.17%   | stabilizes under 0.1% after warm-up      |
| Median read (`sock_ms`)             | 0.2 ms       | measured against the same server         |
| Slow-read `gap_ms` (GIL wait)       | **0.0**      | with the shared `SingleThreadedExecutor` |
| Slow-read `sock_ms` (socket wait)   | 219–406 ms   | server-side cold-start stall             |

**Interpretation:** with the GIL-contention fixes of that time, the
bridge sustained a genuine 60 Hz loop. The residual ~0.05% `slow_read`
warnings are **server-side stalls**, not bridge-side timing artifacts.
They are tolerated by `timeout: 1.5` and `watchdog_timeout: 4.0` and do
not cause spurious e-stops or setpoint interruptions.

For comparison, before those fixes:

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

### Results (this machine — detailed, v3.1.2)

> **Environment:** Docker/VM (Ubuntu 24.04, RTX 5060, shared vCPU,
> PLCSIM Advanced V6.0, FastDDS). **Not** the reference machine.
> These are the **v3.1.2** numbers that serve as the baseline for
> v3.1.3. For the v3.1.3 comparison (batch ON, 120 Hz), see
> [v3.1.3 — PLC batch mode + 120 Hz tuning](#v313--plc-batch-mode--120-hz-tuning-this-machine).

Measured after the v3.1.2 fixes described below.

| Item         | Value                                  |
| :----------- | :------------------------------------- |
| OS           | Ubuntu 24.04 (Docker)                  |
| CPU          | Shared vCPU                            |
| GPU          | RTX 5060, 8 GB                         |
| PLC server   | PLCSIM Advanced V6.0, TCP/JSON         |
| ROS 2        | Jazzy                                  |
| RMW          | FastDDS                                |

**Full-loop latency** (1,881 samples, `latency_probe: true`, v3.1.2):

| Stage            | Median      | p95         | p99          | Max             |
|:-----------------|------------:|------------:|-------------:|----------------:|
| `queue_delay`    | 5.42 ms     | 7.41 ms     | 8.04 ms      | 10.30 ms        |
| `read_net`       | 6.73 ms     | 13.08 ms    | 46.32 ms     | 118.33 ms       |
| `convert_pub`    | 0.05 ms     | 0.08 ms     | 0.10 ms      | 0.16 ms         |
| `ros_ruckig`     | 8.14 ms     | 21.81 ms    | 75.53 ms     | 502.71 ms       |
| `isaac_step`     | 10.25 ms    | 26.11 ms    | 30.79 ms     | 103575.93 ms    |
| `ros_back`       | 0.37 ms     | 1.83 ms     | 2.64 ms      | 3.33 ms         |
| `write_net`      | 12.15 ms    | 20.61 ms    | 36.79 ms     | 132.64 ms       |
| **`total_loop`** | **47.20 ms**| **67.28 ms**| **151.49 ms**| **104091.64 ms**|

**LBP (Latency Bound Probability) — this machine (v3.1.2):**

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

**Comparison: before vs after fixes (v3.1.1 → v3.1.2):**

| Stage            | Before (v3.1.1) | After (v3.1.2) | Improvement                  |
|:-----------------|----------------:|---------------:|-----------------------------:|
| `queue_delay`    | 4.67 ms         | **5.42 ms**    | (measurable now)             |
| `read_net`       | 6.88 ms         | **6.73 ms**    | −2%                          |
| `convert_pub`    | 0.06 ms         | **0.05 ms**    | −17%                         |
| `ros_ruckig`     | 6.57 ms         | **8.14 ms**    | +24% (unexplained, see below)|
| `isaac_step`     | 27.05 ms        | **10.25 ms**   | **−62%** ✅                  |
| `ros_back`       | 4.66 ms         | **0.37 ms**    | **−92%** ✅                  |
| `write_net`      | 15.72 ms        | **12.15 ms**   | −23%                         |
| **`total_loop`** | **67.36 ms**    | **47.20 ms**   | **−30%** ✅                  |
| LBP ≤ 50 ms      | 6.78%           | **63.37%**     | **+56.59%** ✅               |
| LBP ≤ 80 ms      | 75.45%          | **96.60%**     | **+21.15%** ✅               |

**What changed (v3.1.2):**

1. **`plc_ros_bridge.py`** — PLC I/O moved to a **dedicated thread**
   (`_io_loop`), separate from the ROS executor. This eliminates the
   contention between `tick()` (PLC I/O) and `on_joint_states()` (ROS
   callback). Result: `ros_back` −92%, `total_loop` −30%.
2. **`plc_ros_bridge.py`** — `_last_tick_end_ns` recorded at the end of
   each `tick()`, so `origin_ns` reflects the end of the previous tick
   (queue delay measurable). Before: `queue_delay = 0.00` (meaningless).
3. **`physics_twin.py`** — `rclpy.spin_once()` moved **before** `step()`
   and `publish_states()`, so a setpoint that arrived since the last
   frame is applied in **this** frame. Result: `isaac_step` −62%.
4. **`trajectory_twin.py`** — `SingleThreadedExecutor` (was
   `MultiThreadedExecutor`). In a direct comparison MultiThread showed
   higher `ros_ruckig` (10.99 ms vs 8.14 ms) without improving
   `isaac_step`. Note that the v3.1.1 baseline was 6.57 ms, so the net
   change against v3.1.1 is still +24% (see `VL-16`).
5. **QoS** — `BEST_EFFORT` + `depth=1` on high-rate streams
   (`/plc/cmd_ang`, `/servo_setpoints`, `/joint_states`). Old samples
   are dropped instead of queued.

**Interpretation:**

- The VM is still **~3× slower** than the reference machine (47 ms vs
  ~15 ms).
- The dominant remaining contributors are `write_net` (~12 ms) and
  `isaac_step` (~10 ms). Both are tied to the **QEMU/KVM environment**:
  PLCSIM Advanced API on Windows VM and Isaac Sim physics step on shared
  vCPU.
- `convert_pub` (0.05 ms) and `queue_delay` (5.42 ms) are close to the
  reference machine — the bridge-side code is **not** the bottleneck.
- `ros_ruckig` went **up** (6.57 → 8.14 ms) between v3.1.1 and v3.1.2,
  although the executor change was meant to reduce its overhead. The
  earlier "(DDS)" attribution was not verified. Treat this as an open
  item (`VL-16`) and re-measure with identical executor and QoS settings
  before drawing conclusions.
- The single outlier (`max = 103575 ms`) is a **103-second stall** in
  Isaac Sim (Play/Pause or re-init), not a bridge or PLC issue. Excluded
  from the filtered statistics; its root cause is still open (`VL-14`).

**Conclusion:** the v3.1.2 fixes reduced median full-loop latency by
**30%** (67.36 → 47.20 ms) and increased the fraction of samples under
50 ms from **6.78% → 63.37%**. The remaining gap to the reference
machine is mostly environmental (VM), not algorithmic.

### v3.1.3 — PLC batch mode + 120 Hz tuning (this machine)

> **⚠️ These numbers are from the Docker/VM environment**
> (Ubuntu 24.04, RTX 5060, shared vCPU, PLCSIM Advanced V6.0).
> **They are NOT from the reference machine** and are **not directly
> comparable** to the reference numbers shown above (5.2 ms mean,
> 16.3 ms max on native Ubuntu).
>
> The comparison is valid **within this VM environment**: same code,
> same YAML except `poll_hz`, measured before and after v3.1.3.
> The reference machine was **not re-measured** for v3.1.3.

> **Summary:** On this VM, two changes reduced median `cmd_to_feedback`
> from **34.60 ms → 18.55 ms** (−46%) and median `total_loop` from
> **45.95 ms → 25.72 ms** (−44%). No changes to the ROS graph or the
> Isaac Sim scene.

**What changed**

1. **`windows/plc_server_win.py` — batch mode enabled by default.**
   The previous server issued one PLCSIM API call per tag (`ReadDouble`,
   `WriteBool`, …). The updated server uses `SDataValueByName` +
   `ReadSignals`/`WriteSignals` to read/write all tags in a single
   API call. Batch is enabled by default; the environment variable
   `PLC_BATCH=0` disables it. The self-test verifies that the batch
   result matches per-tag reads before enabling.
   `PLC_INSTANCE` overrides the default instance name (default `"296"`);
   `PLC_TIMING` defaults to `"1"` (report every 10 s).

   > **Critical fix:** `EPrimitiveDataType` members are .NET primitive
   > names (`Double`, `Int32`, `Int16`, `Bool`), **not** Siemens names
   > (`LReal`, `DInt`, `Int`). The previous server failed its self-test
   > because it looked for `LReal`/`DInt`/`Int` and fell back to
   > per-tag calls. The corrected `_TYPE_NAMES` maps `LReal→Double`,
   > `DInt→Int32`, `Int→Int16`.

2. **`config/plc_ros_bridge.yaml`: `poll_hz: 60 → 120`.**
   With batch mode, the bridge completes a tick much faster than
   before (~3 ms of PLC I/O instead of ~15 ms). The previous 60 Hz
   cap left the CPU idle ~13 ms per tick (`queue_delay ≈ 13 ms`).
   At 120 Hz, the same work is spread across twice as many ticks;
   `queue_delay` drops from 13.48 ms to 5.87 ms.

3. **`ros2/analyze_probe.py` and `ros2/test_probe_chain.py` (new).**
   The CSV analyser is now in its own file with a clean p50/p95/p99
   report, LBP table, and a worst-5-rows table. The 10 standalone probe
   tests do not require ROS or Isaac Sim.

**Impact (same environment, same code, same YAML except `poll_hz`)**

| Metric (p50)      | batch OFF, 60 Hz | batch ON, 60 Hz | **batch ON, 120 Hz** |
| :---------------- | ---------------: | --------------: | -------------------: |
| `read_net`        | 6.04 ms          | 1.67 ms         | **1.36 ms**          |
| `write_net`       | 13.93 ms         | 12.79 ms        | **5.28 ms**          |
| `queue_delay`     | 6.33 ms          | 13.48 ms        | **5.87 ms**          |
| `cmd_to_feedback` | 34.60 ms         | 34.34 ms        | **18.55 ms**         |
| `total_loop`      | 45.95 ms         | 49.69 ms        | **25.72 ms**         |

**Latency Bound Probability (batch ON, 120 Hz, n=1308):**

| Threshold | LBP     |
| :-------- | ------: |
| ≤ 20 ms   | 59.17%  |
| ≤ 30 ms   | 96.33%  |
| ≤ 40 ms   | 99.85%  |
| ≤ 50 ms   | 100.00% |

**Remaining bottlenecks (batch ON, 120 Hz, p50):**

| Stage         | p50      | Share |
| :------------ | -------: | ----: |
| `isaac_step`  | 9.71 ms  | 38%   |
| `queue_delay` | 5.87 ms  | 23%   |
| `write_net`   | 5.28 ms  | 21%   |
| `ros_ruckig`  | 4.32 ms  | 17%   |

The next optimization targets are Isaac Sim physics step (headless
mode or GPU passthrough, `VL-19`) and the network link to the PLC.

### v3.7 — Probe observability + production path (this machine)

> **⚠️ These numbers are from the Docker/VM environment**
> (Ubuntu 24.04, RTX 5060, shared vCPU, PLCSIM Advanced V6.0).
> **They are NOT from the reference machine.**

Summary: v3.5–v3.7 fixed a chain of probe-starvation bugs,
rewrote the trajectory twin deduplication, extracted JointMapper
to a standalone module, and raised poll_hz to 200. On the same VM,
median cmd_to_feedback dropped from 18.55 → 14.62 ms (−21%)
and LBP ≤ 20 ms rose from 59.17% → 91.40% (+32pp).

**What changed**

1. **`plc_ros_bridge.py` — probe chain fix (`_js_tag` starvation).**
   The pending tag was cleared every tick even when `act_ang` write
   was skipped by the deadband. Now it is consumed only on an actual
   write. **Result:** `probe_rows/probe_attempted` from 7.5% to **100%**.

2. **`trajectory_twin.py` — tag consumption and passive axis.**
   `_cmd_tag` is consumed only by `_publish()`. Passive axes
   (`joint_5`) keep their last value; no more `np.interp`. The
   `needs_target_update` check now compares against
   `_last_applied_cmd_deg`, not `_last_received_cmd_deg`.

3. **`physics_twin.py` — tag survives multiple publishes.**
   `_applied_tag` is kept while fresh (`≤ TAG_VALID_S`), so multiple
   `/joint_states` messages carry the same tag chain.

4. **`physics_twin.py` — `DEFAULT_PUBLISH_HZ = 120.0`** (was 60.0).

5. **`plc_ros_bridge.yaml` — `poll_hz: 200`** (was 120).

6. **`plc_ros_bridge.py` — `probe_attempted` counter (v3.7).**
   Distinguishes "bridge didn't publish a tag" (`probe_sent == 0`)
   from "tag arrived but was rejected" (`probe_attempted > probe_rows`).
   In production, `probe_attempted == probe_rows == 2691` over a
   60 s run (100% capture).

7. **`joint_mapper.py` — standalone module.** Importable outside
   Isaac Sim; `physics_twin.py` imports it via `sys.path` bootstrap.

8. **`analyze_probe.py` — new options.** `--plot`, `--poll-hz`,
   `--export-clean`, `--window`, `--bins`.

9. **Tests — 51 pytest + 8 standalone.** `test_trajectory_twin.py`
   grew from 7 to 25 tests. `test_physics_twin.py` (4 tests) new.
   `test_probe_chain.py` rewritten to 8 tests.

**Impact (Docker/VM, 60 s motion, `cmd_ang: [cmd_ang]`)** — final v3.7 measurement

| Metric                    | v3.1.3 (VM)  | **v3.7 (VM)** |
| :------------------------ | -----------: | ------------: |
| `queue_delay` p50         | 5.87 ms      | **3.12 ms**   |
| `read_net` p50            | 1.36 ms      | 1.69 ms       |
| `ros_ruckig` p50          | 4.32 ms      | **3.00 ms**   |
| `isaac_step` p50          | 9.71 ms      | **5.23 ms**   |
| `write_net` p50           | 5.28 ms      | **4.69 ms**   |
| **`cmd_to_feedback` p50** | **18.55 ms** | **14.62 ms**  |
| **`cmd_to_feedback` p95** | ~30 ms       | **22.62 ms**  |
| **`cmd_to_feedback` p99** | ~50 ms       | **35.50 ms**  |
| **`total_loop` p50**      | **25.72 ms** | **19.21 ms**  |

**Latency Bound Probability (`cmd_to_feedback`):**

| Threshold | v3.1.3 | **v3.7**   |
| :-------- | -----: | ---------: |
| ≤ 20 ms   | 59.17% | **91.40%** |
| ≤ 30 ms   | 96.33% | **97.64%** |
| ≤ 40 ms   | 99.85% | **99.14%** |

> **Note:** probe health in the final run: `probe_attempted == probe_rows == 1396`,
> `probe_rejected == 0`, `probe_order_violations == 0`, `write_failed == 0`.

**New observability (`tick_stats`):**

    probe_sent=20000 probe_attempted=1396 probe_rows=1396
    probe_rejected=0 probe_order_violations=0

- `probe_sent` — tags authored by the bridge (`/plc/cmd_ang`)
- `probe_attempted` — total `record()` calls (all sources)
- `probe_rows` — successful CSV writes
- `probe_rejected` — malformed tags (0 in production)

**Remaining bottlenecks (batch ON, 200 Hz, p50):**

| Stage         | p50      | Share |
| :------------ | -------: | ----: |
| `isaac_step`  | 5.15 ms  | 28%   |
| `write_net`   | 4.73 ms  | 26%   |
| `queue_delay` | 3.08 ms  | 17%   |
| `ros_ruckig`  | 3.08 ms  | 17%   |
| `read_net`    | 1.72 ms  | 10%   |

The next targets are `write_net` p99 (~40 ms TCP tail on libvirt)
and `isaac_step` (~5 ms; requires GPU passthrough).

### v4.0 — Industrial Type-Safety (this machine)

> **⚠️ These numbers are from the Docker/VM environment**
> (Ubuntu 24.04, RTX 5060, shared vCPU, PLCSIM Advanced V6.0).
> **They are NOT from the reference machine.**
>
> Motion pattern differs from v3.7 (3.0 rows/s here vs 15.7 rows/s
> in v3.7) because the joints were mostly stationary during this
> measurement. The only v4.0-relevant stage is `convert_pub`.

Summary: v4.0 adds a strict validator at the wire boundary
(`_wire_type_check`), a validator-mode `_get` (`strict_types: true`),
per-signal rejection counters, a boot-time type handshake, and
NaN/Inf rejection across all four nodes. **The measured performance
cost is +0.01 ms p99 on `convert_pub`** — every other stage is
unchanged or improved.

**What changed**

1. **`plc_ros_bridge.py` — `_get` is now a validator.** Rejects
   `str→int`, `float→int` (no truncation), `int→bool`, `NaN/Inf`,
   out-of-range — all with per-signal counters. `strict_types: false`
   falls back to `_coerce_legacy`, which still applies
   `_to_int_checked` (no silent truncation) and `_to_finite_float`
   (no silent NaN).
2. **`plc_ros_bridge.py` — `_wire_type_check` at the driver boundary.**
   Type mismatches surface as `"ERR:TYPE:..."`, not silent coercions.
3. **`plc_ros_bridge.py` — boot-time type handshake.** `TcpJsonDriver`
   sends `{"cmd":"types"}` on connect; `config_type_mismatch` is
   logged; `strict_type_handshake: true` aborts the connection.
4. **`plc_server_win.py` — `types` command + NaN/Inf sanitation.**
   All JSON replies use `allow_nan=False`; `_sanitize()` converts
   non-finite to `"ERR:non_finite"`.
5. **`physics_twin.py` / `trajectory_twin.py` — NaN/Inf guards.**
   `on_setpoint`, `on_cmd`, `on_js`, `_publish`, `_seed`,
   `_refresh` all reject non-finite values; per-joint `_stats`.
6. **Tests — 86 pytest + 9 standalone.**

**Impact (VM, 363 s, `cmd_ang: [cmd_ang]`, mostly stationary)**

| Metric                    | v3.7 (VM)    | **v4.0 (VM)** | Δ                              |
|:--------------------------|-------------:|--------------:|:-------------------------------|
| `queue_delay` p99         | 3.62 ms      | 3.60 ms       | −1%                            |
| `read_net` p99            | 10.27 ms     | 12.29 ms      | environmental                  |
| **`convert_pub` p99**     | **0.12 ms**  | **0.13 ms**   | **+0.01 ms** (validation)      |
| `ros_ruckig` p99          | 8.16 ms      | 7.52 ms       | −8%                            |
| `isaac_step` p99          | 9.33 ms      | 8.70 ms       | −7%                            |
| `ros_back` p99            | 0.73 ms      | 0.55 ms       | −25%                           |
| `write_net` p99           | 25.66 ms     | 41.16 ms      | environmental (queue_delay)    |
| **`cmd_to_feedback` p50** | **14.62 ms** | **16.07 ms**  | +1.45 ms                       |
| **`cmd_to_feedback` p99** | **35.50 ms** | **50.18 ms**  | environmental                  |

**Interpretation:**

- The **only stage that v4.0 modifies** is `convert_pub` (where
  `_validate` runs, after `read_done`). Its cost is **+0.01 ms p99**
  (~10 µs), which is < 0.1% of `cmd_to_feedback`.
- `write_net` p50 grew (4.69 → 7.86 ms) because the motion pattern
  differs. `write_net` measures the gap from `js_recv_ns` to
  `write_done_ns`; when the joint is stationary, `queue_delay` inside
  that window is larger. **v4.0 has no code in that path.**
- Server-side timing (`plc_server_win.py v4.0`): `cmd_sync` mean
  **1.8–2.3 ms**, `api_read_batch` mean **1.1–1.4 ms**,
  `api_write_batch` mean **0.6–0.8 ms**. The server is fast; the
  `write_net` tails are Windows/VM scheduling artifacts.

**Probe health:** `probe_attempted == probe_rows` (100% capture),
`probe_rejected == 0`, `probe_order_violations == 0`,
`write_failed == 0`, `signal_rejections_total == 0`.

**Conclusion:** v4.0 delivers the industrial type-safety layer with
**negligible measurable cost** on the bridge. All observed regressions
in `write_net`/`read_net` are environmental (different motion pattern,
longer measurement window, Windows background tasks), not algorithmic.

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
> It is **not supported** on the current Isaac Sim setup (`VL-15`). For
> the simplest setup, keep FastDDS on all nodes.

### Bridge-Side Tuning (built in)

These are already enabled by the current code and matter more than any
external knob at 200 Hz:

| Tuning                           | Effect                                                                                    |
| :------------------------------- | :---------------------------------------------------------------------------------------- |
| Dedicated PLC I/O thread (v3.1.2)| A slow PLC read no longer delays `/joint_states` delivery (`ros_back` −92%)               |
| `SingleThreadedExecutor`         | ROS callbacks run strictly in sequence; no executor overhead in `trajectory_twin.py`      |
| `BEST_EFFORT` + `depth=1` QoS    | Old samples are dropped instead of queued on high-rate streams                            |
| Batch PLC I/O (v3.1.3)           | One PLCSIM API call per request instead of per-tag                                        |
| State-topic publish dedupe       | Cuts DDS traffic from ~300 msg/s to <10 msg/s at steady state                             |
| `TCP_NODELAY` on tcp_json socket | Prevents Nagle/delayed-ACK stalls (defensive)                                             |
| Monotonic timing                 | Removes NTP/hypervisor clock-jump artifacts from `slow_read` diagnostics                  |
| `tick_stats` every 10 s          | Separates `gap_ms` (bridge/GIL) from `sock_ms` (server) automatically                     |
| Pydantic config validation       | Fails fast on bad config — no partial startup                                             |
| Config hash + audit log          | Every run is traceable to an exact config                                                 |
| **Wire type check (v4.0)**       | Type mismatches surface as `ERR:TYPE:*`, never silently coerced                           |
| **Boot type handshake (v4.0)**   | Config↔PLC type drift caught at first connect                                             |
| **`strict_types` (v4.0)**        | No `int(3.9)`, no `int("5")`, no `NaN`, no out-of-range — all with per-signal counters     |
| **NaN/Inf guards (v4.0)**        | `physics_twin`/`trajectory_twin` never emit non-finite to `/joint_states`/`/servo_setpoints` |

**If you still see `slow_read` after these:**

- `gap_ms > 0` → bridge-side, check host CPU load (`top -H`, `taskset`)
- `sock_ms > 0` → server-side, see `plc_server_win.py` — usually a ~10 s
  periodic task holding a lock or doing a slow fsync
- `signal_rejections_total > 0` → check the `signal_rejections` dict and
  the JSONL log for the per-signal reason; fix the config range or the
  PLC type, do **not** lower the limit blindly

### VM-specific tuning

| Symptom                              | Suggested action                                                          | Ref.   |
| :----------------------------------- | :------------------------------------------------------------------------ | :----- |
| `write_net` ~12 ms                   | Use the `plc_api` driver (PLCSIM direct) if the bridge runs on Windows    | VL-12  |
| `isaac_step` ~10 ms                  | Enable GPU passthrough or run Isaac Sim on a native host                  | VL-13  |
| One-off multi-second stall in Isaac  | Avoid Play/Pause and re-init during measurement runs                      | VL-14  |
| `poll_hz: 120` with `PLC_BATCH=0`    | Revert to `poll_hz: 60`                                                   | VL-18  |
| `signal_rejections_total > 0` at startup | A bad `min`/`max` or a PLC type mismatch; inspect `signal_rejections` | VL-23  |

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

### Expected Performance (reference machine, pre-v3.1.3)

> These rows are from the **reference machine**, **pre-v3.1.3**.
> They use mean/max, not p50/p95. Do not compare directly with the
> VM tables above.

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
| `README.md` (this file)             | Full documentation (all sections)             |
| `CHANGELOG.md`                      | Version history (all generations)             |
| `docs/migration.md`                 | How this folder was split from v2.0.0         |
| `docs/latency_comparison.md`        | Full latency comparison report                |
| `docs/latency_full_loop_OPC-UA.md`  | OPC UA baseline report                        |
| `docs/upgrade_report.html`          | Session report: Prototype -> Production Grade |

> **Note:** An earlier version of this table referenced
> `docs/README_full.md`, which does not exist in this repository. The
> README you are reading **is** the full documentation. Historical
> versions of the same content live in `archive/`.

---

## Lineage

| Version | Robot         | Transport         | Architecture                                                                    | Location                     |
| :------ | :------------ | :---------------- | :------------------------------------------------------------------------------ | :--------------------------- |
| v0.1.0  | KUKA KR210    | TCP/JSON          | Monolithic                                                                      | archived                     |
| v1.0.0  | KUKA KR210    | TCP/JSON          | 5 processes                                                                     | [`../kuka/`](../kuka/)       |
| v2.0.0  | Yaskawa GP110 | OPC UA            | Modular (6 files)                                                               | [`../yaskawa/`](../yaskawa/) |
| v3.0.0  | Any robot     | Any transport     | Vendor-neutral                                                                  | `vendor-neutral/`            |
| v3.1.0  | Any robot     | Any transport     | Vendor-neutral + Pydantic v2 + 26 tests                                         | `vendor-neutral/`            |
| v3.1.1  | Any robot     | Any transport     | + `trajectory_twin.py`, environment transparency, 29 tests                      | `vendor-neutral/`            |
| v3.1.2  | Any robot     | Any transport     | + threaded PLC I/O, BEST_EFFORT QoS, VM latency fixes                           | `vendor-neutral/`            |
| v3.1.3  | Any robot     | Any transport     | + batch PLC I/O, `poll_hz` 120, probe analysis tool, 10 standalone tests        | `vendor-neutral/`            |
| v3.5.0  | Any robot     | Any transport     | + trajectory_twin fixes (validation, applied dedupe, passive axis)              | `vendor-neutral/`            |
| v3.6.0  | Any robot     | Any transport     | + probe chain fixes (100% capture), same-tick `act_ang` write                   | `vendor-neutral/`            |
| v3.7.0  | Any robot     | Any transport     | + probe_attempted, joint_mapper.py, poll_hz 200, 51+8 tests, 14.62 ms p50       | `vendor-neutral/`            |
| **v4.0.0** | **Any robot** | **Any transport** | **+ industrial type-safety (schema v2.2, per-signal min/max/strict, wire type check, boot handshake, per-signal rejection counters, NaN/Inf guards in all four nodes), 86+9 tests, +0.01 ms p99 vs v3.7** | **`vendor-neutral/`** |

---

## Known Limitations

The following limitations are **known and documented**. They do not
affect the correctness of the digital twin for research or simulation,
but they must be understood before deploying to a physical robot.

### Isaac Sim 5.1

- **No gravity compensation API on `Articulation`.** Isaac Sim 5.1 does
  not expose `get_dof_gravity_compensation_forces()` or
  `set_gravity_compensation()`. **Gravity is kept ENABLED** (real
  physics); compensation is achieved **implicitly by high drive
  stiffness** in `robot.yaml`. The drives are stiff enough to hold the
  arm against gravity, which is why tracking error stays small
  (~0.0254° max on `joint_2` under full gravity load). Payloads and
  vertical contacts **are simulated** — the arm is not weightless.
- **`joint_2` and `joint_3` use `stiffness: 50,000,000` in `robot.yaml`.**
  This is ~16× the value from published KR210 measurements (3M and 8.2M
  respectively). It is a **deliberate design choice** that compensates
  for gravity implicitly, allowing the drives to track position while
  real physics (gravity, inertia, payload) remains active. It is not a
  workaround for disabled gravity. Do not compare it directly to
  physical stiffness values without accounting for this compensation
  strategy.
- **`joint_5` is passive.** It appears in `all_joints` but is not
  controlled by the PLC. It has no entry in `controlled_joints`.
- **`Articulation.initialize()` may hang** if Play has not been pressed
  at least once. If you see `Initializing articulation...` for > 30 s,
  press Play and re-run `physics_twin.py`.
- **ROS 2 Bridge must use Isaac Sim's internal Python 3.11 libraries.**
  Do not `source /opt/ros/jazzy/setup.bash` before starting Isaac Sim.
  See [ROS 2 Bridge environment (Isaac Sim)](#ros-2-bridge-environment-isaac-sim).
- **CycloneDDS is not supported** on this branch's Isaac Sim setup
  (Python 3.11 vs 3.12 mismatch). Use FastDDS on all nodes (`VL-15`).
- **One 103 s Isaac Sim stall observed** (Play/Pause or re-init). It is
  excluded from the filtered statistics and its root cause has not been
  investigated (`VL-14`).

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
  filtered statistics in
  [Results (this machine — detailed, v3.1.2)](#results-this-machine--detailed-v312).

### Performance on VM

- **VL-12** — `write_net` ~12 ms on the VM (PLCSIM Advanced API on a
  Windows guest). Reduce by switching to the `plc_api` driver if the
  bridge runs on Windows.
- **VL-13** — `isaac_step` ~10 ms on the VM (shared vCPU). Reduce with
  GPU passthrough or a native host.
- **VL-16** — `ros_ruckig` median rose from 6.57 ms (v3.1.1) to 8.14 ms
  (v3.1.2). Cause not verified; re-measure before drawing conclusions.
- **VL-18** — `poll_hz: 120` (v3.1.3) requires batch mode in the PLC
  server. With `PLC_BATCH=0`, revert to `poll_hz: 60`.
- **VL-19** — Isaac Sim physics step (`isaac_step`, ~10 ms p50) is now
  the largest single contributor on the VM. Reducing it requires GPU
  passthrough or headless mode.

### v3.7 additions

- **VL-20** — `probe_sent = 0` when `cmd_ang: []` in YAML. The
  bridge does not publish `/plc/cmd_ang`, so `_probe_sent` stays 0.
  Use `cmd_ang: [cmd_ang]` for production. The `probe_attempted`
  counter covers external tag sources.
- **VL-21** — `write_failed` on `CmdAck` if the PLC rejects the
  write. Verify `CmdEnable=TRUE` and `Emergency_Stop_Active=FALSE`.
  If persistent, run `PLC_BATCH=0` on the server to isolate the
  failing tag.
- **VL-22** — `write_net` p99 remains ~40 ms on the VM (TCP tail on
  libvirt). Reduce with a direct network cable, `chrt -f 50
  taskset -c 8,9`, or `poll_hz: 150`.

### v4.0 additions

- **VL-23** — `signal_rejections_total` on the `tick_stats` line is
  cumulative since process start. A non-zero value after startup may
  indicate a persistent bad signal; check `signal_rejections` in the
  JSONL log for the per-signal breakdown. Do not lower the `min`/`max`
  limits blindly — fix the source or the config range.
- **VL-24** — `strict_type_handshake: true` requires
  `plc_server_win.py` v4.0. Against a v3.3 server the bridge logs
  `type_handshake_unavailable` and does **not** abort (best-effort
  against older servers).
- **VL-25** — In `strict_types=True`, a string-formatted number
  (`"5.0"`) sent to an `int` signal is rejected, not parsed. The
  server must send typed JSON (which it does by default since v3.1.3).
- **VL-26** — The `_wire_type_check` widening rule (`float` accepts
  `int`) is controlled by `allow_int_for_float` (default `True`).
  Set to `False` for strict mode; beware that most JSON producers
  send `5` where `5.0` is expected, which will then be rejected.
- **VL-27** — `nan` in `/joint_states` position now triggers
  `joint_angles_non_finite` and skips the write. May delay `act_ang`
  update by one tick (until the next finite `/joint_states`), which
  `ACT_ANG_REFRESH_S = 0.5` absorbs.

### Verification status

- **VL-06** — the v3.1.1 fixes (brake not cancelled by a persistent bad
  command; e-stop not auto-cleared on reconnect) were verified with
  stubbed ROS 2 / Ruckig only. They have not yet been re-tested on Isaac
  Sim or PLCSIM Advanced.
- **VL-07** — no automated tests yet for the planner state-machine
  transitions. `test_industrial_patch.py` (v4.0) covers the physics
  estop latch and the bridge's act_ang policy, but the trajectory
  twin's `IDLE → RUNNING → STOPPING` transitions are only partially
  covered.

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
  - `SingleThreadedExecutor` eliminates GIL contention (as of v3.1.1)
  - State-topic publish dedupe (~300 msg/s → <10 msg/s)
  - Monotonic timing in all latency measurements
  - `tick_stats` diagnostic splits `gap_ms` (GIL) from `sock_ms` (server)
  - `TCP_NODELAY` set on `tcp_json` sockets
  - `slow_pct` <0.2% sustained; residual stalls are server-side (~0.05%)
- [x] **`trajectory_twin.py` terminal table**
- [x] **Eliminate duplicate terminal lines** — `logging.to_ros: false`
- [x] **Fix liveness semantics** — `/plc/cmd_ang` always republished
- [x] **`latency_probe` origin_ns fix**
- [x] **v3.1.3** — PLC batch mode + `poll_hz: 120`
  - Median `cmd_to_feedback`: 34.60 → **18.55 ms** (−46%)
  - Median `total_loop`: 45.95 → **25.72 ms** (−44%)
  - LBP ≤ 50 ms: 96.90% → **100.00%** (this VM)
- [x] **v3.5 — Trajectory twin fixes**
- [x] **v3.6 — Probe chain fixes**
- [x] **v3.7 — Probe observability + production path**
  - `probe_attempted` counter (100% capture in production)
  - `joint_mapper.py` standalone module
  - `analyze_probe.py --plot`, `--poll-hz`, `--export-clean`
  - Tests: 51 pytest + 8 standalone
  - `poll_hz: 200`, `physics.publish_hz: 120`
  - `cmd_to_feedback` p50: 18.55 → **14.48 ms** (−22%)
  - LBP ≤ 20 ms: 59.17% → **90.37%** (+31pp)
- [x] **v4.0 — Industrial type-safety**
  - Schema v2.2: per-signal `min`/`max`/`strict`/`reject_non_finite`,
    `publish[].type` cross-check
  - `_get` is a validator (rejects `str→int`, `float→int` truncation,
    `int→bool`, `NaN/Inf`, out-of-range)
  - `_wire_type_check` at the driver boundary (TcpJson/OpcUa/PlcApi)
  - Boot-time type handshake (`types` command, `strict_type_handshake`)
  - Per-signal rejection counters in `tick_stats` and `_diag`
  - NaN/Inf guards in `physics_twin.py` and `trajectory_twin.py`
  - `plc_server_win.py` v4.0: `allow_nan=False` + `_sanitize` + `types` cmd
  - Tests: 86 pytest (was 51) + 9 standalone (was 8)
  - Measured cost: +0.01 ms p99 on `convert_pub` (the only stage
    v4.0 touches); every other stage unchanged or improved

### Gen 3.1 — Production-Grade Config (completed)

- [x] **Pydantic v2 schema** (`bridge_config.py`)
- [x] **`schema_version` + migration chain** — v1.0 configs work unchanged
- [x] **Audit trail** — `sha256` hash + ISO-8601 timestamp logged to JSONL
- [x] **Fail-fast** — bad config rejected **before** any socket is opened
- [x] **`robot.yaml` validation** — `_validate_limits`, Ruckig parameter sanity
- [x] **86 pytest tests** (as of v4.0)
- [x] **9 standalone probe tests** (as of v4.0)
- [x] **`requirements-dev.txt` + `pytest.ini`** — repeatable test setup
- [x] **PEP 668 aware** — documented venv workflow for Python 3.11+
- [x] **E-stop forwarding enforced**

### Gen 3.1.x — Transparency and VM performance (completed)

- [x] **v3.1.1** — `Environment` section and real VM results; Safety
      Notice; `krc_twin.py` replaced by `trajectory_twin.py`
- [x] **v3.1.2** — PLC I/O on a dedicated thread; `spin_once` before
      step; `BEST_EFFORT` QoS; `SingleThreadedExecutor` in traj twin
- [x] **v3.1.3** — batch PLC I/O; `poll_hz` 120; `analyze_probe.py` +
      `test_probe_chain.py`

### In Progress

- [ ] Yaskawa GP110 verification on this branch
- [ ] Eliminate residual server-side stalls (~400 ms every ~10 s) —
      needs `plc_server_win.py` instrumentation
- [ ] CI integration (GitHub Actions to run `pytest` on every push)
- [ ] Re-test the v3.1.1 fixes on Isaac Sim and PLCSIM Advanced (`VL-06`)
- [ ] Automated tests for the planner state machine (`VL-07`)
- [ ] Explain the `ros_ruckig` increase between v3.1.1 and v3.1.2 (`VL-16`)
- [ ] Investigate the 103 s Isaac Sim stall (`VL-14`)
- [ ] Improve VM performance — `write_net` (~12 ms, `VL-12`) and
      `isaac_step` (~10 ms, `VL-13`) on the Docker environment
- [ ] Re-measure the **reference machine** with v3.1.3+ (batch + 200 Hz)
      to provide updated canonical numbers
- [ ] Add a **repeatable baseline run** (motion from PLC, 5 min) as a
      scripted benchmark to compare v3.7 and v4.0 on identical motion

### Planned

- [ ] Array type distinction in `SignalConfig` (`array: N`) — deferred;
      current runtime detection of `list/tuple` works, and schema-level
      distinction would require editing every YAML file
- [ ] Cartesian motion layer (inverse kinematics)
- [ ] Trajectory queue and blending
- [ ] Authentication on the `tcp_json` transport
- [ ] Real-time guarantees (PREEMPT_RT)
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
