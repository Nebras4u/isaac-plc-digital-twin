# Appendix F — Modular Refactoring: From Monolith to Modular Architecture

## F.1 Context

The original bridge was implemented as a single monolithic Python file
(`plc_ros_bridge4.py`). While functional, it was difficult to maintain,
test, and extend. As part of the modular refactoring effort, the codebase
was split into **five focused modules** with clear responsibilities.

The goal: **preserve the exact runtime behavior of the monolith** while
enabling future development (v0.3 IK node, v0.4 suction gripper,
v1.1 Real-Time Sync).

---

## F.2 Motivation for Refactoring

| Problem (Monolith)                  | Solution (Modular)                     |
|-------------------------------------|----------------------------------------|
| Single 200+ line file               | Five focused modules                   |
| Hard to test individual components  | Each module testable in isolation      |
| PLC / ROS / mapping logic mixed     | Separation of concerns                 |
| Config scattered across file        | Centralized in `config.py`             |
| No clear extension points           | Clear interfaces for new features      |
| Duplicate helper functions          | Reusable functions in `joint_map.py`   |

---

## F.3 New Module Structure

| File             | Responsibility                                                                  | Lines (approx.) |
|------------------|----------------------------------------------------------------------------------|-----------------|
| `config.py`      | All constants: PLC URL, NS, DB_TCP, joint signs, offsets, control rates          | ~90             |
| `joint_map.py`   | Unit conversion (PLC deg ↔ ROS rad) + `PLC_TO_ROS` + `ROS_ORDER`                 | ~25             |
| `plc_client.py`  | OPC UA layer: Node IDs + `opc_write()` helper                                    | ~35             |
| `ros_bridge.py`  | ROS 2 node: `PlcRosBridge` + `run_ros()` spin thread                             | ~60             |
| `main.py`        | Entry point: async main loop orchestrating PLC I/O + ROS publishing              | ~120            |

> **Note on naming:** The mapping module is named `joint_map.py`
> (not `mapping.py`) to avoid a silent name collision with Python's
> built-in `collections.abc.Mapping` hierarchy. Using `mapping.py`
> caused import resolution to pick up the wrong module without any
> error message — a subtle bug that took time to isolate.

---

## F.4 Architecture Diagram

```
+-----------------------------------------------------------------+
|                            main.py                              |
|                                                                 |
|  +----------------+    +-----------------+    +--------------+  |
|  |  asyncio loop  |    |  ROS2 Thread    |    |  asyncua     |  |
|  |  (main_loop)   |<-->|  run_ros()      |    |  Client      |  |
|  +--------+-------+    +--------+--------+    +------+-------+  |
|           |                     |                    |          |
|           v                     v                    v          |
|  +----------------+    +-----------------+    +--------------+  |
|  |  joint_map.py  |    |  ros_bridge.py  |    | plc_client.py|  |
|  |  deg <-> rad   |    |  PlcRosBridge   |    |  Node IDs +  |  |
|  |  PLC_TO_ROS    |    |  publish_cmd    |    |  opc_write   |  |
|  +--------+-------+    +--------+--------+    +------+-------+  |
|           |                     |                    |          |
|           +----------+----------+--------------------+          |
|                      |                                          |
|                      v                                          |
|              +----------------+                                 |
|              |   config.py    |                                 |
|              |   constants    |                                 |
|              +----------------+                                 |
+-----------------------------------------------------------------+
```

---

## F.5 Key Design Decisions

### F.5.1 Single Source of Truth (`config.py`)

All tunable parameters live in one place:

```python
# PLC connection
PLC_URL = "opc.tcp://192.168.100.50:4840"
NS = 3
DB_TCP = "MAIN_KUKA_KR210_L150_L"

# Joint mapping
PLC_TO_ROS = {"A1": "joint_1", "A2": "joint_2", "A3": "joint_3", "A4": "joint_4"}

# Signs and offsets
JOINT_SIGN        = {"A1": +1.0, "A2": -1.0, "A3": -1.0, "A4": +1.0}
MAMES_DEG         = {"A1": 0.0,  "A2": 0.0,  "A3": 0.0,  "A4": 0.0}
ISAAC_HOME_OFFSET = {"A1": 0.0,  "A2": 0.0,  "A3": 0.0,  "A4": 0.0}

# Control loop
CONTROL_HZ       = 20
PRINT_PERIOD_SEC = 1.0
```

**Benefit:** Changing a joint sign or a PLC URL requires editing one line
in one file.

---

### F.5.2 Pure Mapping Layer (`joint_map.py`)

Two pure functions with no side effects:

```python
def plc_deg_to_ros_rad(ax, deg): ...
def ros_rad_to_plc_deg(ax, ros_rad): ...
```

**Benefit:** These can be unit-tested with synthetic values, reused by
future IK nodes, and reasoned about without knowledge of ROS or OPC UA.

---

### F.5.3 OPC UA Layer Isolation (`plc_client.py`)

Node IDs are defined as module-level constants:

```python
NODE_CMD_ENABLE = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_CmdEnable"'
NODE_CMD_ACK    = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_CmdAck"'
NODE_ACT_ANG    = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActAng"'
NODE_ACT_POS    = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActPos"'
NODE_CMD_ANG    = NODE_ACT_ANG  # command source (same node in this deployment)
```

And a single low-level write helper:

```python
async def opc_write(node, value, variant_type): ...
```

**Benefit:** The `opc_write()` helper is ready for bidirectional control
(v1.1 Real-Time Sync) without refactoring the call sites in `main.py`.

---

### F.5.4 ROS 2 Isolation (`ros_bridge.py`)

The ROS 2 node is a self-contained class:

```python
class PlcRosBridge(Node):
    def __init__(self): ...
    def on_joint_states(self, msg): ...
    def publish_command(self): ...

def run_ros(bridge, stop_event): ...
```

`run_ros()` is a free function so it can be launched in a daemon thread
from `main.py` without any global ROS state leaking into the asyncio loop.

**Benefit:** The node can be swapped or mocked for testing. A future
multi-robot deployment can instantiate one `PlcRosBridge` per robot.

---

### F.5.5 Async Orchestration (`main.py`)

`main_loop()` is the only place where PLC I/O and ROS publishing meet:

```python
while not stop_event.is_set():
    enable  = await cmd_enable_node.read_value()
    cmd     = await cmd_ang_node.read_value()
    act_pos = await act_pos_node.read_value()

    with bridge.lock:
        bridge.plc_enable         = enable
        bridge.plc_cmd_angles_deg = cmd[:4]

    a_vals = [...]  # from latest joint_states
    await opc_write(act_ang_node, a_vals, ua.VariantType.Double)
    await opc_write(cmd_ack_node, bool(enable), ua.VariantType.Boolean)

    await asyncio.sleep(dt)
```

**Benefit:** The orchestration is linear, easy to read, and easy to
extend (add a new read, add a new write, add a status print).

---

## F.6 Migration from Monolith

| Old (Monolith)                           | New (Modular)     |
|------------------------------------------|-------------------|
| `PLC_URL`, `NS`, `DB_TCP` inline         | `config.py`       |
| `JOINT_SIGN`, `MAMES_DEG` inline         | `config.py`       |
| `ISAAC_HOME_OFFSET` inline               | `config.py`       |
| `PLC_TO_ROS`, `ROS_ORDER` inline         | `joint_map.py`    |
| `plc_deg_to_ros_rad()` inline            | `joint_map.py`    |
| `ros_rad_to_plc_deg()` inline            | `joint_map.py`    |
| `NODE_*` Node IDs inline                 | `plc_client.py`   |
| `opc_write()` inline                     | `plc_client.py`   |
| `PlcRosBridge` class inline              | `ros_bridge.py`   |
| `run_ros()` inline                       | `ros_bridge.py`   |
| `main_loop()` + `if __name__` inline     | `main.py`         |

---

## F.7 How to Extend

| Future Feature             | Where to Add                                          |
|----------------------------|-------------------------------------------------------|
| New PLC variable           | `config.py` → add to `PLC_ARRAYS` or `PLC_FLAGS`      |
| New joint mapping          | `joint_map.py` → update `PLC_TO_ROS`                  |
| New frame to track         | `config.py` → update `TRACKED_FRAMES`                 |
| Safety clamp               | `joint_map.py` → add `clamp_angles()`                 |
| Fault gating               | `ros_bridge.py` → check `plc_fault` before publishing |
| Watchdog / reconnect       | `main.py` → wrap `client.connect()` in retry loop     |
| CSV logging                | `main.py` → write `a_vals` + `act_pos` to file        |
| IK node (v0.3)             | New module `ik_node.py`, import from `joint_map`      |
| Suction gripper (v0.4)     | New module `gripper.py`, add ROS publisher            |

---

## F.8 Benefits Realized

| Benefit            | Impact                                                                  |
|--------------------|-------------------------------------------------------------------------|
| Maintainability    | Each file < 130 lines, single responsibility                            |
| Testability        | `joint_map.py` is pure; `plc_client.py` is mockable                     |
| Readability        | Clear data flow: `config → joint_map → plc_client / ros_bridge → main`  |
| Extensibility      | New features plug in without touching existing modules                  |
| Reusability        | `joint_map` usable by future IK / logging nodes                         |
| Team collaboration | Multiple developers can work on separate modules                        |

---

## F.9 Lessons Learned

| # | Lesson                                                                                          |
|---|-------------------------------------------------------------------------------------------------|
| 1 | Start modular early — refactoring later costs more than designing modular from the start.       |
| 2 | Config centralization prevents bugs from scattered magic numbers.                               |
| 3 | **Avoid reserved/standard Python names for filenames** (`mapping.py`, `types.py`, `copy.py`).    |
|   | A silent import collision (`mapping` vs `collections.abc.Mapping`) broke the split codebase.     |
| 4 | Clean interfaces (`opc_write`) future-proof the codebase.                                       |
| 5 | Small focused modules are easier to debug than large monolithic files.                          |
| 6 | **Preserve exact runtime behavior when refactoring** — move code, do not redesign it.           |
| 7 | When a split codebase behaves differently from its monolith, check imports before logic.        |

---

## F.10 References

- Original monolith: `plc_ros_bridge4.py`
- Modular version:
  - `config.py`
  - `joint_map.py`
  - `plc_client.py`
  - `ros_bridge.py`
  - `main.py`
- Related appendices:
  - Appendix E (Hidden A5 Problem)
  - Appendix D (Zero-Pose Validation)
