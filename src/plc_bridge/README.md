## Appendix F — Modular Refactoring: From Monolith to Modular Architecture

### F.1 Context

The original bridge was implemented as a single monolithic Python file (`plc_isaac_ros2_bridge.py`). While functional, it became difficult to maintain, test, and extend. As part of v0.2.2, the codebase was refactored into a modular architecture to support future development (v0.3 IK node, v0.4 suction gripper, v1.1 Real-Time Sync).

### F.2 Motivation for Refactoring

| Problem (Monolith) | Solution (Modular) |
|---------------------|---------------------|
| Single 400+ line file | Six focused modules |
| Hard to test individual components | Each module testable in isolation |
| PLC/Isaac/Comparison logic mixed | Separation of concerns |
| Config scattered across file | Centralized in config.py |
| No clear extension points | Clear interfaces for new features |
| Duplicate code for similar tasks | Reusable helper functions |

### F.3 New Module Structure

| File | Responsibility | Lines (approx.) |
|------|----------------|-----------------|
| config.py | All constants: PLC URL, node IDs, frame names, signs, offsets, joint mappings | ~50 |
| opcua_bridge.py | OPC UA connection layer: connect / read / write / close | ~60 |
| plc_view.py | PLC data extraction and display formatting | ~60 |
| isaac_view.py | Isaac Sim data extraction, frame conversion, quaternion math | ~130 |
| comparator.py | Joint/TCP/Frame comparison tables | ~110 |
| main.py | Entry point: orchestrates asyncio + ROS2 threads | ~150 |

### F.4 Architecture Diagram

+-----------------------------------------------------------------+
|                         main.py                                 |
|  +--------------+    +--------------+    +--------------+       |
|  |  asyncio     |    |  ROS2 Thread |    |  OpcUaBridge |       |
|  |  main_loop() |<-->|  (daemon)    |    |  (async)     |       |
|  +------+-------+    +------+-------+    +------+-------+       |
|         |                   |                   |               |
|         v                   v                   v               |
|  +--------------+    +--------------+    +--------------+       |
|  |  plc_view.py |    | isaac_view.py|    |  config.py   |       |
|  |  extract +   |    |  extract +   |    |  (constants) |       |
|  |  print       |    |  convert +   |    |              |       |
|  +------+-------+    |  print       |    +--------------+       |
|         |            +------+-------+                           |
|         |                   |                                   |
|         v                   v                                   |
|  +-----------------------------------------+                    |
|  |           comparator.py                 |                    |
|  |  joint + TCP + frame comparison tables  |                    |
|  +-----------------------------------------+                    |
+-----------------------------------------------------------------+

### F.5 Key Design Decisions

#### F.5.1 Single Source of Truth (config.py)

All tunable parameters live in one place:

# PLC connection
PLC_URL = "opc.tcp://111.111.111.10:4840"
NS = 3
DB_TCP = "MAIN_KUKA_KR210_L150_L"

# Frame definitions
WORLD_FRAME = "base_link"
TCP_FRAME = "tool"
TRACKED_FRAMES = ["base_link", "link_1", "link_2", "link_3", "link_4", "tool"]

# Joint mapping
PLC_TO_ROS = {"A1": "joint_1", "A2": "joint_2", "A3": "joint_3", "A4": "joint_4"}

# Signs and offsets
JOINT_SIGN = {"A1": +1.0, "A2": -1.0, "A3": -1.0, "A4": +1.0}
MAMES_DEG = {"A1": 0.0, "A2": 0.0, "A3": 0.0, "A4": 0.0}
ISAAC_HOME_OFFSET = {"A1": 0.0, "A2": 0.0, "A3": 0.0, "A4": 0.0}

Benefit: Changing a joint mapping or a sign requires editing one line in one file.

#### F.5.2 Separation of Data Extraction and Display

Each view module (plc_view.py, isaac_view.py) provides:
- extract_*() — pure data extraction (no printing)
- print_*() — display formatting (no data logic)

Benefit: Data can be used by other consumers (e.g., v0.3 IK node) without pulling in printing logic.

#### F.5.3 OPC UA Layer Isolation

opcua_bridge.py exposes a clean async interface:

class OpcUaBridge:
    async def connect(self)
    async def close(self)
    async def read_arrays(self) -> dict
    async def read_flags(self) -> dict
    async def read_all(self) -> tuple[dict, dict]
    async def write(self, node_id: str, value)

Benefit: The write() method is ready for future bidirectional control (v1.1 Real-Time Sync) without refactoring.

#### F.5.4 Comparison Layer Independence

comparator.py receives plc_data and isaac_data dicts, making it:
- Testable with synthetic data
- Reusable for logging to CSV/JSON
- Independent of OPC UA or ROS2

### F.6 Migration from Monolith

| Old (Monolith) | New (Modular) |
|----------------|---------------|
| PLC_URL, NS, DB_TCP inline | config.py |
| MAMES_DEG, ISAAC_HOME_OFFSET inline | config.py |
| PLC_TO_ROS inline | config.py |
| kuka_math_to_mech() inline | plc_view.py |
| isaac_raw_to_mech() inline | isaac_view.py |
| quat_to_kuka_a() inline | isaac_view.py |
| arr4(), fmt() inline | plc_view.py / comparator.py |
| OPC UA loop inline | opcua_bridge.py |
| Comparison table inline | comparator.py |
| JointStateListener inline | main.py |

### F.7 How to Extend

| Future Feature | Where to Add |
|----------------|--------------|
| New PLC variable | config.py -> PLC_ARRAYS or PLC_FLAGS |
| New joint mapping | config.py -> PLC_TO_ROS |
| New frame to track | config.py -> TRACKED_FRAMES |
| New comparison metric | comparator.py -> new print_*() function |
| Write-back to PLC | opcua_bridge.py -> use existing write() |
| CSV logging | main.py -> call comparator functions, save output |
| IK node (v0.3) | New module ik_node.py, import isaac_view |
| A5 kinematics | config.py -> add compute_A5() helper |

### F.8 Benefits Realized

| Benefit | Impact |
|---------|--------|
| Maintainability | Each file < 150 lines, single responsibility |
| Testability | Each module can be unit-tested independently |
| Readability | Clear data flow: config -> extract -> convert -> compare -> print |
| Extensibility | New features plug in without touching existing modules |
| Reusability | isaac_view, comparator usable by future IK/logging nodes |
| Team collaboration | Multiple developers can work on separate modules |

### F.9 Lessons Learned

| # | Lesson |
|---|--------|
| 1 | Start modular early — refactoring later costs more than designing modular from the start. |
| 2 | Config centralization prevents bugs from scattered magic numbers. |
| 3 | Separating data from display enables reuse in non-printing contexts. |
| 4 | Clean interfaces (OpcUaBridge.write) future-proof the codebase. |
| 5 | Small focused modules are easier to debug than large monolithic files. |
| 6 | Comparison layer independence enables unit tests with synthetic data. |

### F.10 References

- Original monolith: plc_isaac_ros2_bridge.py (deprecated)
- Modular version: config.py, opcua_bridge.py, plc_view.py, isaac_view.py, comparator.py, main.py
- Related appendices: Appendix E (Hidden A5 Problem), Appendix D (Zero-Pose Validation)

