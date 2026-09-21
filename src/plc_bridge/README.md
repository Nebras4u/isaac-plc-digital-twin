# OPC UA ↔ ROS2 Bridge

**Purpose**: Bidirectional bridge between **Siemens S7-1500T (OPC UA)** and **Isaac Sim (ROS2)**.  
Reads PLC telemetry, reads Isaac `/joint_states`, converts reference frames, prints comparison table.

**File**: `bridge.py`  
**Runtime**: Python 3.8+ | `asyncua` | `rclpy`

---

## Overview

```
S7-1500T ──OPC UA──▶ [bridge.py] ──ROS2──▶ Isaac Sim
   ActPos              convert              /joint_states
   ActAng               frames
   Flags
```

**Loop rate**: 5 Hz (200 ms per cycle).

---

## Configuration

| Constant | Value | Meaning |
|----------|-------|---------|
| `PLC_URL` | `opc.tcp://111.111.111.10:4840` | OPC UA endpoint |
| `NS` | `3` | Namespace index |
| `DB_TCP` | `MAIN_KUKA_KR210_L150_L` | TIA Portal DB name |

### Read from PLC

**Arrays** (`PLC_ARRAYS`):
- `MainRobotLeft_ActPos` — TCP [X, Y, Z, A]
- `MainRobotLeft_ActAng` — Joint angles [A1..A4] (math frame)
- `MainRobotLeft_Pos4Maint` — Maintenance pose

**Flags** (`PLC_FLAGS`):
- `MainRobotLeft_Go2Pos4Maint`
- `MainRobotLeft_Group_Fault`
- `MainRobotLeft_Axis1..4_Fault`

### NodeId Pattern

```
ns=3;s="MAIN_KUKA_KR210_L150_L"."<Variable>"
```

---

## Reference Frames

Three frames exist; conversion is required for correct comparison.

### 1. KUKA Mathematical (PLC)

Angles inside the controller and PLC. **Not** the physical robot angle.

### 2. KUKA Mechanical (Physical)

Physical mastering zero position. Related by `$MAMES`:

```
Q_mechanical = Q_mathematical - $MAMES
```

**$MAMES** (from `$MACHINE.DAT`):

| Axis | Value (deg) |
|------|-------------|
| A1 | 0.0 |
| A2 | -90.0 |
| A3 | +90.0 |
| A4 | 0.0 |

### 3. Isaac Raw (ROS2)

Isaac URDF zero ≠ KUKA mechanical zero. Offset removed by:

```
Q_mech = Q_isaac_raw - ISAAC_HOME_OFFSET
```

**ISAAC_HOME_OFFSET** (measured at Isaac home pose):

| Axis | Value (deg) |
|------|-------------|
| A1 | +0.802 |
| A2 | +5.105 |
| A3 | -5.094 |
| A4 | -1.702 |

**After both conversions**: PLC and Isaac are in the **same mechanical frame** → comparison is valid.

---

## Axis Mapping

| PLC | ROS / Isaac |
|-----|-------------|
| A1 | joint_a1 |
| A2 | joint_a2 |
| A3 | joint_a3 |
| A4 | joint_a4 |

(`A5`, `A6` mapped but unused.)

---

## Key Functions

| Function | Purpose |
|----------|---------|
| `kuka_math_to_mech(q_math_deg)` | Subtract `$MAMES` |
| `isaac_raw_to_mech(q_ros_deg)` | Subtract `ISAAC_HOME_OFFSET` |
| `fmt(v, nd=3)` | Format float to `nd` decimals |
| `arr4(vals)` | Pad list to exactly 4 items |

---

## Threading Model

```
Main Thread                    ROS Thread (daemon)
─────────────                  ────────────────────
asyncio loop                   rclpy.spin_once()
  │                              │
  ▼                              ▼
opcua_loop()                   JointStateListener
  │                              │
  ├─ read PLC arrays             ├─ subscribe /joint_states
  ├─ read PLC flags              └─ store msg in holder["msg"]
  ├─ read holder["msg"]
  ├─ convert frames
  ├─ print table
  └─ sleep 0.2s
```

- **`holder`**: dict shared between threads (`{"msg": JointState}`)
- **`stop_event`**: `threading.Event` for graceful shutdown
- **`SignalHandlerOptions.NO`**: Ctrl+C goes to asyncio, not ROS

---

## Main Loop (per iteration)

1. **Read arrays** — 1 network round-trip (`read_values`)
2. **Read flags** — 1 network round-trip
3. **Extract** — `arr4()` ensures 4-element lists
4. **PLC convert** — math → mechanical
5. **ROS read** — from `holder["msg"]`, rad → deg → mechanical
6. **Print snapshot** — ActPos, ActAng, Pos4Maint, Flags
7. **Print table** — per-axis comparison (PLC math/mech vs ROS raw/mech, delta)
8. **Sleep 200 ms**

---

## Sample Output

```
================================================================================
PLC ActPos [mm/deg] : X=450.000  Y=0.000  Z=712.000  A=0.000
PLC ActAng (math)   : A1=0.000, A2=90.000, A3=-90.000, A4=0.000
PLC Pos4Maint       : ['-486.800', '-418.600', '205.000', '0.000']
PLC Flags           : {'MainRobotLeft_Group_Fault': False, ...}
--------------------------------------------------------------------------------
Axis     PLC math   PLC mech     ROS raw    ROS mech   Δ(PLC-ROS)
A1          0.000      0.000       0.802       0.000         0.000
A2         90.000    180.000      95.105     180.000         0.000
A3        -90.000   -180.000     -95.094    -180.000         0.000
A4          0.000      0.000      -1.702       0.000         0.000
================================================================================
```

Delta ≈ 0 → frames aligned ✅

---

## Run

```bash
# Terminal 1: Isaac Sim publishing /joint_states
# Terminal 2:
python3 bridge.py
```

**Requirements**: `pip install asyncua` + ROS2 installed and sourced.

**Shutdown**: `Ctrl+C` → signals `stop_event` → ROS thread joins (2 s timeout).

---

## Known Limitations

| Issue | Note |
|-------|------|
| Hardcoded endpoint | `PLC_URL` not configurable via CLI |
| Read-only | No write-back to PLC implemented |
| Print-only | No logging / file output |
| 4 axes only | A5/A6 mapped but unused in this robot |
| No reconnect | Connection lost → script exits |
| Fixed 5 Hz | Not adaptive to load |

---

## Data Flow Reference (AAS Alignment)

| Bridge reads | AAS Path | Submodel |
|--------------|----------|----------|
| `MainRobotLeft_ActPos` | `Status/MainRobotLeft/ActPos` | `Status.json` |
| `MainRobotLeft_ActAng` | `Status/MainRobotLeft/ActAng` | `Status.json` |
| `MainRobotLeft_Group_Fault` | `Status/MainRobotLeft/Group_Fault` | `Status.json` |
| `$MAMES` | `DigitalTwin_Config/KUKA_Mastering` | `DigitalTwin_Config.json` |
| `ISAAC_HOME_OFFSET` | `DigitalTwin_Config/IsaacSim_HomeOffset` | `DigitalTwin_Config.json` |

**Full AAS**: `/aas/KUKA_KR210_L150_L.json`

---

## Conversion Formulas (Summary)

```
PLC math  →  PLC mech:     Q_mech = Q_math  - $MAMES
ROS raw   →  ROS mech:     Q_mech = Q_raw   - ISAAC_HOME_OFFSET
Delta:                     Δ      = PLC_mech - ROS_mech
```

Both endpoints in **mechanical frame** → comparison meaningful.

---

**Repository**: https://github.com/Nebras4u/isaac-plc-digital-twin  
**Last updated**: 2026-09-19
