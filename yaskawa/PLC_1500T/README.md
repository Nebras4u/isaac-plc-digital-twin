# PLC IsaacSim ROS2 — SCL Control Program

**Project**: Isaac Digital Twin (S7-1500T ↔ OPC UA ↔ ROS2 ↔ Isaac Sim)  
**PLC**: Siemens S7-1500T | TIA Portal v17 | SCL  
**DB**: `MAIN_KUKA_KR210_L150_L`  
**AAS**: `/aas/KUKA_KR210_L150_L.json`

---

## Overview

SCL program controlling a **KUKA KR210 L150 L** (4 axes) via **S7-1500T Technology Objects**.
Integrated into Isaac Sim Digital Twin through **OPC UA (ns=3) + ROS2**.

**Modes**: Auto Left Stacking | Auto Right Stacking | Manual PTP

---

## Architecture

```
S7-1500T ──OPC UA──▶ ROS2 Bridge ──DDS──▶ Isaac Sim
   │
   ├── DB: MAIN_KUKA_KR210_L150_L  (Inputs / Outputs)
   ├── TO: MainRobotLeft_Kinematics_1
   ├── TO: MainRobotLeft_PositioningAxis_1..4
   └── SCL: this program
```

---

## DB Variables (Key)

### Inputs (read by SCL)

| Variable | Type | Purpose |
|----------|------|---------|
| `MainRobotLeft_EnableAll` | BOOL | Power all axes |
| `MainRobotLeft_EnableKinematics` | BOOL | Enable group |
| `MainRobotLeft_Auto_Mode` | BOOL | TRUE=Auto, FALSE=Manual |
| `MainRobotLeft_Vlcty` | LREAL | Velocity (mm/s) |
| `MainRobotLeft_Stacking2Left/Right` | BOOL | Trigger stacking |
| `MainRobotLeft_Go2HomePos/Pos4Maint` | BOOL | Manual triggers |
| `MainRobotLeft_HomePos[1..4]` | ARRAY LREAL | Home [X,Y,Z,A] |
| `MainRobotLeft_UpTheLinePos[1..4]` | ARRAY LREAL | Park pose |
| `MainRobotLeft_PickUp1..3[1..4]` | ARRAY LREAL | Pickup points |
| `MainRobotLeft_StackingLeft/Right_Points1..6[1..4]` | ARRAY LREAL | Stacking seq |
| `MainRobotLeft_Pos4Maint[1..4]` | ARRAY LREAL | Maintenance pose |

### Outputs (written by SCL)

| Variable | Type | Purpose |
|----------|------|---------|
| `MainRobotLeft_Group_Fault` | BOOL | Group fault |
| `MainRobotLeft_Axis1..4_Fault` | BOOL | Per-axis fault |
| `MainRobotLeft_ActPos[1..4]` | ARRAY LREAL | TCP [X,Y,Z,A] |
| `MainRobotLeft_ActAng[1..4]` | ARRAY LREAL | Joint angles |
| `MainRobotLeft_Dynamic_Speed[1..4]` | ARRAY LREAL | Axis velocity |
| `MainRobotLeft_Dynamic_Torque[1..4]` | ARRAY LREAL | Torque (simulated ⚠️) |
| `MainRobotLeft_Axis_Power[1..4]` | ARRAY LREAL | Power (simulated ⚠️) |

---

## Program Sections

### §1 — Fault Monitoring

Maps `StatusWord.%X1` (FollowUpError) from each TO to DB fault flags.

```scl
"MAIN_KUKA_KR210_L150_L".MainRobotLeft_Group_Fault := "MainRobotLeft_Kinematics_1".StatusWord.%X1;
"MAIN_KUKA_KR210_L150_L".MainRobotLeft_Axis1_Fault := "MainRobotLeft_PositioningAxis_1".StatusWord.%X1;
// ... Ax2, Ax3, Ax4
```

### §2 — Power Management

`MC_PWR_Ax1..4` FBs enable all axes together when `EnableAll=TRUE`.

```scl
"MC_PWR_Ax1_DB"(Axis := "MainRobotLeft_PositioningAxis_1",
                Enable := "MAIN_KUKA_KR210_L150_L".MainRobotLeft_EnableAll,
                StartMode := 1, StopMode := 0);

#System_Hardware_Ready := "MC_PWR_Ax1_DB".Status
                      AND "MC_PWR_Ax2_DB".Status
                      AND "MC_PWR_Ax3_DB".Status
                      AND "MC_PWR_Ax4_DB".Status;
```

### §3 — Traffic Control

- Rising edge of `EnableKinematics` → single-shot `#Trigger_Motion`
- Loss of hardware → force-disable kinematics + clear maintenance commands

```scl
IF NOT #System_Hardware_Ready THEN
    "MAIN_KUKA_KR210_L150_L".MainRobotLeft_EnableKinematics := FALSE;
    "MAIN_KUKA_KR210_L150_L".MainRobotLeft_Go2Pos4Maint := FALSE;
    #Trigger_Motion := FALSE;
ELSE
    IF "MAIN_KUKA_KR210_L150_L".MainRobotLeft_EnableKinematics AND NOT #Old_Kinematics_State THEN
        #Trigger_Motion := TRUE;
    ELSE
        #Trigger_Motion := FALSE;
    END_IF;
END_IF;
#Old_Kinematics_State := "MAIN_KUKA_KR210_L150_L".MainRobotLeft_EnableKinematics;
```

### §4 — Motion Execution

| Mode | Trigger | Sequence |
|------|---------|----------|
| Auto Left | `Auto_Mode AND Stacking2Left` | PickUp1→2→3 → Stkng_L1..6 → Park |
| Auto Right | `Auto_Mode AND Stacking2Right` | PickUp1→2→3 → Stkng_R1..6 → Park |
| Manual | `NOT Auto_Mode AND (Go2Home/Go2Maint)` | Single move |

**Chaining**: `MC_X.Active` → next block's `Execute`  
**BufferMode**: `2` (blend) between points, `1` (stop) at end

```scl
"MC_PickUp1"(Execute := #Trigger_Motion, Position := "...PickUp1", ...);
"MC_PickUp2"(Execute := "MC_PickUp1".Active, Position := "...PickUp2", ...);
"MC_PickUp3"(Execute := "MC_PickUp2".Active, Position := "...PickUp3", ...);
// ... then MC_Stkng_L1..6 or R1..6, then MC_UpTheLinePos_Return
```

### §5 — Telemetry

```scl
// Pose feedback
"MAIN_KUKA_KR210_L150_L".MainRobotLeft_ActPos[1] := "MainRobotLeft_Kinematics_1".Tcp.x;
"MAIN_KUKA_KR210_L150_L".MainRobotLeft_ActPos[4] := "MainRobotLeft_Kinematics_1".Tcp.a;

// Joint angles feedback
"MAIN_KUKA_KR210_L150_L".MainRobotLeft_ActAng[1] := "MainRobotLeft_PositioningAxis_1".ActualPosition;

// Limit monitoring (stubs only ⚠️)
IF "MainRobotLeft_PositioningAxis_1".StatusWord.%X17 THEN ... END_IF; // HW min
IF "MainRobotLeft_PositioningAxis_1".StatusWord.%X18 THEN ... END_IF; // HW max

// Simulated dynamics ⚠️
"MAIN_KUKA_KR210_L150_L".MainRobotLeft_Dynamic_Torque[1] := "MainRobotLeft_PositioningAxis_1".ActualVelocity * 0.15;
"MAIN_KUKA_KR210_L150_L".MainRobotLeft_Axis_Power[1]  := "MainRobotLeft_PositioningAxis_1".ActualVelocity * 2.3;
```

---

## Motion FB Parameters (Common)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `AxesGroup` | `MainRobotLeft_Kinematics_1` | Kinematic group |
| `Velocity` | `MainRobotLeft_Vlcty` | Default 50 mm/s |
| `Acceleration/Deceleration/Jerk` | `-1` | TO default |
| `DirectionA` | `3` | Shortest path |
| `CoordSystem` | `0` | MCS |
| `BufferMode` | `2` (blend) / `1` (stop) | Blending |
| `TransitionParameter` | `#Temp_Transition_Blend/Stop` | [50.0 / 0.0] |
| `DynamicAdaption` | `-1` | No override |

---

## AAS Integration

| SCL Variable | AAS Path |
|--------------|----------|
| `MainRobotLeft_EnableAll` | `Inputs/MainRobotLeft/EnableAll` |
| `MainRobotLeft_Vlcty` | `Inputs/MainRobotLeft/Vlcty` |
| `MainRobotLeft_StackingLeft_PointsN` | `Inputs/MainRobotLeft/StackingLeft_Points/Point_N` |
| `MainRobotLeft_ActPos` | `Status/MainRobotLeft/ActPos` |
| `MainRobotLeft_Group_Fault` | `Status/MainRobotLeft/Group_Fault` |

**Submodels**: `KinematicsControl` | `OPCUA_Mapping` | `Status` | `DigitalTwin_Config`

```
/aas/KUKA_KR210_L150_L.json                    ← Main AAS
/sm/KUKA_KR210_L150_L/
    ├── KinematicsControl.json                 ← Inputs
    ├── OPCUA_Mapping.json                     ← NodeId map
    ├── Status.json                            ← Outputs
    └── DigitalTwin_Config.json                ← $MAMES + Isaac offsets
```

---

## OPC UA

- **Endpoint**: `opc.tcp://111.111.111.10:4840`
- **Namespace**: `3`
- **Pattern**: `ns=3;s="MAIN_KUKA_KR210_L150_L"."<Variable>"`

Example:
```
ns=3;s="MAIN_KUKA_KR210_L150_L"."MainRobotLeft_EnableAll"
ns=3;s="MAIN_KUKA_KR210_L150_L"."MainRobotLeft_ActPos"
```

Full mapping → `OPCUA_Mapping.json`

---

## Safety — Implemented vs TODO

| ✅ Implemented | ❌ TODO |
|----------------|---------|
| Hardware readiness check | E-Stop (F-CPU) |
| Auto-disable on fault | Workspace boundary |
| Command clear on fault | Collision detection |
| Fault flags to OPC UA | Hard limit handling |
| | Manual velocity clamp |

---

## Known Limitations

1. **Torque/Power** = simulated (`velocity × 0.15` / `× 2.3`) — replace with `SINA_PARA`
2. **Limit bits X15..X18** — stubs only (`#non := #non`)
3. **Single robot** — only `MainRobotLeft_*` implemented
4. **Manual mode** — only Home/Maint hardcoded; `TargetX/Y/Z` unused
5. **No state machine** — state inferred from FB `Active`/`Done`

---

## Adding a New Routine

1. Add points: `MainRobotLeft_StackingX_Points1..N`
2. Add trigger: `MainRobotLeft_StackingX` BOOL
3. Add `ELSIF` branch in §4, chain `MC_Stkng_X1..XN`
4. Terminate with `MC_UpTheLinePos_Return`
5. **No changes** to §1, §2, §3, §5

---

## Conventions

- DB prefix: `MainRobotLeft_*`
- Chain via `.Active`, not `.Done` (for blending)
- `-1` = use TO default for dynamics
- BufferMode: `2` between waypoints, `1` at end
- Comments: `// Section + purpose`

---

## Testing Checklist

- [ ] All 4 axes reach `System_Hardware_Ready` when `EnableAll = TRUE`
- [ ] `Trigger_Motion` pulses once per `EnableKinematics` rising edge
- [ ] Left stacking completes and returns to park
- [ ] Right stacking completes and returns to park
- [ ] Manual Home command moves robot to `HomePos`
- [ ] Manual Maintenance command moves robot to `Pos4Maint`
- [ ] Fault on any axis → `Group_Fault = TRUE` and kinematics disabled
- [ ] Telemetry arrays update every scan cycle

---

## References

- IEC 63278 (AAS) | IEC 62541 (OPC UA)
- PLCopen Motion Control Part 1 & 4
- Siemens S7-1500T Motion Control Manual
- NVIDIA Isaac Sim ROS2 Bridge

---

**Repository**: https://github.com/Nebras4u/isaac-plc-digital-twin  
**Last updated**: 2026-09-19
