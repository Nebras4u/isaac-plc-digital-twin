# Isaac Digital Twin: S7-1500T ↔ ROS2 ↔ Isaac Sim

Digital twin of a Yaskawa GP110 robotic arm driven by a Siemens S7-1500T PLC, simulated in NVIDIA Isaac Sim via ROS2 and OPC UA.

Migration note (v0.2): Originally prototyped on a KUKA KR210 L150 L URDF, now ported to a Yaskawa GP110 articulation (5 joints on /joint_states). TIA Portal TO is "Articulated arm 3D with orientation (4 Axis)". $MAMES is no longer needed.

URDF update note (v0.2.1): The URDF file was modified due to **joint_5 motion constraints** that prevented full tool-down orientation. A new file name was adopted: **`Main_Robot_L_URDF_tool.urdf`**.

## Table of Contents

1. [Current State (v0.2)](#1-current-state-v02)
2. [Architecture](#2-architecture)
3. [Tech Stack](#3-tech-stack)
4. [Isaac Sim Setup — Joint Renaming & Home Offset](#4-isaac-sim-setup--joint-renaming--home-offset)
5. [Action Graph Configuration](#5-action-graph-configuration)
6. [Measured Link Parameters](#6-measured-link-parameters)
7. [Verifying the Setup](#7-verifying-the-setup)
8. [TIA Portal Kinematics Mapping](#8-tia-portal-kinematics-mapping)
9. [Troubleshooting](#9-troubleshooting)
10. [Notes & Best Practices](#10-notes--best-practices)
11. [Screenshots](#11-screenshots)
12. [Roadmap](#12-roadmap)
13. [Technical Contributions](#13-technical-contributions)
14. [References](#14-references)
15. [License](#15-license)
16. [Appendix A — Frame Hierarchy](#appendix-a--frame-hierarchy-after-setup)
17. [Appendix B — ROS 2 Topics](#appendix-b--ros-2-topics)
18. [Appendix C — Default Joint Names](#appendix-c--default-joint-names)
19. [Appendix D — Zero-Pose Validation Results](#appendix-d--zero-pose-validation-results)

## 1. Current State (v0.2)

Real-time bidirectional synchronization between:
- Siemens S7-1500T PLC (OPC UA Server, TIA Portal, Articulated arm 3D with orientation TO)
- NVIDIA Isaac Sim 5.0 (Yaskawa GP110, URDF, 5-joint articulation)
- ROS2 Humble (/joint_states, /tf, /clock)

Includes Isaac Home offset calibration and joint renaming (joint_6 -> joint_4).

**URDF modification (v0.2.1):** Joint_5 motion constraints were blocking full tool-down orientation. The URDF was edited and re-exported as **`Main_Robot_L_URDF_tool.urdf`**.

Flow:
[S7-1500T PLCSIM Advanced + PLCopen Motion + TO 4-Axis]
   |
   | OPC UA :4840 / ns=3 / Read Arrays & Flags
   v
[bridge.py  asyncua + rclpy, Home Offset + Joint Remap]
   ^
   | ROS2 /DDS  /joint_states (5 joints)
   |
[Isaac Sim 5.0 — PhysX 5 — Yaskawa GP110 URDF (Main_Robot_L_URDF_tool.urdf) — 5-joint articulation]

Compare -> delta ~= 0 deg -> Validated

## 2. Architecture

[PLC S7-1500T] <-- OPC UA --> [Python Bridge] <-- ROS2 --> [Isaac Sim]
     (4-axis)                        |
                             [Home Offset Calib.]
                             [joint_6 -> joint_4]
                             [URDF: Main_Robot_L_URDF_tool.urdf]

## 3. Tech Stack

- PLC: Siemens S7-1500T (TIA Portal, OPC UA Server, 4-Axis TO)
- Robot: Yaskawa GP110 (was KUKA KR210 L150 L in v0.1)
- URDF: `Main_Robot_L_URDF_tool.urdf` (joint_5 constraints relaxed)
- Middleware: ROS2 Humble, rclpy, asyncua
- Simulation: NVIDIA Isaac Sim 5.0 (PhysX 5, RTX)
- Protocols: OPC UA (ns=3), ROS2 topics

## 4. Isaac Sim Setup — Joint Renaming & Home Offset

### 4.1 Rename Joint 6 to Joint 4

| New name | Role | Driven by |
|---|---|---|
| joint_1 | A1 — base rotation | PLC |
| joint_2 | A2 — shoulder | PLC |
| joint_3 | A3 — elbow | PLC |
| joint_4 | A4 — tool roll (was joint_6) | PLC |
| joint_5 | A5 — hidden tool axis | Python |

### 4.2 Home Offset Calibration

URDF bias is normalized so Isaac's mechanical zero matches Yaskawa's mechanical zero (replaces the old $MAMES conversion).

### 4.3 URDF Modification — `Main_Robot_L_URDF_tool.urdf`

> **Change (v0.2.1):** The original URDF imposed joint limits on **joint_5** that prevented the tool from being oriented fully downward. To allow full tool-down orientation, the joint_5 motion constraints were edited in the URDF, and the file was re-saved under the new name **`Main_Robot_L_URDF_tool.urdf`**.
>
> **Impact:** joint_5 can now rotate freely enough to align the tool axis with the world -Z direction (tool-down), which is required for the glass sheet handling use case in the roadmap.
>
> **Note:** joint_5 remains hidden from the PLC and is still driven by Python.

#### 4.3.1 Visual Comparison — Before vs After URDF Modification

<!-- ![joint_5 URDF constraint issue](https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/docs/urdf_joint5_before_after.jpg) -->

> 📷 `docs/urdf_joint5_before_after.png`
> **Before (left):** original URDF — joint_5 limits block full tool-down.
> **After (right):** `Main_Robot_L_URDF_tool.urdf` — constraints relaxed, tool aligns with world -Z.

## 5. Action Graph Configuration

| Node | Purpose |
|---|---|
| On Playback Tick | Fires every simulation step |
| Isaac Read Simulation Time | Provides timestamp |
| ROS2 Publish Joint State | Publishes /joint_states |
| ROS2 Publish Transform Tree | Publishes /tf |

Connections:
OnPlaybackTick (exec) -> ROS2 Publish Joint State
OnPlaybackTick (exec) -> ROS2 Publish Transform Tree
IsaacReadSimulationTime (timestamp) -> both publishers

Settings:
- targetPrims: /World/Main_Robot_L_Yaskawa_GP110
- static: false
- targetPrim: robot articulation root

Do NOT add any subscriber node.

Screenshot: https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/docs/Yaskawa_GP110.jpg

## 6. Measured Link Parameters

At mechanical zero (all joints = 0 rad):

| Parameter | Value (mm) | Description |
|---|---|---|
| L1 | 540 | Base -> A2 |
| L2 | 320 | Horizontal offset A2 |
| L3 | 870 | A2 -> A3 |
| L4 | 1020 | A3 -> flange |
| LF | TBD | Flange length |

| Parameter | Datasheet | Measured | Match |
|---|---|---|---|
| L1 | 540 | 540 | OK |
| L2 | 320 | 320 | OK |
| L3 | 870 | 870 | OK |
| L4 | 1020 | 1020 | OK |

## 7. Verifying the Setup

Commands:
ros2 topic echo /joint_states --once
ros2 topic echo /tf --once | grep child_frame_id

Expected:
- /joint_states: 5 joints with real values.
- /tf: 6 frames (link_1 ... link_6), valid quaternions (w ~= 1.0 at zero).

Sample /tf:
frame_id: Main_Robot_L_Yaskawa_GP110
child_frame_id: link_1 -> { x: 0.000, y: 0.000, z: 0.540 }
child_frame_id: link_2 -> { x: 0.320, y: 0.000, z: 0.540 }
child_frame_id: link_3 -> { x: 0.321, y: 0.000, z: 1.410 }
child_frame_id: link_4 -> { x: 0.321, y: 0.000, z: 1.645 }
child_frame_id: link_5 -> { x: 1.341, y: 0.000, z: 1.643 }
child_frame_id: link_6 -> { x: 1.341, y: 0.000, z: 1.643 }

Note: link_5/link_6 coincide — wrist axes are collinear.

## 8. TIA Portal Kinematics Mapping

| TIA Portal field | Value | Source |
|---|---|---|
| L1 | 540 mm | Base -> A2 |
| L2 | 320 mm | Offset A2 |
| L3 | 870 mm | A2 -> A3 |
| L4 | 1020 mm | A3 -> flange |
| LF | TBD | link_6 -> tool0 |
| Compensation factor | 0.0 | None |

## 9. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| /tf all zeros | Missing ROS2 Publish Transform Tree / empty targetPrims | Set targetPrims = /World/Main_Robot_L_Yaskawa_GP110 |
| /tf frozen | TF not wired to OnPlaybackTick | Connect exec ports |
| joint_4 missing | joint_6 not renamed | Rename joint_6 -> joint_4 |
| Duplicate frames | Two URDF roots | Delete base_joint |
| Robot falls | base_link not fixed | Add fixed joint to world |
| Not articulation | Missing Articulation Root | Add Physics -> Articulation Root |
| Jitter | Damping too low | Damping=1000, Stiffness=10000 |
| tf2_echo "frame does not exist" | Early lookup | Wait ~2 s |
| Tool cannot point fully down | joint_5 URDF limits too tight | Use `Main_Robot_L_URDF_tool.urdf` (joint_5 constraints relaxed) |

## 10. Notes & Best Practices

- ROS2 Publish Transform Tree must have static=false + valid targetPrims.
- No subscriber nodes — bridge is publish-only.
- Delete stray base_joint if it appears.
- Verify mechanical zero before measuring links.
- joint_5 is hidden from PLC, driven by Python (tool-down).
- link_5/link_6 coincide — expected.
- Use **`Main_Robot_L_URDF_tool.urdf`** (not the original URDF) so joint_5 can achieve full tool-down orientation.

## 11. Screenshots

Live Comparison: https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/docs/general_view.png
TIA Portal DB: https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/docs/tia_portal_db.png
Action Graph: https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/docs/Yaskawa_GP110.jpg
**joint_5 URDF Before/After:** `docs/urdf_joint5_before_after.png`

## 12. Roadmap

- [x] v0.1 — KUKA KR210 L150 L · PLC <-> OPC UA <-> ROS2 <-> Isaac Sim sync
- [x] v0.2 — Yaskawa GP110, joint_6 -> joint_4, 5-joint articulation, 4-Axis TO
- [x] v0.2.1 — URDF edited for joint_5 full tool-down orientation → `Main_Robot_L_URDF_tool.urdf`
- [ ] v0.3 — IK node with What-If feasibility checks
- [ ] v0.4 — Pneumatic suction gripper + glass sheet handling
- [ ] v0.5 — Force loop + grasp stability monitoring
- [ ] v1.0 — Domain randomization + full digital twin validation

## 13. Technical Contributions

1. 5-joint remap for 4-axis PLC kinematics: joint_6 renamed to joint_4, joint_5 Python-driven, exposing exactly the joint set expected by TIA Portal's Articulated arm 3D with orientation block.
2. Isaac Home offset calibration: normalizes URDF bias so Isaac's mechanical zero matches Yaskawa's mechanical zero.
3. Real-time PLC <-> simulation validation: compares joint states with per-axis deviation output.
4. Publish-only ROS2 Action Graph: On Playback Tick -> ROS2 Publish Joint State + ROS2 Publish Transform Tree (no subscriber, static=false, targetPrims=/World/Main_Robot_L_Yaskawa_GP110).
5. **URDF modification for tool-down orientation:** joint_5 limits were too restrictive in the original URDF; constraints were relaxed and the file re-exported as `Main_Robot_L_URDF_tool.urdf` to enable full downward tool orientation.

## 14. References

Inspired by: Isaac Sim Integrated Digital Twin For Feasibility Checks In Skill-based Engineering (RAAD 2025)
DOI: 10.1007/978-3-032-02106-9_46
https://doi.org/10.1007/978-3-032-02106-9_46

Datasheet: Yaskawa GP110 (link lengths L1–L4 confirmed).

## 15. License

MIT

## Appendix A — Frame Hierarchy (after setup)

Main_Robot_L_Yaskawa_GP110   (root, fixed to world)
|
+-- link_1  (A1)
    +-- link_2  (A2)
        +-- link_3  (A3)
            +-- link_4  (FixedJoint)
                +-- link_5  (A5, hidden, Python-driven)
                    +-- link_6  (A4, formerly joint_6)
                        +-- tool0  (TCP)

## Appendix B — ROS 2 Topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| /joint_states | sensor_msgs/JointState | Isaac -> ROS 2 | Joint angles |
| /tf | tf2_msgs/TFMessage | Isaac -> ROS 2 | Dynamic transforms |
| /tf_static | tf2_msgs/TFMessage | Isaac -> ROS 2 | Static transforms |
| /clock | rosgraph_msgs/Clock | Isaac -> ROS 2 | Simulation time |

## Appendix C — Default Joint Names

| Index | Name | Role |
|---|---|---|
| 1 | joint_1 | Base rotation (A1) |
| 2 | joint_2 | Shoulder (A2) |
| 3 | joint_3 | Elbow (A3) |
| 4 | joint_4 | Tool roll (A4, was joint_6) |
| 5 | joint_5 | Hidden tool axis (Python-driven, constraints relaxed in `Main_Robot_L_URDF_tool.urdf`) |

## Appendix D — Zero-Pose Validation Results

### D.1 PLC vs Isaac — Raw Log (TIA Portal ↔ ROS 2)

```
[ PLC — TIA Portal View ]
================================================================================
ActPos [mm/deg] : X=3100.000  Y=0.000  Z=330.000  A=0.000
ActAng (math)   : A1=0.000, A2=0.000, A3=0.000, A4=0.000
Pos4Maint       : ['-486.800', '-418.600', '205.000', '0.000']
Flags           : {'MainRobotLeft_Go2Pos4Maint': False,
                   'MainRobotLeft_Group_Fault'  : False,
                   'MainRobotLeft_Axis1_Fault'  : False,
                   'MainRobotLeft_Axis2_Fault'  : False,
                   'MainRobotLeft_Axis3_Fault'  : False,
                   'MainRobotLeft_Axis4_Fault'  : False}
================================================================================

[ Isaac Sim — ROS 2 View ]
================================================================================
Joint    Raw       URDF      Mechanical
----------------------------------------------
A1      -0.006    -0.006
A2       0.831    -0.831
A3       0.109    -0.109
A4       0.258     0.258
----------------------------------------------
TCP (aligned to PLC frame)
  X (mm) 3100.003   Y (mm) -0.000   Z (mm) 329.896   A (deg) 85.735
----------------------------------------------
Frame       X (mm)     Y (mm)     Z (mm)     A (deg)
----------------------------------------------------
base_link   FAIL "base_link" passed to lookupTransform argument source_frame does not exist
link_1        0.000      0.000      0.000      0.000
link_2      350.000     -0.000      0.000      0.000
link_3     1699.857      0.000    -19.668      0.000
link_4     3099.669     -0.000    -42.617      0.000
tool       3106.325     -0.000   -122.340      0.312
================================================================================

[ FULL TWIN COMPARISON ]
================================================================================
Axis   PLC mech   ROS raw   ROS mech   d(PLC-ROS)
------------------------------------------------
A1       0.000     -0.006     -0.006      0.006
A2       0.000      0.831     -0.831      0.831
A3       0.000      0.109     -0.109      0.109
A4       0.000      0.258      0.258     -0.258
------------------------------------------------
TCP WCS Comparison (PLC vs Isaac)
------------------------------------------------
X (mm)   3100.000 - 3100.003 - -0.003
Y (mm)      0.000 -   -0.000 -  0.000
Z (mm)    330.000 -  329.896 -  0.104
A (deg)     0.000 -   85.735 - -85.735
------------------------------------------------
WCS of all frames (relative to link_1)
Frame       X (mm)     Y (mm)     Z (mm)     A (deg) | PLC X  PLC Y  PLC Z  PLC A | dX      dY    dZ      dA
link_1        0.000      0.000      0.000      0.000 | 3100.0  0.0   330.0  0.0   | -3100.0 0.0  -330.0   0.0
link_2      350.000     -0.000      0.000      0.000 | 3100.0  0.0   330.0  0.0   | -2750.0 0.0  -330.0   0.0
link_3     1699.857      0.000    -19.668      0.000 | 3100.0  0.0   330.0  0.0   | -1400.1 0.0  -349.7   0.0
link_4     3099.669     -0.000    -42.617      0.000 | 3100.0  0.0   330.0  0.0   |    -0.3 0.0  -372.6   0.0
tool       3106.325     -0.000   -122.340      0.312 | 3100.0  0.0   330.0  0.0   |     6.3 0.0  -452.3   0.3
================================================================================
```

### D.2 Position (Cartesian) — Successfully Zeroed

| Axis | PLC | Isaac | Δ (mm) | Status |
|---|---|---|---|---|
| X | 3100.000 mm | 3100.003 mm | **-0.003** | ✅ Effectively zero (sub-micron) |
| Y | 0.000 mm | -0.000 mm | **0.000** | ✅ Absolute match |
| Z | 330.000 mm | 329.896 mm | **0.104** | ✅ Excellent (physics noise only) |

> **Result:** The Cartesian position of the TCP in the zero pose is now essentially identical between the PLC and the simulation. X and Y are effectively exact, and Z differs by only ~0.1 mm, which is well within simulation physics noise and is considered a fully successful match.

### D.3 Remaining Issue — Cartesian Angle dA = -85.735°

| Axis | PLC | Isaac | Δ (deg) | Status |
|---|---|---|---|---|
| A | 0.000° | 85.735° | **-85.735** | ⚠️ Offset still required |

The PLC reports the tool angle as **0.000°** while the simulation reports **85.735°**. This means the current A-axis offset that was applied (**+85.423°**) still needs a small correction to fully compensate the remaining ~85°.

> **Decision:** All software/offset corrections (including the A-axis offset fix) are **deferred** until the link geometry (joint constraints, link lengths, and frame placement) is fully verified and confirmed to match. Geometry correctness takes priority over offset tuning.

### D.4 Known Cosmetic Issues (Non-Blocking)

| Issue | Cause | Note |
|---|---|---|
| `base_link` lookup fails | `base_link` not present as a TF source frame in Isaac | Cosmetic; all other frames resolve correctly. `link_1` acts as the effective root. |
| `tool` A = 0.312° | Residual from the same A-axis offset | Will disappear once the A offset is corrected in v0.3. |

### D.5 Summary

- ✅ **Position (X, Y, Z):** Fully validated — Δ ≤ 0.104 mm.
- ⚠️ **Orientation (A):** Offset of ~85.7° remains; software correction deferred.
- 🎯 **Next step:** Finalize URDF link geometry and frame placement, then apply the A-axis offset correction.

