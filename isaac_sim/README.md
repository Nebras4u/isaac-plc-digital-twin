# Yaskawa GP110 — Isaac Sim ↔ Siemens TIA Portal Bridge

> Setup guide for the **Yaskawa GP110** robot in **NVIDIA Isaac Sim**, configured to match the kinematic model supported by **Siemens TIA Portal** (`Articulated arm 3D with orientation — 4 Axis`), and bridged to a **Siemens S7-1500T PLC** over OPC UA.

---

## Table of Contents
1. [Overview](#1-overview)
2. [Why We Replaced the KUKA KR210](#2-why-we-replaced-the-kuka-kr210)
3. [Comparison: KR210 vs. Yaskawa GP110](#3-comparison-kr210-vs-yaskawa-gp110)
4. [Setup Steps in Isaac Sim](#4-setup-steps-in-isaac-sim)
5. [Action Graph Configuration](#5-action-graph-configuration)
6. [Measured Link Parameters](#6-measured-link-parameters)
7. [Verifying the Setup](#7-verifying-the-setup)
8. [TIA Portal Kinematics Mapping](#8-tia-portal-kinematics-mapping)
9. [Troubleshooting](#9-troubleshooting)
10. [Notes & Best Practices](#10-notes--best-practices)

---

## 1. Overview

This document describes the complete setup of the **Yaskawa GP110** robot inside NVIDIA Isaac Sim, and how it is configured to match the kinematic model supported by Siemens TIA Portal. The final result is a **4-axis controllable arm** (plus one hidden axis driven by Python for tool orientation), publishing both `/joint_states` and `/tf` for downstream integration with a **Siemens S7-1500T PLC** over OPC UA.

**Key outcomes:**
- ✅ Yaskawa GP110 imported and rigged as a proper articulation.
- ✅ Reduced from 6 axes to 4 controlled + 1 hidden tool axis.
- ✅ `/joint_states` and `/tf` published with real, dynamic values.
- ✅ Link parameters match the official Yaskawa datasheet.
- ✅ Direct mapping to TIA Portal kinematics fields (`L1`–`L4`, `LF`).

---

## 2. Why We Replaced the KUKA KR210

The original robot model in the project was a **KUKA KR210 L150**. During calibration, a fundamental mismatch was discovered:

- In the KR210 URDF, the axis **A2 is not horizontal** — it carries a vertical offset (**Z ≈ 419 mm**) between A2 and A3.
- **TIA Portal's `Articulated arm 3D with orientation` kinematics block only accepts a horizontal offset (`L2`) for axis A2.** There is no field to describe a vertical offset between A2 and A3.
- As a result, the KR210 could not be represented faithfully in TIA Portal without introducing kinematic errors.

The **Yaskawa GP110** was chosen as a replacement because:
- Its URDF geometry matches the TIA Portal kinematic model natively (A2 is horizontal, A3 is directly above, standard industrial articulated layout).
- Its measured link lengths match the official Yaskawa datasheet.
- It provides the required 6 joints from which we can derive the 4-axis + hidden-tool-axis configuration needed by the PLC.

---

## 3. Comparison: KR210 vs. Yaskawa GP110

| Aspect | KUKA KR210 L150 | Yaskawa GP110 |
|---|---|---|
| Axis A2 | Not horizontal (vertical offset) | Horizontal |
| TIA Portal fit | ❌ Kinematic mismatch | ✅ Native match |
| Link lengths (measured) | Non-standard vs. datasheet | Match datasheet |
| Payload | 150 kg | 110 kg |
| Reach | ~3100 mm | ~2237 mm |
| Axes (original) | 6 | 6 |
| Axes (after reduction) | 6 (unchanged) | **5** (4 controlled + 1 hidden) |
| Frame root name | `kr210_l150` | `Main_Robot_L_Yaskawa_GP110` |
| TF frame names | `link_1` … `link_6`, `tool0` | `link_1` … `link_6` |

**Conclusion:** The Yaskawa GP110 is a drop-in match for the TIA Portal kinematic model. The KR210 required workarounds that introduced unacceptable kinematic error.

---

## 4. Setup Steps in Isaac Sim

### 4.1 Import the URDF
Import the file `Yaskawa_GP110_v1/GP110.urdf` using `File → Import`. Isaac Sim will generate the articulation and the associated prim hierarchy.

### 4.2 Move the Root Prim into the World
After import, drag the robot's root prim into the `World` prim so that it becomes a proper child of the stage at `/World/Main_Robot_L_Yaskawa_GP110`. This ensures TF publishers resolve the correct frame hierarchy and that the robot is subject to the stage's physics.

### 4.3 Delete the Duplicate Base Joint Path
Depending on the URDF, Isaac Sim may generate **two parallel joint paths**: `link_1 → link_2 → … → tool0` and `base_link → base_joint → …`. If this happens, **delete `base_joint`**. Only one root path must remain — otherwise the articulation will be broken and TF will publish ambiguous transforms.

### 4.4 Mark `base_link` as Articulation Root *(Important)*
Select `base_link` → `Add → Physics → Articulation Root`. This is **mandatory**. Without it, Isaac Sim will not treat the kinematic chain as an articulation, and `/joint_states` and `/tf` will not be published correctly.

### 4.5 Fix `base_link` to the World
Select `base_link` → `Create → Physics → Joint → Fixed Joint`. This anchors the robot to the world frame so it does not fall under gravity.

### 4.6 Configure Joint Drives
Select **all joints** and set:

| Parameter | Value |
|---|---|
| Damping | **1000** |
| Stiffness | **10000** |

These values provide stable, non-oscillating joint behavior under gravity and external loads, while still allowing ROS 2 commands to drive the joints smoothly.

### 4.7 Reduce the Robot from 6 Axes to 5 (4 Controlled + 1 Hidden)
The robot is a 6-axis Yaskawa GP110, but the PLC only needs **4 controllable axes**. The fifth axis is kept hidden and driven by Python to maintain the tool pointing downward at all times.

**Steps:**
1. **Delete `joint4`** from the articulation.
2. **Add a fixed joint** in its place, under a different name (e.g., `joint4_fixed`), preserving the geometric offset of the original axis.
3. **Rename `Joint 6` to `Joint 4`** so that the final articulation exposes 5 named joints:

| New name | Role |
|---|---|
| `joint_1` | A1 — base rotation |
| `joint_2` | A2 — shoulder |
| `joint_3` | A3 — elbow |
| `joint_4` | A4 — tool roll (was `joint_6`) |
| `joint_5` | A5 — hidden, driven by Python to keep tool downward |

This mapping matches the 4-axis kinematics expected by the Siemens TIA Portal `Articulated arm 3D with orientation` block.

---

## 5. Action Graph Configuration

Create an Action Graph with the following nodes:

| Node | Purpose |
|---|---|
| `On Playback Tick` | Fires every simulation step |
| `Isaac Read Simulation Time` | Provides the timestamp for messages |
| `ROS2 Publish Joint State` | Publishes `/joint_states` |
| `ROS2 Publish Transform Tree` | Publishes `/tf` |

**Connections:**

OnPlaybackTick (exec) -> ROS2 Publish Joint State

OnPlaybackTick (exec) -> ROS2 Publish Transform Tree

IsaacReadSimulationTime (timestamp) -> both publishers

**Settings:**
- `ROS2 Publish Transform Tree → targetPrims`: `/World/Main_Robot_L_Yaskawa_GP110`
- `ROS2 Publish Transform Tree → static`: `false`
- `ROS2 Publish Joint State → targetPrim`: robot articulation root

### Isaac Sim — ROS2 Action Graph
![Isaac Sim Action Graph](https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/isaac_sim/Yaskawa_GP110.jpg)

**Important:** Do **not** add any subscriber node. The bridge is publish-only.

---

## 6. Measured Link Parameters

All values measured from `/tf` with every joint at **0 rad** (mechanical zero):

| Parameter | Value (mm) | Description |
|---|---|---|
| **L1** | **540** | Base height (base → A2) |
| **L2** | **320** | Horizontal offset of A2 |
| **L3** | **870** | Distance A2 → A3 (vertical) |
| **L4** | **1020** | Distance A3 → flange |
| **LF** | *to be measured* | Flange length (`link_6` → `tool0`) |

These values are entered directly into the TIA Portal kinematics configuration.

**Reference (Yaskawa GP110 datasheet):**

| Parameter | Datasheet | Measured | Match |
|---|---|---|---|
| L1 | 540 mm | 540 mm | ✅ |
| L2 | 320 mm | 320 mm | ✅ |
| L3 | 870 mm | 870 mm | ✅ |
| L4 | 1020 mm | 1020 mm | ✅ |

---

## 7. Verifying the Setup

With Isaac Sim playing, run:

ros2 topic echo /joint_states --once

ros2 topic echo /tf --once | grep child_frame_id

**Expected result:**
- `/joint_states` publishes **5 joints** with real values.
- `/tf` publishes **6 frames** (`link_1` … `link_6`) with non-zero translations and valid quaternions.
- At mechanical zero, all `w ≈ 1.0` and translations match the table in Section 6.

**Example `/tf` output at mechanical zero:**

frame_id: Main_Robot_L_Yaskawa_GP110

child_frame_id: link_1 -> translation: { x: 0.000, y: 0.000, z: 0.540 }

child_frame_id: link_2 -> translation: { x: 0.320, y: 0.000, z: 0.540 }

child_frame_id: link_3 -> translation: { x: 0.321, y: 0.000, z: 1.410 }

child_frame_id: link_4 -> translation: { x: 0.321, y: 0.000, z: 1.645 }

child_frame_id: link_5 -> translation: { x: 1.341, y: 0.000, z: 1.643 }

child_frame_id: link_6 -> translation: { x: 1.341, y: 0.000, z: 1.643 }

---

## 8. TIA Portal Kinematics Mapping

In TIA Portal, configure the technology object as `Articulated arm 3D with orientation (4 Axis)`. Enter the measured values into the corresponding fields:

| TIA Portal field | Value | Source |
|---|---|---|
| **Length L1** | **540 mm** | Base height (base → A2) |
| **Length L2** | **320 mm** | Horizontal offset of A2 |
| **Length L3** | **870 mm** | Distance A2 → A3 |
| **Length L4** | **1020 mm** | Distance A3 → flange |
| **Flange length LF** | *to be measured* | `link_6` → `tool0` |
| **Compensation factor** | **0.0** | No mechanical coupling |

**Notes:**
- `L1` and `L2` define the position of axis A2 relative to the kinematic zero point (KZP).
- `L3` is the distance between axes A2 and A3.
- `L4` is the distance from A3 to the mandatory coupling point.
- `LF` is the flange length from the coupling point to the flange coordinate system (FCS).

---

## 9. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `/tf` publishes all zeros | `ROS2 Publish Transform Tree` missing or `targetPrims` empty | Add the node and set `targetPrims = /World/Main_Robot_L_Yaskawa_GP110` |
| `/joint_states` shows values, but `/tf` does not change | TF publisher not connected to `OnPlaybackTick` | Connect the exec ports |
| `joint_4` missing from TF | Original joint_4 was deleted but not renamed | Rename `joint_6` → `joint_4` |
| Duplicate joint paths (`link_1` and `base_joint`) | URDF import created two roots | Delete `base_joint` |
| Robot falls under gravity | `base_link` not fixed | Add fixed joint to world |
| Robot not treated as articulation | `base_link` missing Articulation Root | Add `Physics → Articulation Root` |
| Joints oscillate or jitter | Drive damping too low | Set `Damping=1000`, `Stiffness=10000` |
| `tf2_echo` reports "frame does not exist" | Lookup runs before first TF message | Wait ~2 s; it is transient |

---

## 10. Notes & Best Practices

- The `ROS2 Publish Transform Tree` node must have `static = false` and a valid `targetPrims` path; otherwise it publishes empty transforms (all zeros).
- Do **not** create a subscriber node — the bridge is publish-only.
- If you see duplicate frame paths after import, delete the non-articulation root (`base_joint`) before proceeding.
- Always verify the robot is at **mechanical zero** before measuring link parameters.
- The 5th axis (`joint_5`) is intentionally hidden from the PLC and driven by Python to keep the tool pointing downward.
- The `link_5` and `link_6` frames often coincide because the wrist axes are mechanically collinear — this is expected.

---

## Appendix A — Frame Hierarchy (after setup)

Main_Robot_L_Yaskawa_GP110   (root, fixed to world)

- link_1  (A1)
  - link_2  (A2)
    - link_3  (A3)
      - link_4  (A4, formerly joint_6)
        - link_5  (A5, hidden, Python-driven)
          - link_6  (tool flange)
            - tool0  (TCP)

## Appendix B — ROS 2 Topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Isaac → ROS 2 | Joint angles |
| `/tf` | `tf2_msgs/TFMessage` | Isaac → ROS 2 | Dynamic transforms |
| `/tf_static` | `tf2_msgs/TFMessage` | Isaac → ROS 2 | Static transforms |
| `/clock` | `rosgraph_msgs/Clock` | Isaac → ROS 2 | Simulation time |

## Appendix C — Default Joint Names

| Index | Name | Role |
|---|---|---|
| 1 | `joint_1` | Base rotation (A1) |
| 2 | `joint_2` | Shoulder (A2) |
| 3 | `joint_3` | Elbow (A3) |
| 4 | `joint_4` | Tool roll (A4, was `joint_6`) |
| 5 | `joint_5` | Hidden tool axis (Python-driven) |

---
