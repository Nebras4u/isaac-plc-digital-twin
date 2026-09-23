# Yaskawa GP110 — Isaac Sim ↔ Siemens TIA Portal Bridge

> Setup guide for the **Yaskawa GP110** robot in **NVIDIA Isaac Sim**, configured to match the kinematic model supported by **Siemens TIA Portal** (`Articulated arm 3D with orientation — 4 Axis`), and bridged to a **Siemens S7-1500T PLC** over OPC UA.

---

## Table of Contents
1. [Overview](#1-overview)
2. [Why We Replaced the KUKA KR210](#2-why-we-replaced-the-kuka-kr210)
3. [The Link-3 (L-shaped) Problem](#3-the-link-3-l-shaped-problem)
4. [Decision: Hand-Crafted URDF](#4-decision-hand-crafted-urdf)
5. [Setup Steps in Isaac Sim](#5-setup-steps-in-isaac-sim)
6. [Action Graph Configuration](#6-action-graph-configuration)
7. [Measured Link Parameters](#7-measured-link-parameters)
8. [TIA Portal Kinematics Mapping](#8-tia-portal-kinematics-mapping)
9. [Troubleshooting](#9-troubleshooting)
10. [Notes & Best Practices](#10-notes--best-practices)

---

## 1. Overview

Setup of the **Yaskawa GP110** in NVIDIA Isaac Sim, configured to match the Siemens TIA Portal kinematic model. Final result: **4-axis controllable arm + 1 hidden tool axis**, publishing `/joint_states` and `/tf`, bridged to an **S7-1500T PLC** over OPC UA.

**Key outcomes:**
- ✅ Yaskawa GP110 imported and rigged as a proper articulation.
- ✅ Reduced from 6 axes to 4 controlled + 1 hidden tool axis.
- ✅ `/joint_states` and `/tf` published with real, dynamic values.
- ✅ Link parameters match the official Yaskawa datasheet.
- ✅ Direct mapping to TIA Portal kinematics fields (`L1`–`L4`, `LF`).

---

## 2. Why We Replaced the KUKA KR210

The original model was a **KUKA KR210 L150**, but a fundamental mismatch was found:

- In the KR210 URDF, **A2 is not horizontal** — it carries a vertical offset between A2 and A3.
- **TIA Portal's `Articulated arm 3D with orientation` block only accepts a horizontal offset (`L2`) for A2.** No field exists for a vertical offset.
- The KR210 could not be represented faithfully without kinematic errors.

The **Yaskawa GP110** was chosen because its URDF geometry matches the TIA Portal kinematic template natively, and its link lengths match the official datasheet.

---

## 3. The Link-3 (L-shaped) Problem

After importing the Yaskawa GP110 URDF and reducing it from 6 to 5 axes, a critical mismatch was found between the **Isaac Sim model** and the **TIA Portal kinematic model**.

### 3.1 Expected (TIA Portal)

In TIA Portal, `link_3` (the elbow-to-flange segment, labeled **L4**) is modeled as a **straight horizontal link** (`_`). The flange traces a **smooth circular arc** determined entirely by joint angles and straight link lengths.

| |___
| |___ ← link_3 = straight horizontal
|
|___

### 3.2 Actual (Isaac Sim / Yaskawa GP110)

In the imported URDF, `link_3` is **not** a straight continuation. It forms an **L-shaped bent link** with an extra **vertical offset (~235 mm)** and **horizontal offset (~1020 mm)** between the elbow and the flange.

| |___
| |
| |___ ← link_3 = L-shaped bend
|
|___

### 3.3 Why This Breaks the Kinematics

| Consequence | Explanation |
|---|---|
| **Different arc in space** | Because `link_3` is bent, the flange traces a **different circular arc** than the one TIA Portal computes. On curved paths, the real TCP and the simulated TCP diverge. |
| **Wrong TCP height** | At mechanical zero, the simulated TCP sits **~235 mm higher** than the TCP calculated by TIA (`dZ = +233.46 mm`). |
| **Not representable in TIA** | TIA's `Articulated arm 3D with orientation` only supports **straight link segments**. There is no parameter for an L-shaped link. |
| **Cartesian comparison invalid** | The error is **pose-dependent** — it cannot be fixed by a simple offset. |

### 3.4 Evidence from Live Data

At mechanical zero (`A1=0`, `A2=90`, `A3=-90`, `A4=0`):

| Source | X (mm) | Y (mm) | Z (mm) |
|---|---|---|---|
| PLC `ActPos` | 1340.000 | 0.000 | 1410.000 |
| Isaac `link_3` | 320.772 | -0.002 | 1410.001 |
| Isaac `link_4` | 321.128 | -0.002 | 1645.001 |
| Isaac `link_5` | 1341.127 | -0.007 | 1643.458 |

- `link_3` matches PLC in **Z** but not **X**.
- `link_5` matches PLC in **X** but not **Z**.
- **No link matches both** — the L-shaped geometry is the root cause.

### 3.5 Illustration

![Link-3 Geometry Mismatch](isaac_sim/Yaskawa_VS-TIA-Portal.jpg)

---

## 4. Decision: Hand-Crafted URDF

### 4.1 Rationale

The Yaskawa GP110 URDF is geometrically correct for the Yaskawa robot but **not kinematically compatible** with TIA Portal's `Articulated arm 3D with orientation` model — because of the L-shaped `link_3`. Fixing this properly requires rebuilding the URDF.

Rather than spend additional weeks hunting for a URDF that:
- matches TIA's kinematic template exactly (straight links),
- carries the physical parameters of the robots originally intended for the cell (KUKA KR210 class),
- and remains compatible with the existing PLC program and the 4-axis + hidden-tool-axis layout,

**we have decided to move forward with a hand-crafted URDF.**

### 4.2 What the Hand-Crafted URDF Provides

- **Straight link geometry** between A2 → A3 → flange (no L-bend) — directly matching TIA's kinematics template.
- **Physical parameters close to the originally intended robots** (KUKA KR210 / Yaskawa GP110 class): link lengths (L1–L4, LF), masses, inertias, joint limits, velocities, efforts.
- **4 controllable axes + 1 hidden tool axis**, exposed with the same joint names (`joint_1` … `joint_5`) already used by the ROS 2 ↔ PLC bridge.
- **Single-file URDF**, easy to adjust as measurement data from the real cell becomes available.

### 4.3 Why This Is the Right Trade-Off

| Criterion | Continue with GP110 URDF | Hand-Crafted URDF |
|---|---|---|
| TIA Portal compatibility | ❌ L-shaped link_3 | ✅ Straight links |
| Time to resolution | Weeks of workarounds | Days |
| Physical realism | ✅ High (real Yaskawa) | ⚠️ Approximate (closest match) |
| Future maintainability | ⚠️ Fragile | ✅ Fully under our control |
| Trajectory accuracy | ❌ Different arc in space | ✅ Arc matches TIA |

The project is **time-boxed**; cell kinematics is the priority, visualization realism is secondary. The hand-crafted URDF preserves PLC-side correctness while remaining visually representative.

### 4.4 Illustration

![Hand-Crafted URDF in Isaac Sim](isaac_sim/handcrafted_urdf.jpg)

---

## 5. Setup Steps in Isaac Sim

### 5.1 Import the URDF
Import `Yaskawa_GP110_v1/GP110.urdf` via `File → Import`. Isaac Sim generates the articulation and prim hierarchy.

### 5.2 Move Root Prim into World
Drag the robot's root prim into `World` so it becomes `/World/Main_Robot_L_Yaskawa_GP110`.

### 5.3 Delete Duplicate Base Joint Path
If Isaac generates two parallel joint paths (`link_1 → link_2 → …` and `base_link → base_joint → …`), **delete `base_joint`**.

### 5.4 Mark `base_link` as Articulation Root *(Mandatory)*
Select `base_link` → `Add → Physics → Articulation Root`. Without it, `/joint_states` and `/tf` will not publish correctly.

### 5.5 Fix `base_link` to World
Select `base_link` → `Create → Physics → Joint → Fixed Joint`.

### 5.6 Configure Joint Drives
Set all joints: **Damping = 1000**, **Stiffness = 10000**.

### 5.7 Reduce from 6 Axes to 5 (4 Controlled + 1 Hidden)
1. **Delete `joint4`**, replace with a **fixed joint** (`joint4_fixed`).
2. **Rename `Joint 6` → `Joint 4`**.

| New name | Role |
|---|---|
| `joint_1` | A1 — base rotation |
| `joint_2` | A2 — shoulder |
| `joint_3` | A3 — elbow |
| `joint_4` | A4 — tool roll (was `joint_6`) |
| `joint_5` | A5 — hidden, Python-driven (tool downward) |

> ⚠️ The above reduction was performed on the GP110 URDF and revealed the L-shaped `link_3` problem (Section 3). The same reduction will be applied to the hand-crafted URDF, without the L-shaped geometry.

---

## 6. Action Graph Configuration

| Node | Purpose |
|---|---|
| `On Playback Tick` | Fires every simulation step |
| `Isaac Read Simulation Time` | Provides timestamps |
| `ROS2 Publish Joint State` | Publishes `/joint_states` |
| `ROS2 Publish Transform Tree` | Publishes `/tf` |

**Connections:**

OnPlaybackTick (exec) → ROS2 Publish Joint State
OnPlaybackTick (exec) → ROS2 Publish Transform Tree
IsaacReadSimulationTime (timestamp) → both publishers

**Settings:**
- `ROS2 Publish Transform Tree → targetPrims`: `/World/Main_Robot_L_Yaskawa_GP110`
- `ROS2 Publish Transform Tree → static`: `false`
- `ROS2 Publish Joint State → targetPrim`: robot articulation root

> **Important:** Do **not** add any subscriber node — the bridge is publish-only.

---

## 7. Measured Link Parameters

Measured from `/tf` at mechanical zero:

| Parameter | Value (mm) | Description |
|---|---|---|
| **L1** | **540** | Base height (base → A2) |
| **L2** | **320** | Horizontal offset of A2 |
| **L3** | **870** | Distance A2 → A3 (vertical) |
| **L4** | **1020** | Distance A3 → flange |
| **LF** | *to be measured* | Flange length (`link_6` → `tool0`) |

**Verification vs. Yaskawa datasheet:**

| Parameter | Datasheet | Measured | Match |
|---|---|---|---|
| L1 | 540 mm | 540 mm | ✅ |
| L2 | 320 mm | 320 mm | ✅ |
| L3 | 870 mm | 870 mm | ✅ |
| L4 | 1020 mm | 1020 mm | ✅ |

---

## 8. TIA Portal Kinematics Mapping

Configure the technology object as `Articulated arm 3D with orientation (4 Axis)`:

| TIA Portal field | Value | Source |
|---|---|---|
| **Length L1** | **540 mm** | Base height (base → A2) |
| **Length L2** | **320 mm** | Horizontal offset of A2 |
| **Length L3** | **870 mm** | Distance A2 → A3 |
| **Length L4** | **1020 mm** | Distance A3 → flange |
| **Flange length LF** | *to be measured* | `link_6` → `tool0` |
| **Compensation factor** | **0.0** | No mechanical coupling |

> ⚠️ TIA Portal only supports **straight link segments** (L1–L4 + LF). The hand-crafted URDF (Section 4) must respect this constraint.

---

## 9. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `/tf` publishes all zeros | `ROS2 Publish Transform Tree` missing or `targetPrims` empty | Add the node and set `targetPrims = /World/Main_Robot_L_Yaskawa_GP110` |
| `/joint_states` shows values, but `/tf` does not change | TF publisher not connected to `OnPlaybackTick` | Connect the exec ports |
| `joint_4` missing from TF | Original `joint_4` deleted but not renamed | Rename `joint_6` → `joint_4` |
| Duplicate joint paths | URDF import created two roots | Delete `base_joint` |
| Robot falls under gravity | `base_link` not fixed | Add fixed joint to world |
| Robot not treated as articulation | `base_link` missing Articulation Root | Add `Physics → Articulation Root` |
| Joints oscillate | Drive damping too low | Set `Damping=1000`, `Stiffness=10000` |
| `tf2_echo` "frame does not exist" | Lookup runs before first TF message | Wait ~2 s; transient |
| **TCP mismatch (~235 mm in Z)** | **L-shaped `link_3` in GP110 URDF** | **Replace with hand-crafted URDF (Section 4)** |
| **Trajectory arc differs from TIA** | **L-shaped `link_3`** | **Same as above** |

---

## 10. Notes & Best Practices

- `ROS2 Publish Transform Tree` must have `static = false` and a valid `targetPrims` path.
- Do **not** create a subscriber node — the bridge is publish-only.
- Delete non-articulation root (`base_joint`) if duplicate paths appear.
- Always verify the robot is at **mechanical zero** before measuring link parameters.
- `joint_5` is intentionally hidden from the PLC and driven by Python to keep the tool downward.
- `link_5` and `link_6` often coincide because the wrist axes are mechanically collinear — this is expected.
- **The Yaskawa GP110 URDF is retained only as a reference. The production model is the hand-crafted URDF (Section 4).**

---

## Appendix A — Frame Hierarchy (after setup)

Main_Robot_L_Yaskawa_GP110 (root, fixed to world)
└── link_1 (A1)
└── link_2 (A2)
└── link_3 (A3)
└── link_4 (FixedJoint)
└── link_5 (A5, hidden, Python-driven)
└── link_6 (A4, formerly joint_6)
└── tool0 (TCP)


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
