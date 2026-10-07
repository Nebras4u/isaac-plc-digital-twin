# Recalculated Kinematic Parameters for TIA Portal

**Configuration:** joint_a4 and joint_a6 locked, joint_a5 active

With A4 and A6 locked, the physical robot behaves as a standard **4-axis articulated robot** with an active pitch wrist — often styled as *Articulated 3D with 1 orientation axis* or *Palletizer* kinematic profile in Siemens Technology Objects.

In this structure, the kinematics engine calculates lengths up to the center of the active wrist pivot (joint_a5), then applies the distance from joint_a5 to `tool0` as a fixed rigid tool/flange offset.

All values below are recalculated directly from the raw Isaac Sim data table.

---

## 1. Length of Link 2 — Main Boom (Shoulder → Elbow)

Net vertical height between the shoulder pivot axis (joint_a2) and the elbow pivot axis (joint_a3).

| Item | Value |
|---|---|
| Calculation | `Z(A3) - Z(A2) = 750.190 - 330.990` |
| **TIA Portal Entry** | **419.2 mm** |

---

## 2. Length of Link 3 — Forearm (Elbow → Wrist Pivot Center)

Because joint_a4 is locked, the section from the elbow (joint_a3) out to the active wrist axis (joint_a5) acts as a single structural link. Siemens kinematic blocks break this arm piece into two projections:

### Horizontal Extension (X component)

| Item | Value |
|---|---|
| Calculation | `X(A5) - X(A3) = 1308.128 - 350.150` |
| **TIA Portal Entry** | **958.0 mm** |

### Vertical Drop / Rise (Z component)

| Item | Value |
|---|---|
| Calculation | `Z(A5) - Z(A3) = 1944.901 - 750.190` |
| **TIA Portal Entry** | **1194.7 mm** |

---

## 3. Flange / Tool Output Offset (joint_a5 → tool0)

Because joint_a6 is locked, the distance from the active moving wrist center (joint_a5) out to the final structural attachment interface flange (`tool0`) behaves as a completely rigid, unmoving extension vector.

In the Siemens TIA Portal kinematic profile for an articulated arm (typically *Articulated Arm 3D with orientation* / *Palletizer* profile), the final horizontal rigid distance from the wrist axis (joint_a5) to the tool flange coordinate system anchor is designated as **LF** (Link Flange / Flange Length). There are minor variations depending on the exact TO version, but **LF** is the standard Siemens terminology for this horizontal component.

### Flange Horizontal Offset (X direction)

| Item | Value |
|---|---|
| Calculation | `X(tool0) - X(A5) = 2080.128 - 1308.128` |
| **TIA Portal Entry** | **772.0 mm** |
| **TIA Portal Tag** | **LF** (Flange / Tool Offset X) |
| Verification | Exact rigid horizontal bridge length between joint_a5 (1308.128 mm) and the tool0 flange (2080.128 mm). |

### Flange Vertical Offset (Z direction)

| Item | Value |
|---|---|
| Calculation | `Z(tool0) - Z(A5) = 1944.556 - 1944.901 = -0.345` |
| Note | Effectively zero, within rounding boundaries |
| **TIA Portal Entry** | **0.0 mm** |

---

## Complete 4-Axis Geometry Parameters Checklist

Standard link tokens associated with this specific profile configuration:

| TIA Portal Layout Tag | Calculated Value | Definition / Calculation |
|---|---|---|
| **LF** (Flange Offset X) | **772.0 mm** | Horizontal offset from joint_a5 to `tool0` |
| **L2** (Shoulder → Elbow) | **419.2 mm** | Net vertical height between joint_a2 and joint_a3 |
| **L3** (Elbow → Wrist X) | **958.0 mm** | Horizontal projection between joint_a3 and joint_a5 |
| **L4** (Elbow → Wrist Z) | **1194.7 mm** | Vertical drop/rise between joint_a3 and joint_a5 |

---

## Verification

Inputting these exact values into your TIA Portal geometry parameters ensures the PLC's internal forward-kinematics engine accurately represents the physical model in Isaac Sim.

When you open your **Technology Object Configuration Editor** under the **Geometry / Link Parameters** panel, do these four specific data inputs match the parameters diagram requested by your Siemens 4-axis profile layout selection?

LF = 772.0
L2 = 419.2
L3 = 958.0
L4 = 1194.7


---

## Source Data (Isaac Sim Raw Reference)

| Point | X (mm) | Z (mm) |
|---|---|---|
| joint_a2 | — | 330.990 |
| joint_a3 | 350.150 | 750.190 |
| joint_a5 | 1308.128 | 1944.901 |
| tool0 | 2080.128 | 1944.556 |
