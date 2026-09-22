# Isaac Digital Twin: S7-1500T ↔ ROS2 ↔ Isaac Sim

Digital twin of a Yaskawa GP110 robotic arm driven by a Siemens S7-1500T PLC, simulated in NVIDIA Isaac Sim via ROS2 and OPC UA.

Migration note (v0.2): The digital twin was originally prototyped on a KUKA KR210 L150 L URDF. It has now been ported to a Yaskawa GP110 articulation (5 joints exposed via /joint_states), and the TIA Portal technology object is configured as "Articulated arm 3D with orientation (4 Axis)". The $MAMES frame conversion developed for KUKA is no longer required — Yaskawa's mechanical zero maps directly to Isaac's home pose after the calibration in Section 4.

## Table of Contents

1. Current State (v0.2)
2. Architecture
3. Tech Stack
4. Isaac Sim Setup — Joint Renaming & Home Offset
5. Action Graph Configuration
6. Measured Link Parameters
7. Verifying the Setup
8. TIA Portal Kinematics Mapping
9. Troubleshooting
10. Notes & Best Practices
11. Screenshots
12. Roadmap
13. Technical Contributions
14. References
15. License
16. Appendix A — Frame Hierarchy
17. Appendix B — ROS 2 Topics
18. Appendix C — Default Joint Names

---

## 1. Current State (v0.2)

Real-time bidirectional synchronization between:
- Siemens S7-1500T PLC (OPC UA Server, TIA Portal, "Articulated arm 3D with orientation" TO)
- NVIDIA Isaac Sim 5.0 (Yaskawa GP110, URDF, 5-joint articulation)
- ROS2 Humble (/joint_states, /tf, /clock)

Includes Isaac Home offset calibration for accurate PLC <-> simulation joint comparison, and joint renaming (joint_6 -> joint_4) so the articulation exposes exactly the 5 joints expected by the TIA Portal kinematics block.

Flow:

    [S7-1500T PLCSIM Advanced + PLCopen Motion + TO 4-Axis]
              |
              | OPC UA :4840 / ns=3 / Read Arrays & Flags
              v
    [bridge.py  (asyncua + rclpy, Home Offset + Joint Remap)]
              ^
              | ROS2 /DDS  /joint_states  (5 joints)
              |
    [Isaac Sim 5.0 — PhysX 5 — Yaskawa GP110 URDF — 5-joint articulation]

    Compare -> delta ~= 0 deg -> Validated

---

## 2. Architecture

    [PLC S7-1500T] <-- OPC UA --> [Python Bridge] <-- ROS2 --> [Isaac Sim]
         (4-axis)                        |
                                [Home Offset Calib.]
                                [joint_6 -> joint_4]

---

## 3. Tech Stack

- PLC: Siemens S7-1500T (TIA Portal, OPC UA Server, "Articulated arm 3D with orientation — 4 Axis")
- Robot: Yaskawa GP110 (was KUKA KR210 L150 L in v0.1)
- Middleware: ROS2 Humble, rclpy, asyncua
- Simulation: NVIDIA Isaac Sim 5.0 (PhysX 5, RTX)
- Protocols: OPC UA (ns=3), ROS2 topics

---

## 4. Isaac Sim Setup — Joint Renaming & Home Offset

### 4.1 Rename Joint 6 to Joint 4

The articulation is renumbered so the final joint list matches the TIA Portal 4-axis kinematics:

| New name | Role | Driven by |
|---|---|---|
| joint_1 | A1 — base rotation | PLC |
| joint_2 | A2 — shoulder | PLC |
| joint_3 | A3 — elbow | PLC |
| joint_4 | A4 — tool roll (was joint_6) | PLC |
| joint_5 | A5 — hidden tool axis (Python, keeps tool pointing down) | Python |

Note: This mapping matches the 4-axis kinematics expected by the Siemens TIA Portal "Articulated arm 3D with orientation" block.

### 4.2 Home Offset Calibration

The URDF bias is normalized so that Isaac's mechanical zero matches Yaskawa's mechanical zero. This eliminates the need for $MAMES frame conversion that was used in v0.1 with the KUKA KR210.

---

## 5. Action Graph Configuration

Create an Action Graph with the following nodes:

| Node | Purpose |
|---|---|
| On Playback Tick | Fires every simulation step |
| Isaac Read Simulation Time | Provides the timestamp for messages |
| ROS2 Publish Joint State | Publishes /joint_states |
| ROS2 Publish Transform Tree | Publishes /tf |

Connections:

    OnPlaybackTick (exec) -> ROS2 Publish Joint State
    OnPlaybackTick (exec) -> ROS2 Publish Transform Tree
    IsaacReadSimulationTime (timestamp) -> both publishers

Settings:
- ROS2 Publish Transform Tree -> targetPrims: /World/Main_Robot_L_Yaskawa_GP110
- ROS2 Publish Transform Tree -> static: false
- ROS2 Publish Joint State -> targetPrim: robot articulation root

Important: Do NOT add any subscriber node. The bridge is publish-only.

Isaac Sim — ROS2 Action Graph:
https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/isaac_sim/Yaskawa_GP110.jpg

---

## 6. Measured Link Parameters

All values measured from /tf with every joint at 0 rad (mechanical zero):

| Parameter | Value (mm) | Description |
|---|---|---|
| L1 | 540 | Base height (base -> A2) |
| L2 | 320 | Horizontal offset of A2 |
| L3 | 870 | Distance A2 -> A3 (vertical) |
| L4 | 1020 | Distance A3 -> flange |
| LF | to be measured | Flange length (link_6 -> tool0) |

These values are entered directly into the TIA Portal kinematics configuration.

Reference (Yaskawa GP110 datasheet):

| Parameter | Datasheet | Measured | Match |
|---|---|---|---|
| L1 | 540 mm | 540 mm | OK |
| L2 | 320 mm | 320 mm | OK |
| L3 | 870 mm | 870 mm | OK |
| L4 | 1020 mm | 1020 mm | OK |

---

## 7. Verifying the Setup

With Isaac Sim playing, run:

    ros2 topic echo /joint_states --once
    ros2 topic echo /tf --once | grep child_frame_id

Expected result:
- /joint_states publishes 5 joints with real values.
- /tf publishes 6 frames (link_1 ... link_6) with non-zero translations and valid quaternions.
- At mechanical zero, all w ~= 1.0 and translations match the table in Section 6.

Example /tf output at mechanical zero:

    frame_id: Main_Robot_L_Yaskawa_GP110

    child_frame_id: link_1 -> translation: { x: 0.000, y: 0.000, z: 0.540 }
    child_frame_id: link_2 -> translation: { x: 0.320, y: 0.000, z: 0.540 }
    child_frame_id: link_3 -> translation: { x: 0.321, y: 0.000, z: 1.410 }
    child_frame_id: link_4 -> translation: { x: 0.321, y: 0.000, z: 1.645 }
    child_frame_id: link_5 -> translation: { x: 1.341, y: 0.000, z: 1.643 }
    child_frame_id: link_6 -> translation: { x: 1.341, y: 0.000, z: 1.643 }

Note: link_5 and link_6 coincide because the wrist axes are mechanically collinear — this is expected.

---

## 8. TIA Portal Kinematics Mapping

In TIA Portal, configure the technology object as "Articulated arm 3D with orientation (4 Axis)". Enter the measured values into the corresponding fields:

| TIA Portal field | Value | Source |
|---|---|---|
| Length L1 | 540 mm | Base height (base -> A2) |
| Length L2 | 320 mm | Horizontal offset of A2 |
| Length L3 | 870 mm | Distance A2 -> A3 |
| Length L4 | 1020 mm | Distance A3 -> flange |
| Flange length LF | to be measured | link_6 -> tool0 |
| Compensation factor | 0.0 | No mechanical coupling |

Notes:
- L1 and L2 define the position of axis A2 relative to the kinematic zero point (KZP).
- L3 is the distance between axes A2 and A3.
- L4 is the distance from A3 to the mandatory coupling point.
- LF is the flange length from the coupling point to the flange coordinate system (FCS).

---

## 9. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| /tf publishes all zeros | ROS2 Publish Transform Tree missing or targetPrims empty | Add node; set targetPrims = /World/Main_Robot_L_Yaskawa_GP110 |
| /joint_states updates but /tf frozen | TF publisher not wired to OnPlaybackTick | Connect exec ports |
| joint_4 missing from TF | joint_6 not renamed | Rename joint_6 -> joint_4 |
| Duplicate joint paths (link_1 and base_joint) | URDF import created two roots | Delete base_joint |
| Robot falls under gravity | base_link not fixed | Add fixed joint to world |
| Not treated as articulation | Missing Articulation Root | Add Physics -> Articulation Root |
| Joints jitter | Drive damping too low | Set Damping=1000, Stiffness=10000 |
| tf2_echo "frame does not exist" | Lookup before first TF message | Wait ~2 s; transient |

---

## 10. Notes & Best Practices

- ROS2 Publish Transform Tree must have static = false and a valid targetPrims path; otherwise it publishes empty (zero) transforms.
- Do not create a subscriber node — the bridge is publish-only.
- If duplicate frame paths appear after import, delete the non-articulation root (base_joint).
- Always verify the robot is at mechanical zero before measuring link parameters.
- The 5th axis (joint_5) is intentionally hidden from the PLC and driven by Python to keep the tool pointing downward.
- link_5 and link_6 often coincide because the wrist axes are mechanically collinear — expected.

---

## 11. Screenshots

Live PLC <-> Isaac Sim Joint Comparison:
https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/docs/general_view.png

TIA Portal — PLC Data Block:
https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/docs/tia_portal_db.png

Isaac Sim — ROS2 Action Graph:
https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/isaac_sim/Yaskawa_GP110.jpg

---

## 12. Roadmap

- [x] v0.1 — KUKA KR210 L150 L · PLC <-> OPC UA <-> ROS2 <-> Isaac Sim joint state sync
- [x] v0.2 — Port to Yaskawa GP110, remap joint_6 -> joint_4, 5-joint articulation, "Articulated arm 3D (4 Axis)" TO
- [ ] v0.3 — IK node with What-If feasibility checks
- [ ] v0.4 — Pneumatic suction gripper + glass sheet handling
- [ ] v0.5 — Force loop + grasp stability monitoring
- [ ] v1.0 — Domain randomization + full digital twin validation

---

## 13. Technical Contributions

1. 5-joint remap for 4-axis PLC kinematics: joint_6 renamed to joint_4, joint_5 Python-driven, exposing exactly the joint set expected by TIA Portal's "Articulated arm 3D with orientation" block.
2. Isaac Home offset calibration: normalizes URDF bias so Isaac's mechanical zero matches Yaskawa's mechanical zero.
3. Real-time PLC <-> simulation validation: compares joint states with per-axis deviation output.
4. Publish-only ROS2 Action Graph: On Playback Tick -> ROS2 Publish Joint State + ROS2 Publish Transform Tree (no subscriber, static=false, targetPrims=/World/Main_Robot_L_Yaskawa_GP110).

---

## 14. References

Inspired by: Isaac Sim Integrated Digital Twin For Feasibility Checks In Skill-based Engineering (RAAD 2025) — DOI: 10.1007/978-3-032-02106-9_46
https://doi.org/10.1007/978-3-032-02106-9_46

Datasheet: Yaskawa GP110 (link lengths L1–L4 confirmed).

---

## 15. License

MIT

---

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

---

## Appendix B — ROS 2 Topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| /joint_states | sensor_msgs/JointState | Isaac -> ROS 2 | Joint angles |
| /tf | tf2_msgs/TFMessage | Isaac -> ROS 2 | Dynamic transforms |
| /tf_static | tf2_msgs/TFMessage | Isaac -> ROS 2 | Static transforms |
| /clock | rosgraph_msgs/Clock | Isaac -> ROS 2 | Simulation time |

---

## Appendix C — Default Joint Names

| Index | Name | Role |
|---|---|---|
| 1 | joint_1 | Base rotation (A1) |
| 2 | joint_2 | Shoulder (A2) |
| 3 | joint_3 | Elbow (A3) |
| 4 | joint_4 | Tool roll (A4, was joint_6) |
| 5 | joint_5 | Hidden tool axis (Python-driven) |
