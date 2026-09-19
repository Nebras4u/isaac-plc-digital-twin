# Isaac Digital Twin: S7-1500T ↔ ROS2 ↔ Isaac Sim
Digital Twin of S7-1500T PLC-driven robotic arm in NVIDIA Isaac Sim via ROS2/OPC UA

A research-oriented digital twin for industrial robotic manipulation, 
connecting a Siemens S7-1500T PLC to NVIDIA Isaac Sim via ROS2 and OPC UA.

## 🎯 Objective
Demonstrate a full-stack digital twin for robotic glass sheet handling 
using pneumatic suction grippers, with What-If feasibility checks before 
physical execution.

## 🏗️ Architecture
[PLC S7-1500T] ← OPC UA → [ROS2 Bridge] ← → [NVIDIA Isaac Sim]
                                    ↓
                            [IK Node + What-If Checks]

## ⚙️ Tech Stack
- **PLC**: Siemens S7-1500T (TIA Portal, OPC UA Server)
- **Middleware**: ROS2 Humble, Python `asyncua`
- **Simulation**: NVIDIA Isaac Sim (PhysX 5, RTX)
- **Control**: Inverse Kinematics, MoveIt2 (planned)
- **Protocols**: OPC UA, ROS2 topics/services

## 🚦 Roadmap
- [x] **v0.1** — Minimal viable integration: PLC → OPC UA → ROS2 → Isaac Sim
- [ ] **v0.2** — IK node with What-If feasibility checks
- [ ] **v0.3** — Force loop + grasp stability monitoring
- [ ] **v1.0** — Domain randomization + full digital twin validation

## 🎥 Demo
[Link to video]

## 📖 References
Inspired by: *Isaac Sim Integrated Digital Twin For Feasibility Checks 
In Skill-based Engineering* (RAAD 2025) — [DOI link]

## 📜 License
MIT
