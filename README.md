# Isaac Digital Twin: S7-1500T ↔ ROS2 ↔ Isaac Sim

Digital twin of a KUKA KR210 L150 L robotic arm driven by a Siemens 
S7-1500T PLC, simulated in NVIDIA Isaac Sim via ROS2 and OPC UA.

## 🎯 Current State (v0.1)

Real-time bidirectional synchronization between:
- Siemens S7-1500T PLC (OPC UA Server, TIA Portal)
- NVIDIA Isaac Sim 5.0 (KUKA KR210 L150 L, URDF)
- ROS2 Humble (joint_states topic)

Includes KUKA $MAMES frame conversion and Isaac Home offset 
calibration for accurate PLC ↔ simulation joint comparison.

## 🏗️ Architecture

```
[PLC S7-1500T] ← OPC UA → [Python Bridge] ← ROS2 → [Isaac Sim]
                                ↓
                      [$MAMES Conversion]
                      [Isaac Home Offset]
```

## ⚙️ Tech Stack
- **PLC**: Siemens S7-1500T (TIA Portal, OPC UA Server)
- **Robot**: KUKA KR210 L150 L
- **Middleware**: ROS2 Humble, rclpy, asyncua
- **Simulation**: NVIDIA Isaac Sim 5.0 (PhysX 5, RTX)
- **Protocols**: OPC UA (ns=3), ROS2 topics

## 📸 Screenshots

### Live PLC ↔ Isaac Sim Joint Comparison
![Live Comparison](https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/docs/general_view.png)

### TIA Portal — PLC Data Block
![TIA Portal DB](https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/docs/tia_portal_db.png)

### Isaac Sim — ROS2 Action Graph
![Isaac Sim Action Graph](https://raw.githubusercontent.com/Nebras4u/isaac-plc-digital-twin/main/docs/action_graph.png)

## 🚦 Roadmap
- [x] **v0.1** — PLC ↔ OPC UA ↔ ROS2 ↔ Isaac Sim joint state sync
- [ ] **v0.2** — IK node with What-If feasibility checks
- [ ] **v0.3** — Pneumatic suction gripper + glass sheet handling
- [ ] **v0.4** — Force loop + grasp stability monitoring
- [ ] **v1.0** — Domain randomization + full digital twin validation

## 🔬 Technical Contributions (v0.1)
1. **$MAMES frame conversion**: Maps KUKA mathematical zero to 
   mechanical zero using $MACHINE.DAT values.
2. **Isaac Home offset calibration**: Normalizes URDF bias so 
   Isaac's mechanical zero matches KUKA's.
3. **Real-time PLC ↔ simulation validation**: Compares joint states 
   with per-axis deviation output.

## 📖 References
Inspired by: *Isaac Sim Integrated Digital Twin For Feasibility Checks 
In Skill-based Engineering* (RAAD 2025) — [DOI link]

## 📜 License
MIT
