# Isaac Lab 510 — Startup Guide

**Docker image:** `hhanoo/sim:isaac-lab-5.1.0-jazzy`  
**Container name:** `isaac-lab-510-persistent`  
**Base:** Isaac Lab 5.1.0 + ROS 2 Jazzy  
**Robot:** KUKA KR210 L150 L  
**PLC:** Siemens S7-1500T (PLCSIM Advanced v6.0)

---

## Transport Selection

The bridge supports three transports. Select one in `plc_ros_bridge.yaml`:

```yaml
plc:
  driver: tcp_json        # tcp_json | opcua | plc_api
```

| Driver | Runs Where | Extra File Needed on Windows |
| :--- | :--- | :--- |
| `tcp_json` | Bridge on Linux | ✅ `plc_server_win.py` |
| `opcua` | Bridge on Linux | ❌ (bridge talks to OPC UA server directly) |
| `plc_api` | Bridge on Windows | ❌ (bridge calls PLCSIM API directly) |

---

## Terminal 0 — Windows (only for `tcp_json`)

**This terminal is required ONLY when the bridge uses the `tcp_json` driver.**

If `plc_ros_bridge.yaml` has:

```yaml
plc:
  driver: tcp_json
```

then you must start `plc_server_win.py` on the Windows machine (the one running PLCSIM Advanced v6.0) before starting the bridge on Linux.

### On Windows:

```powershell
cd C:\path\to\project
python plc_server_win.py
```

**Expected output:**

```
Connected to PLCSIM: ros2_, State: Run
Tag list updated. Total tags: 40
TCP server listening on 0.0.0.0:5000
Waiting for ROS 2 bridge to connect...
```

Leave this terminal running for the entire session.

### If using `opcua` or `plc_api`:

Skip this terminal entirely. No server needed on Windows.

---

## Terminal 1 — Isaac Sim (Isaac 5.1)

```bash
xhost +local:docker
docker exec -it isaac-lab-510-persistent bash
start_isaac
```

Then inside Isaac Sim:

- `Window` → `Script Editor`
- Load the scene
- ▶ Play
- Run `physics_twin.py`

---

## Terminal 2 — plc_ros_bridge.py

```bash
docker exec -it isaac-lab-510-persistent bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=45
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /workspace
python3 plc_ros_bridge.py
```

**Note:** The `driver` field in `plc_ros_bridge.yaml` determines the transport. For `tcp_json`, make sure `plc_server_win.py` is already running on Windows (Terminal 0).

---

## Terminal 3 — trajectory_twin.py

```bash
docker exec -it isaac-lab-510-persistent bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=45
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /workspace
python3 trajectory_twin.py
```

---

## Terminal 4 — Monitoring

```bash
export ROS_DOMAIN_ID=45
ros2 topic list
ros2 topic echo /joint_states
```
