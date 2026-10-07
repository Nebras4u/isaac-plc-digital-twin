# Commissioning Guide

## 1. Prerequisites Checklist

### Hardware
- [ ] Windows machine (192.168.100.18)
- [ ] Linux machine (192.168.100.x)
- [ ] Isaac Sim host (same Linux machine or separate)
- [ ] Network cable (or switch)

### Software — Windows
- [ ] TIA Portal V15.1+
- [ ] PLCSIM Advanced V6.0
- [ ] Python 3.x
- [ ] pythonnet
- [ ] PLCSIM Advanced API DLL

### Software — Linux
- [ ] Ubuntu 22.04
- [ ] ROS 2 Humble
- [ ] Python 3.10
- [ ] ruckig
- [ ] numpy

### Software — Isaac Sim
- [ ] Isaac Sim 4.x+
- [ ] ROS 2 Bridge extension enabled
- [ ] KR210 URDF at /World/Main_Robot_L_URDF

### Network
- [ ] Both machines on 192.168.100.x
- [ ] Windows firewall allows TCP port 5000
- [ ] Ping works: ping 192.168.100.18

## 2. Pre-Flight Checks

### 2.1 PLC Side
- [ ] TIA Portal project downloaded to PLCSIM instance "ros2_"
- [ ] OB30 loaded (heartbeat.scl)
- [ ] OB1 loaded (SCL_MAIN.txt)
- [ ] PLCSIM instance in RUN state

### 2.2 Windows Side

    cd windows
    python plc_server_win.py

Wait for: TCP server listening on 0.0.0.0:5000

### 2.3 Isaac Sim Side
- [ ] Isaac Sim open
- [ ] KR210 URDF loaded
- [ ] Simulation in Play mode
- [ ] Run isaac/physics_twin.py

Wait for: Entering main loop. Publishing /joint_states @ 60 Hz.

### 2.4 Linux Side — Bridge

    cd linux
    python3 plc_ros_bridge.py

Wait for: Connected to 192.168.100.18:5000

### 2.5 Linux Side — KRC Twin

    cd linux
    python3 krc_twin.py

Wait for: KRC Twin ready. DOF=5

### 2.6 Linux Side — Accuracy Check (optional)

    cd linux
    python3 check_accuracy.py

## 3. Startup Sequence (Visual)

    [1] Windows: plc_server_win.py       <-- FIRST
         |
    [2] Isaac: physics_twin.py           <-- SECOND
         |
    [3] Linux: plc_ros_bridge.py         <-- THIRD
         |
    [4] Linux: krc_twin.py               <-- FOURTH
         |
    [5] Linux: check_accuracy.py         <-- OPTIONAL

## 4. Post-Startup Verification

### 4.1 Topic Rates

    ros2 topic hz /joint_states           # 25-60 Hz
    ros2 topic hz /plc/cmd_ang            # 20 Hz
    ros2 topic hz /servo_setpoints        # 0 Hz until enabled

### 4.2 Heartbeat

    ros2 topic echo /plc/ros_alive --once # data: true

In PLCSIM Watch Table:
- [ ] Comm_State = 0
- [ ] ROS_Ack_Counter follows Heartbeat_Counter

## 5. Enabling Motion

In TIA Portal Watch Table:

    MainRobotLeft_CmdEnable := TRUE

Expected within 200 ms:
1. /plc/axis_enable = true
2. krc_twin logs "-> RUNNING"
3. /servo_setpoints starts at 60 Hz
4. Isaac robot moves
5. ActAng[1..4] updates in TIA

## 6. Shutdown Procedure

1. In TIA Portal: MainRobotLeft_CmdEnable := FALSE
2. Wait for robot to brake to stop
3. Ctrl+C in Linux terminals (reverse order)
4. Ctrl+C in Windows TCP server
5. Close Isaac Sim script

## 7. First-Time Verification Test

Command: CmdAng = [30, -30, 60, 0]

Expected results (after 5 s):

| Joint   | Error     |
|---------|-----------|
| joint_1 | < 0.01°   |
| joint_2 | < 0.03°   |
| joint_3 | < 0.01°   |
| joint_4 | < 0.01°   |

If errors are larger, check:
- GRAVITY_COMP in physics_twin.py
- JUMP_THRESHOLD_RAD in krc_twin.py
- Drive stiffness values

## 8. Common Startup Failures

| Symptom                  | Cause               | Fix                       |
|--------------------------|---------------------|---------------------------|
| TCP connection refused   | Server not running  | Start plc_server_win.py   |
| CmdEnable stays FALSE    | Heartbeat dead      | Check plc_ros_bridge      |
| /servo_setpoints 0 Hz    | Not in RUNNING      | Check enable + feedback   |
| Robot does not move      | Edit mode           | Switch to Play            |
| DOF mismatch warning     | Wrong URDF          | Verify DOF_ORDER          |
