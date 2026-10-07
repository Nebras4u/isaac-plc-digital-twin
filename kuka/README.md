# KUKA KR210 Digital Twin — Current State Documentation

This document describes the **current implementation** of the KUKA KR210
Digital Twin. It contains no roadmap, no future plans, and no speculative
sections. Everything here reflects code and data that exist today and
have been verified on the target hardware.

## 1. Overview

The project is a five-layer Digital Twin of a KUKA KR210 robot. A real
Siemens S7-1500 PLC program runs inside PLCSIM Advanced on Windows and
drives the system through a heartbeat protocol. A TCP server exposes PLC
tags over JSON. A ROS 2 bridge on Linux polls the PLC, publishes command
topics, and writes feedback back. A KRC Digital Twin uses Ruckig to
generate jerk-limited joint trajectories at 60 Hz. An Isaac Sim physics
twin applies setpoints to the KR210 articulation and publishes joint
feedback. Every layer is independent and can be tested in isolation.

## 2. Architecture

Five layers across three machines:

- Layer 4 — PLC: Siemens S7-1500 in PLCSIM Advanced (Windows)
- Layer 3 — TCP Server: plc_server_win.py (Windows)
- Layer 2 — ROS 2 Bridge: plc_ros_bridge.py (Linux)
- Layer 1b — KRC Digital Twin: krc_twin.py (Linux)
- Layer 1a — Physics Twin: physics_twin.py (Isaac Sim)

Data moves as follows:

- PLC -> TCP server via pythonnet (in-process)
- TCP server -> ROS 2 bridge via TCP/JSON on port 5000
- ROS 2 bridge -> KRC twin via ROS 2 topic /plc/cmd_ang
- KRC twin -> Physics twin via ROS 2 topic /servo_setpoints
- Physics twin -> KRC twin via ROS 2 topic /joint_states
- KRC twin -> PLC (through bridge) via written tags ActAng[1..4]

## 3. File Inventory

### Windows Side

| File | Purpose |
|------|---------|
| plc_server_win.py | TCP/JSON server. Calls PLCSIM Advanced API through pythonnet. Exposes ~30 tags. |

### Linux Side

| File | Purpose |
|------|---------|
| plc_ros_bridge.py | Bidirectional bridge between PLCSIM (TCP) and ROS 2. Publishes /plc/* topics, writes feedback and heartbeat ack. |
| krc_twin.py | KRC Digital Twin. Ruckig online trajectory generation. State machine IDLE/RUNNING/STOPPING. |
| check_accuracy.py | Subscribes to /servo_setpoints and /joint_states. Prints a comparison table every second. |

### Isaac Sim Side

| File | Purpose |
|------|---------|
| physics_twin.py | Articulation driver. Applies setpoints to KR210, publishes /joint_states at 60 Hz. |

### PLC Side (TIA Portal)

| File | Purpose |
|------|---------|
| heartbeat.scl | OB30 heartbeat protocol. Runs every 20 ms. |
| SCL_MAIN.txt | OB1 motion logic. Uses MC_PickUp, MC_Stkng_*, MC_Go2HomePos with TO_Kinematics. |

### Tools

| File | Purpose |
|------|---------|
| srv.py | Clock drift measurement — responder side. UDP port 9999. |
| cli.py | Clock drift measurement — client side. Same protocol. |

### Data

| File | Purpose |
|------|---------|
| calib_*_summary.json | Clock drift results for one session. |
| calib_*_raw.csv | Raw timestamp data from one session. |

## Documentation

| Document | Description |
|----------|-------------|
| `docs/PROJECT_DOCUMENTATION.txt` | Complete project documentation (18 sections) |
| `docs/architecture.md` | System architecture (planned) |
| `docs/commissioning.md` | Startup procedure (planned) |

## 4. Data Flow (Nominal Operation)

The following sequence repeats continuously while the system is running.
Step 1: PLC increments Heartbeat_Counter every 20 ms (OB30)
Step 2: plc_ros_bridge polls via TCP every 50 ms (20 Hz)
Step 3: Bridge reads CmdAng[1..4], CmdEnable, Vlcty, Heartbeat_Counter
Step 4: Bridge writes ROS_Ack_Counter = Heartbeat_Counter
Step 5: Bridge publishes /plc/cmd_ang, /plc/axis_enable
Step 6: krc_twin reads /plc/*, enters RUNNING state
Step 7: krc_twin resets Ruckig planner from actual positions
Step 8: krc_twin builds target vector from CmdAng (deg -> rad)
Step 9: krc_twin calls Ruckig, gets next setpoint
Step 10: krc_twin publishes /servo_setpoints @ 60 Hz
Step 11: physics_twin applies setpoints to Isaac articulation
Step 12: Isaac physics computes new joint positions
Step 13: physics_twin publishes /joint_states @ 60 Hz
Step 14: krc_twin reads feedback, syncs Ruckig
Step 15: plc_ros_bridge reads feedback, converts to degrees
Step 16: Bridge writes ActAng[1..4] to PLC
Step 17: PLC sees updated ActAng in watch table


Total loop latency is approximately 150-300 ms. The dominant factor is
PLC polling at 20 Hz plus the TCP round trip.

## 5. Heartbeat Protocol

The heartbeat runs in OB30 every 20 ms on the PLC side, and in
plc_ros_bridge.py every 50 ms on the Linux side. It is the primary
mechanism for detecting communication faults.

### 5.1 PLC Side (OB30)

Every 20 ms:

1. Heartbeat_Counter is incremented (with rollover at 2^31).
2. Heartbeat_Timestamp is updated via RD_LOC_T.
3. HB_Age is computed as Heartbeat_Counter - ROS_Ack_Counter.
4. The communication state machine runs.

### 5.2 Comm_State Values

| Value | Name   | Meaning                                      |
|-------|--------|----------------------------------------------|
| 0     | OK     | ROS ack is within HB_Timeout_Mult            |
| 1     | Late   | HB_Age > HB_Timeout_Mult                     |
| 2     | Dead   | HB_Age > 2 * HB_Timeout_Mult                 |
| 3     | BadAck | Ack out of range or counter rollover         |

### 5.3 State Transitions

- State 0 (OK) -> State 1 (Late): HB_Age > HB_Timeout_Mult
- State 1 (Late) -> State 0 (OK): HB_Age <= HB_Timeout_Mult
- State 1 (Late) -> State 2 (Dead): HB_Age > 2 * HB_Timeout_Mult
- State 2 (Dead) -> State 0 (OK): HB_Age <= HB_Timeout_Mult
- Any state -> State 3 (BadAck): HB_Age < 0

### 5.4 Forced Actions

- State 2 (Dead): CmdEnable is forced FALSE, ROS_Alive is FALSE.
- State 3 (BadAck): CmdEnable is forced FALSE.

Every cycle:

ROS_Alive := (Comm_State = 0)


### 5.5 ROS Side (plc_ros_bridge.py)

Every 50 ms:

1. Read Heartbeat_Counter from PLC.
2. If Heartbeat_Counter changed since last poll, write
   ROS_Ack_Counter = Heartbeat_Counter back to the PLC.
3. Read Comm_State and ROS_Alive.
4. Publish /plc/ros_alive as a Bool.

### 5.6 Observed Nominal Values

During successful runs:

| Tag                | Value  |
|--------------------|--------|
| Heartbeat_Counter  | 37257  |
| ROS_Ack_Counter    | 37255  |
| Delta              | 2      |

A delta of 2 means the ROS ack is written faster than the PLC increments
the counter. ROS is never the bottleneck.

### 5.7 Failure Modes Handled

| Fault                        | Detection           | Reaction               |
|------------------------------|---------------------|------------------------|
| ROS process stops            | HB_Age > 50         | Comm_State = 1 (Late)  |
| ROS dead for > 2 s           | HB_Age > 100        | Comm_State = 2 (Dead), CmdEnable = FALSE |
| Ack out of range / rollover  | HB_Age < 0          | Comm_State = 3 (BadAck), CmdEnable = FALSE |
| TCP connection lost          | socket error        | bridge reconnects      |
| /joint_states stale > 0.5 s  | no callback         | bridge stops writing, krc_twin brakes |
| Emergency stop               | EStop flag = TRUE   | krc_twin brakes        |
| Ruckig error (5 consecutive) | error counter       | krc_twin brakes        |

## 6. ROS 2 Topics

### 6.1 Published by plc_ros_bridge.py

| Topic                | Type                    | Rate    | Content                          |
|----------------------|-------------------------|---------|----------------------------------|
| /plc/cmd_ang         | sensor_msgs/JointState  | 20 Hz   | Target joint angles in degrees   |
| /plc/axis_enable     | std_msgs/Bool           | 20 Hz   | CmdEnable from PLC               |
| /plc/emergency_stop  | std_msgs/Bool           | 20 Hz   | Emergency stop flag              |
| /plc/velocity        | std_msgs/Float64        | 20 Hz   | Velocity percentage (0..500)     |
| /plc/ros_alive       | std_msgs/Bool           | 20 Hz   | Comm_State == 0                  |
| /plc/cmd_ack         | std_msgs/Bool           | 20 Hz   | Acknowledgement of command       |

### 6.2 Subscribed by plc_ros_bridge.py

| Topic          | Type                    | Purpose                          |
|----------------|-------------------------|----------------------------------|
| /joint_states  | sensor_msgs/JointState  | Feedback from Isaac (radians)    |

### 6.3 Published by krc_twin.py

| Topic             | Type                    | Rate   | Content                        |
|-------------------|-------------------------|--------|--------------------------------|
| /servo_setpoints  | sensor_msgs/JointState  | 60 Hz  | Joint setpoints in radians     |

### 6.4 Subscribed by krc_twin.py

| Topic                 | Type                    | Purpose                       |
|-----------------------|-------------------------|-------------------------------|
| /joint_states         | sensor_msgs/JointState  | Feedback from Isaac           |
| /plc/cmd_ang          | sensor_msgs/JointState  | Target from PLC (degrees)     |
| /plc/axis_enable      | std_msgs/Bool           | Enable from PLC               |
| /plc/emergency_stop   | std_msgs/Bool           | Emergency stop from PLC       |

### 6.5 Published by physics_twin.py

| Topic          | Type                    | Rate   | Content                       |
|----------------|-------------------------|--------|-------------------------------|
| /joint_states  | sensor_msgs/JointState  | 60 Hz  | Actual joint positions        |

### 6.6 Subscribed by physics_twin.py

| Topic             | Type                    | Purpose                       |
|-------------------|-------------------------|-------------------------------|
| /servo_setpoints  | sensor_msgs/JointState  | Setpoints from KRC twin       |

## 7. PLC DB Reference

Database name: MAIN_KUKA_KR210_L150_L

### 7.1 Robot Commands (PLC -> ROS)

| Tag                                | Type       | Description                |
|------------------------------------|------------|----------------------------|
| MainRobotLeft_CmdAng[1..4]         | Array LReal| Target joint angles (deg)  |
| MainRobotLeft_CmdEnable            | Bool       | Motion enable              |
| MainRobotLeft_Vlcty                | LReal      | Velocity percentage        |
| MainRobotLeft_Emergency_Stop_Active| Bool       | Emergency stop flag        |

### 7.2 Robot Status (Read Only)

| Tag                          | Type | Description              |
|------------------------------|------|--------------------------|
| MainRobotLeft_Motion_Active  | Bool | Motion in progress       |
| MainRobotLeft_Group_Fault    | Bool | Kinematics group fault   |
| MainRobotLeft_Axis1_Fault    | Bool | Axis 1 fault             |
| MainRobotLeft_Axis2_Fault    | Bool | Axis 2 fault             |
| MainRobotLeft_Axis3_Fault    | Bool | Axis 3 fault             |
| MainRobotLeft_Axis4_Fault    | Bool | Axis 4 fault             |

### 7.3 Robot Feedback (ROS -> PLC)

| Tag                       | Type        | Description             |
|---------------------------|-------------|-------------------------|
| MainRobotLeft_ActAng[1..4]| Array LReal | Actual joint angles     |
| MainRobotLeft_CmdAck      | Bool        | Command acknowledged    |

### 7.4 Heartbeat Tags

| Tag                  | Type | Description                          |
|----------------------|------|--------------------------------------|
| Heartbeat_Counter    | DInt | Incremented every 20 ms              |
| Heartbeat_Timestamp  | DTL  | Diagnostic timestamp                 |
| ROS_Ack_Counter      | DInt | Written by ROS                       |
| ROS_Ack_Timestamp    | DTL  | Diagnostic timestamp                 |
| ROS_Alive            | Bool | TRUE only if Comm_State == 0         |
| Comm_State           | Int  | 0=OK, 1=Late, 2=Dead, 3=BadAck       |
| Comm_LostCount       | DInt | Number of lost heartbeats            |
| HB_Timeout_Mult      | Int  | Cycles before Late (default 50)      |

## 8. KRC Twin State Machine

### 8.1 States

| State    | Meaning                                       |
|----------|-----------------------------------------------|
| IDLE     | No motion. Waiting for enable and valid cmd.  |
| RUNNING  | Actively generating trajectories with Ruckig. |
| STOPPING | Braking with jerk-limited velocity profile.   |

### 8.2 Transitions

| From     | To       | Condition                                          |
|----------|----------|----------------------------------------------------|
| IDLE     | RUNNING  | enable AND fresh_feedback AND valid_cmd            |
| RUNNING  | STOPPING | any fault (disable, estop, stale, bad cmd, Ruckig) |
| STOPPING | IDLE     | Ruckig reports Finished on braking trajectory      |

A return to RUNNING from STOPPING requires going through IDLE implicitly
via the next enable + cmd cycle.

### 8.3 Jump Protection

Threshold: JUMP_THRESHOLD_RAD = 0.30 (~17 degrees)

If |measured - last| > threshold in any joint:

1. Reset Ruckig's internal velocity and acceleration to zero.
2. Continue from the new position.

This prevents violent corrective motion when Isaac resets or when the ROS
feedback drops temporarily.

### 8.4 Braking Behavior

On any fault, krc_twin switches Ruckig to Velocity interface with
target_velocity = 0. This produces a jerk-limited deceleration profile.
This matches IEC 60204-1 Stop Category 1 (controlled stop with power
retained).

### 8.5 Feedback-Driven Control

Every servo cycle (60 Hz):

1. Read latest /joint_states (5 joints, radians).
2. If fresh (within 0.5 s):
   - planner.sync_state(actual)
   - planner.set_target(target)
   - planner.step()
   - publish /servo_setpoints
3. Else: enter STOPPING (braking).

Ruckig manages internal velocity and acceleration. Only the position is
overwritten each cycle.

## 9. Ruckig Parameters

| Parameter   | Value      | Unit    | Notes                        |
|-------------|------------|---------|------------------------------|
| CONTROL_HZ  | 60         | Hz      | Servo loop rate              |
| MAX_VEL     | 1.5        | rad/s   | ~86 deg/s                    |
| MAX_ACC     | 5.0        | rad/s^2 |                              |
| MAX_JERK    | 30.0       | rad/s^3 |                              |
| NUM_DOF     | 5          | --      | Matches Isaac articulation   |

These are conservative values for a KR210-class robot. They were chosen
for smooth motion without risk of saturation.

## 10. Joint Configuration

### 10.1 ROS Order

ROS_ORDER = ["joint_1", "joint_2", "joint_3", "joint_5", "joint_4"]


This order matches the Isaac articulation DOF order. It is critical that
this order matches exactly.

### 10.2 Controlled Joints

CONTROLLED_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4"]


Only four joints receive commands from the PLC. joint_5 is not controlled
and holds its measured position.

### 10.3 PLC to ROS Mapping

| PLC Axis | ROS Joint | Index in ROS_ORDER |
|----------|-----------|--------------------|
| A1       | joint_1   | 0                  |
| A2       | joint_2   | 1                  |
| A3       | joint_3   | 2                  |
| A4       | joint_4   | 4 (not 3)          |
| --       | joint_5   | 3 (not controlled) |

Note: joint_4 is at index 4 because Isaac articulation order places
joint_5 at index 3.

### 10.4 Unit Conversion

- PLC side: degrees
- ROS side: radians
- Conversion is localized in plc_ros_bridge.py
- rad = deg * pi / 180
- deg = rad * 180 / pi
- No sign flips or offsets are used (JOINT_SIGN = +1)

## 11. Physics Twin (Isaac Sim)

### 11.1 Robot Configuration

| Parameter    | Value                             |
|--------------|-----------------------------------|
| ROBOT_PATH   | /World/Main_Robot_L_URDF/base_link|
| ROBOT_SCOPE  | /World/Main_Robot_L_URDF          |
| PUBLISH_HZ   | 60                                |
| DOF_ORDER    | joint_1, joint_2, joint_3, joint_5, joint_4 |

### 11.2 Joint Prim Paths

| Joint   | Prim Path                                     |
|---------|-----------------------------------------------|
| joint_1 | {ROBOT_SCOPE}/base_link/joint_1               |
| joint_2 | {ROBOT_SCOPE}/link_1/joint_2                  |
| joint_3 | {ROBOT_SCOPE}/link_2/joint_3                  |
| joint_4 | {ROBOT_SCOPE}/link_4/joint_4                  |
| joint_5 | {ROBOT_SCOPE}/link_3/joint_5                  |

### 11.3 Drive Gains

| Joint   | Stiffness | Damping | Max Force  |
|---------|-----------|---------|------------|
| joint_1 | 1,000,000 | 100,000 | 50,000,000 |
| joint_2 | 1,000,000 | 100,000 | 50,000,000 |
| joint_3 | 1,000,000 | 100,000 | 50,000,000 |
| joint_4 |   200,000 |  20,000 |  5,000,000 |
| joint_5 |   200,000 |  20,000 |  5,000,000 |

### 11.4 Gravity Compensation

GRAVITY_COMP = [0.0, 1200.0, 600.0, 0.0, 50.0]


Order matches DOF_ORDER:

| Index | Joint   | Value (N·m) |
|-------|---------|-------------|
| 0     | joint_1 | 0.0         |
| 1     | joint_2 | 1200.0      |
| 2     | joint_3 | 600.0       |
| 3     | joint_5 | 0.0         |
| 4     | joint_4 | 50.0        |

Constant torque per joint. Tuned empirically to hold the arm against
gravity with minimal residual error.

### 11.5 Setpoint Handling

On /servo_setpoints received:

- Store as dict {name: position_rad}

On every physics frame:

1. Read current joint positions.
2. For each joint i in DOF_ORDER:
   - If joint_i is in setpoint dict: target[i] = setpoint_dict[joint_i]
   - Else: target[i] = current[i] (hold)
3. Apply position targets.
4. Apply gravity compensation efforts.

Joints not present in the setpoint are held at their current position.

## 12. Verification Results

### 12.1 Tracking Accuracy

Command: CmdAng = [30.0, -30.0, 60.0, 0.0]
Settling time: 5 seconds

| Joint   | Setpoint   | Actual     | Error      |
|---------|------------|------------|------------|
| joint_1 | 30.0002°   | 30.0002°   | 0.0001°    |
| joint_2 | -29.9603°  | -29.9349°  | 0.0254°    |
| joint_3 | 60.0097°   | 60.0133°   | 0.0036°    |
| joint_4 | -0.0025°   | -0.0033°   | 0.0008°    |
| joint_5 | -0.4668°   | -0.4668°   | 0.0000°    |

Maximum error: 0.0254° on joint_2 under full gravity load.

### 12.2 Verification Tool

check_accuracy.py subscribes to /servo_setpoints and /joint_states and
prints a comparison table every second. It confirms that the Ruckig
output matches the achieved Isaac Sim physics state.

### 12.3 Clock Drift Measurement

Custom CLK1 protocol over UDP. Two-way timestamp exchange.

| Metric              | Value       |
|---------------------|-------------|
| Duration            | 900 s       |
| Samples             | 293,585     |
| Packet Loss         | 0.00%       |
| Drift               | -0.065 ppm  |
| Standard Error      | 0.00016 ppm |
| Asymmetry Bound     | ±16 µs      |
| RTT (minimum)       | 32.0 µs     |
| Allan Deviation @ 320s | 0.026 ppm |

A drift of -0.065 ppm means an accumulated error of less than 0.25 ms
per hour of operation. This is 80 times smaller than the PLC cycle
time (20 ms).

## 13. Known Issues

The following issues exist in the current code and have not been fixed:

### 13.1 CmdAng Not Written by PLC

In SCL_MAIN.txt, the lines that write MainRobotLeft_CmdAng are commented
out. The PLC currently controls motion through MC_* blocks and does not
update CmdAng. As a result, the bridge reads a constant CmdAng = 0.0.

Impact: Isaac Sim does not receive commands from the PLC through the
expected channel. Motion in Isaac is driven by MC_* blocks inside the PLC
only, and the KRC twin sees a static target.

### 13.2 joint_5 Gravity Compensation

GRAVITY_COMP[3] = 0.0 corresponds to joint_5. joint_5 is not controlled
and receives no gravity compensation. Under load it may drift freely.

### 13.3 DOF Order Mismatch Passes Silently

In physics_twin.py, if the Isaac articulation DOF order does not match
DOF_ORDER, the code prints a warning but continues. It should abort
instead.

### 13.4 TCP Reconnect Has No Backoff

In plc_ros_bridge.py, when the TCP connection drops, the bridge retries
every poll cycle (20 Hz). A longer outage fills the log with retry
messages. No backoff is implemented.

### 13.5 joint_5 Limits Not Defined

In krc_twin.py, AXIS_LIMITS_DEG contains only 4 joints. joint_5 has no
limits defined. It is not controlled, so this does not cause motion
errors, but it means no validation exists for it.

## 14. Current Limitations

| ID   | Limitation                                              |
|------|---------------------------------------------------------|
| L-01 | TCP/JSON is used instead of OPC UA                      |
| L-02 | joint_5 is not controlled (only 4 axes from PLC)        |
| L-03 | Isaac physics runs at 25-60 Hz (target 60)              |
| L-04 | Bridge polls at 20 Hz (50 ms latency floor)             |
| L-05 | Linux process is not real-time (no PREEMPT_RT)          |
| L-06 | No Cartesian motion or inverse kinematics in KRC twin   |
| L-07 | Single trajectory at a time (no queue, no blending)     |
| L-08 | No authentication on the TCP protocol                   |

## 15. Engineering Decisions

| ID      | Decision                                              |
|---------|-------------------------------------------------------|
| DEC-001 | PLC as heartbeat master, ROS as slave                 |
| DEC-002 | Counter-based handshake, not timestamp-based          |
| DEC-003 | Separate Python processes: 3.10 (ROS) + 3.11 (Isaac)  |
| DEC-004 | Ruckig for online trajectory generation               |
| DEC-005 | Jump protection with 0.3 rad threshold                |
| DEC-006 | Velocity-interface braking (Stop Category 1)          |
| DEC-007 | Feedback injection every servo cycle                  |
| DEC-008 | pass_to_input() after every Ruckig step               |
| DEC-009 | Three-state machine: IDLE / RUNNING / STOPPING        |
| DEC-010 | Command validation before use                         |
| DEC-011 | No fake zeros on missing feedback                     |
| DEC-012 | TCP/JSON transport instead of OPC UA                  |
| DEC-013 | Transport isolated from control logic                 |
| DEC-014 | Rate-limited logging via log_every()                  |
| DEC-015 | Reconnect loop with force-disable on failure          |
| DEC-016 | __slots__ on hot-path objects                         |
| DEC-017 | Pre-allocated buffers in servo loop                   |
| DEC-018 | ROS 2 QoS left at defaults                            |
| DEC-019 | Isaac Sim as physics twin only                        |
| DEC-020 | No PubSub in this version                             |

## 16. Troubleshooting Reference

| Issue                          | Cause                            | Fix                                      |
|--------------------------------|----------------------------------|------------------------------------------|
| CmdEnable stays FALSE          | Heartbeat detects ROS dead       | Ensure plc_ros_bridge.py is running      |
| DoesNotExist errors from PLCSIM| Wrong tag name                   | DB name is MAIN_KUKA_KR210_L150_L        |
| TCP connection refused         | Server not running or firewall   | Start plc_server_win.py, allow port 5000 |
| /servo_setpoints shows 0 Hz    | krc_twin in IDLE                 | Check /plc/axis_enable, /joint_states    |
| Robot does not move            | Isaac in Edit mode or wrong names| Check DOF_ORDER, switch to Play          |
| Oscillation or vibration       | Stiffness low or jump firing     | Increase stiffness or jump threshold     |
| Tracking error > 0.5°          | Gravity load or saturation       | Increase GRAVITY_COMP[1] or stiffness    |

## 17. References

- PROJECT_DOCUMENTATION.txt — Full documentation (18 sections)
- calib_*_summary.json — Clock drift session results
- calib_*_raw.csv — Raw clock drift measurements
- heartbeat.scl — OB30 heartbeat implementation
- SCL_MAIN.txt — OB1 motion logic implementation

## 18. License

Apache License 2.0

## 19. Contact

Nebras — nebras4u@gmail.com
GitHub: https://github.com/nebras4u/isaac-plc-digital-twin
