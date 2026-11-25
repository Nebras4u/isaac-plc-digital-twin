# Technical Architecture & Code Walkthrough: `kuka_supervised_control` (v2.0)

## 1. System Overview & Industrial Philosophy
This project implements a **Safety-Supervised Control Architecture** for a 6-DOF **KUKA KR210** robot. Instead of allowing ROS 2 to drive the robot actuators directly, the system enforces **Defense in Depth** across three decoupled layers running deterministically at **50 Hz (20 ms cycle)**:

1. **ROS 2 (`ros2_control` C++ Hardware Interface):** Acts purely as a **Motion Proposer**. It computes desired joint positions and sends proposals to the PLC.
2. **Siemens S7-1500 PLC (`PLCSIM Advanced` OB30):** Acts as a **Pure Safety Supervisor** (no Technology Objects or kinematics inside the PLC). It validates every proposal for CRC integrity, liveness, state readiness, and safety limits before forwarding approved commands to the plant.
3. **NVIDIA Isaac Sim (`isaac_plant.py`):** Acts as the **Physical Plant**. It executes approved targets in the physics engine and reports actual joint positions and velocities back to the PLC.

---

## 2. End-to-End Execution Workflow (20 ms Loop)

Every **20 milliseconds**, a single motion command travels through six strict stages:

1. **Controller Manager (`write()`):**
   * The active ROS 2 controller writes target joint angles (in radians) into `hw_commands_rad_`, and the safety controller writes `1.0` to `safety/axis_enable`.
   * `KukaSupervisorSystem::write()` checks for `NaN`/`Inf`, increments `proposal_id_` if any joint target changed by more than `0.01°`, timestamps the snapshot (`written_at`), and pushes it into a wait-free buffer.
2. **Real-Time Executor Thread (`RtExecutor::send_proposal()`):**
   * Wakes up every `20 ms` (`SCHED_FIFO` capable).
   * Checks command freshness: if `now - written_at > 100 ms`, the command is marked **stale** and `axis_enable` is immediately dropped to `0` (protecting against a frozen ROS 2 Controller Manager).
   * Applies defensive clamping against `soft_min_deg`/`soft_max_deg` and the PLC's per-cycle `max_step_deg` budget.
   * Encodes a **72-byte Big-Endian packet (`DB100`)** with a sequence counter (`seq`), heartbeat echo (`hb_echo`), and an **IEEE 802.3 CRC32** checksum, then writes it via Snap7.
3. **PLC Supervision (`FB_KukaSupervisor` in `OB30`):**
   * Increments `plc_heartbeat` and deserializes `DB100` (from ROS) and `DB39` (from Isaac Sim) to verify their CRC32 checksums and protocol version (`version == 1`).
   * Monitors liveness: if `DB100.seq` stops changing for `> 100 ms`, the PLC transitions to `LOST (8)` and drops `axis_enable`. If `DB39.plant_heartbeat` stops for `> 200 ms`, it transitions to `FAULT (6)`.
   * Evaluates the state machine (`BOOT -> COMM_WAIT -> IDLE -> ENABLING -> READY -> RUNNING`).
   * If all checks pass (`rejectMask == 0`), it sets `decision = 1 (Accepted)` and copies `proposed_pos_deg` into **`DB37` (`approved_pos_deg`)**. Otherwise, it rejects the command and holds the last safe position.
4. **Plant Execution (`IsaacPlantBridge` in Isaac Sim):**
   * A dedicated background thread reads **`DB37` (64 bytes)** at 50 Hz, verifies its CRC32, and confirms that `plc_heartbeat` is actively updating (`< 150 ms`).
   * If `plc_axis_enable` is true and no E-Stop is active, it applies a final local step limiter (`MAX_STEP_DEG`) and updates `approved_pos_deg`.
   * On every physics tick (`on_physics_step`), Isaac Sim applies the targets to the first 6 DOFs via `omni.physics.tensors` (`ArticulationView::set_dof_position_targets`).
5. **Plant Feedback (`DB39` -> PLC -> `DB101`):**
   * Isaac Sim reads actual joint positions and velocities (`get_dof_positions`, `get_dof_velocities`), packs them into **`DB39` (128 bytes)** with `plant_heartbeat` and CRC32, and writes them to the PLC.
   * The PLC mirrors these actual values into **`DB101` (192 bytes)** along with `plc_state`, `decision`, `reject_reason`, and `max_step_deg`.
6. **Controller Manager (`read()`):**
   * `RtExecutor::ingest_feedback()` reads `DB101`, validates CRC32 and finiteness, tracks heartbeat freshness, and updates the state buffer.
   * `KukaSupervisorSystem::read()` publishes the fresh joint states to ROS 2 (`/joint_states`) and exposes read-only safety interfaces (`comm_ok`, `plc_state`, `plc_estop`, `plc_axis_enable`).

---

## 3. Breakdown of the C++ Codebase (v2.0)

### A. `supervisor_protocol.hpp` & `.cpp` (Wire Contract)
* Defines the exact binary layout of `DB100` (72 bytes) and `DB101` (192 bytes).
* Implements Big-Endian serialization helpers (`be::put_f64`, `be::get_u32`, etc.) because Siemens S7-1500 stores multi-byte numbers in Big-Endian format while x86 Linux is Little-Endian.
* Generates a **compile-time CRC32 lookup table** (`make_crc_table()`), allowing checksum calculation in nanoseconds without runtime overhead.

### B. `spsc_triple_buffer.hpp` (Lock-Free Concurrency)
* Replaces `realtime_tools::RealtimeBuffer` (which uses a mutex under the hood).
* Implements a **Wait-Free Single-Producer / Single-Consumer Triple Buffer** using atomic index swapping (`std::atomic::exchange`).
* Guarantees that the Controller Manager thread (`read()`/`write()`) and the PLC I/O thread (`RtExecutor::run()`) **never block or stall each other**.

### C. `plc_transport.hpp` & `plc_bridge.hpp/.cpp` (Network Abstraction)
* `PlcTransport` defines a clean abstract interface so Snap7 can be swapped for OPC UA or PROFINET in the future without touching real-time logic.
* `PlcBridge` wraps the **Snap7 C API** (`Cli_Create`, `Cli_ConnectTo`, `Cli_DBRead`, `Cli_DBWrite`) with zero heap allocations after `configure()`.
* Implements **exponential reconnect backoff** (`100 ms` up to `2000 ms`) on network drops so a disconnected PLC does not flood the network or CPU.

### D. `rt_executor.hpp` & `.cpp` (Deterministic I/O Loop)
* Runs the 20 ms communication cycle in an isolated thread.
* Optionally configures Linux real-time scheduling (`SCHED_FIFO`), CPU core pinning (`pthread_setaffinity_np`), and memory locking (`mlockall`) when `enable_rt` is set to `true`.
* Tracks diagnostic counters (`io_errors`, `crc_errors`, `stale_cmds`, `hb_timeouts`, `overruns`, `max_cycle_us`) using `std::atomic` variables.

### E. `kuka_supervisor_system.hpp` & `.cpp` (`ros2_control` Plugin)
* Inherits from `hardware_interface::SystemInterface`.
* **Safe Defaults:** Initializes `hw_axis_enable_ = 0.0` so the robot remains locked until a controller explicitly commands `1.0`.
* **Offloaded Logging (`diag_thread_`):** Spawns a separate low-priority background thread that wakes up once per second to print warnings via `RCLCPP_WARN`, keeping the real-time `read()` and `write()` methods 100% deterministic.

---

## 4. Key Bug Fixes & Engineering Lessons Resolved in v1 -> v2
1. **S7-1500 Memory Alignment:** In non-optimized Siemens Data Blocks (`Optimized_Access = FALSE`), arrays always align to even byte offsets. Replacing `Array of Byte` padding with `pad_byte` + `pad_word` aligned `DB101.crc` to offset `168` and `DB39.crc` to offset `104`.
2. **Snap7 C++ vs. C Symbols:** `libsnap7.so` exports C symbols (`Cli_*`) rather than C++ `TS7Client` class symbols. Switching `PlcBridge` to the C API eliminated runtime `undefined symbol` errors.
3. **ROS 2 Jazzy `robot_description` Topic:** In ROS 2 Jazzy, `controller_manager` requires `robot_state_publisher` to publish `/robot_description` rather than reading it from a node parameter.
4. **Isaac Sim Physics Tensor API:** Used `omni.physics.tensors` (`ArticulationView`) directly with explicit `robot_indices` to read and command the first 6 revolute joints of the KUKA KR210 articulation reliably.


---

## 5. Industrial Safety Hardening & Verified Fault-Injection Tests (v2.1)

In addition to the core 50 Hz loop, the supervisor enforces **latched fail-safe state transitions** across all three layers:

* **Directional Soft-Limit Recovery (`FB_KukaSupervisor`):** Prevents any outward excursion beyond `SoftLimMin`/`SoftLimMax` (`rej=SOFT_LIMIT`), while permitting inward recovery commands back into the safe envelope.
* **Latched E-Stop & Comm-Loss (`ESTOP` / `LOST`):** Any E-Stop trigger or heartbeat loss (`> 100 ms`) immediately drops `axis_enable` and locks the PLC state machine. Reconnecting the network or releasing E-Stop **never** resumes motion automatically; an explicit 4-step operator recovery sequence is required:
  1. `safety/axis_enable` = `0.0`
  2. `safety/reset` = `1.0`
  3. `safety/reset` = `0.0`
  4. `safety/axis_enable` = `1.0`

### Verified Safety Test Matrix

| Test Case | Fault Injected | PLC Response | Isaac Sim & ROS 2 Response | Status |
| :--- | :--- | :--- | :--- | :---: |
| **T1: Soft-Limit & Recovery** | Command `joint_a2` > `15.0°`, then command `-17.2°` | Rejects outward (`16#0002`), accepts inward recovery | Stops at `15.0°` before floor contact; recovers cleanly | **PASS** |
| **T2: Latched E-Stop** | Publish `safety/estop = 1.0`, then `0.0` | Latches in `ESTOP (7)` until explicit `Reset` pulse | Drops `axis_enable` immediately; holds pose | **PASS** |
| **T3: ROS 2 Process Freeze** | `pkill -STOP -f ros2_control_node` during motion | Detects `seq` timeout (`> 100 ms`), enters `LOST (8)` | Halts arm mid-trajectory within `100 ms` | **PASS** |
| **T4: Transient Glitch** | Momentary PLC stop/restart mid-trajectory | Enters latched `LOST (8)`; refuses auto-resume | `PlcBridge` reconnects automatically; arm stays locked | **PASS** |
