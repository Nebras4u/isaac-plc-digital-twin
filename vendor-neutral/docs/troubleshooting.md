# Troubleshooting Log — KR210 Digital Twin

> **KUKA KR210 + Siemens S7-1500 + PLCSIM Advanced v6.0 + ROS 2 Jazzy + Isaac Sim 5.1**
>
> Practical index of every problem encountered, with root cause and fix.

---

## Index

| # | Title | Category |
| :--- | :--- | :--- |
| A1 | [Python 3.11 (Isaac) vs 3.12 (ROS 2) — `rclpy` import fails](#a1) | Python & Runtime |
| A2 | [pythonnet: `import clr` fails or is missing](#a2) | Python & Runtime |
| A3 | [PLCSIM instance name mismatch](#a3) | Python & Runtime |
| B1 | [Wrong IP: PLC address used instead of Windows host](#b1) | Connectivity |
| B2 | [Windows Firewall blocking port 5000](#b2) | Connectivity |
| B3 | [Docker container cannot reach Windows host](#b3) | Connectivity |
| B4 | [Wrong DB prefix: `L150L` vs `L150_L`](#b4) | Connectivity |
| C1 | [Connection closed by peer (S7comm)](#c1) | S7 Protocol |
| C2 | [S7 protocol error `0x84, 0x04`](#c2) | S7 Protocol |
| C3 | [Invalid address `0x05` — Optimized Block Access](#c3) | S7 Protocol |
| C4 | [CmdEnable flag erased within one scan cycle](#c4) | S7 Protocol |
| D1 | [Type mismatch `-29`: tag was DInt, not LReal](#d1) | Types & Symbols |
| D2 | [Type change not reflected after download](#d2) | Types & Symbols |
| D3 | [Stale tag list: `-47, NotUpToDate`](#d3) | Types & Symbols |
| D4 | [OPC UA `BadNodeIdInvalid` on arrays](#d4) | Types & Symbols |
| D5 | [Empty response `{"values": {}}` from TCP server](#d5) | Types & Symbols |
| D6 | [Missing ARRAYS support in the TCP server](#d6) | Types & Symbols |
| E1 | [Robot "bounces" at the target then settles](#e1) | Motion Quality |
| E2 | [Sustained oscillation: damping below critical](#e2) | Motion Quality |
| E3 | [joint_4 / joint_5 drift continuously](#e3) | Motion Quality |
| E4 | [Feedback loop between Ruckig and Isaac](#e4) | Motion Quality |
| E5 | [joint_4 stuck at ~2° steady-state error](#e5) | Motion Quality |
| E6 | [Acceleration violations 11× KR210 spec](#e6) | Motion Quality |
| E7 | [Isaac delay of 10 s before the robot moves](#e7) | Motion Quality |
| F1 | [PLC forces `CmdEnable := FALSE` continuously](#f1) | Safety |
| F2 | [Robot moves briefly on link loss](#f2) | Safety |
| G1 | [`/joint_states` stuck at 20–25 Hz instead of 60](#g1) | Isaac Physics |
| G2 | [Script Editor: `'await' outside function`](#g2) | Isaac Physics |
| G3 | [Extension registry 404](#g3) | Isaac Physics |
| H1 | [`.bashrc` corrupted by aggressive `sed`](#h1) | Tooling |
| H2 | [Empty `plc_server_win.py` on Windows](#h2) | Tooling |
| H3 | [Multi-line paste mangled in terminal](#h3) | Tooling |
| H4 | [`ros2 topic list` shows stale topics](#h4) | Tooling |
| H5 | [`rclpy.shutdown()` double-call warning](#h5) | Tooling |
| Z1 | [`No thread available` when creating `zmq.Context()` inside rclpy](#z1) | ZeroMQ |
| Z2 | [`No thread available` persists even with `io_threads=0`](#z2) | ZeroMQ |
| Z3 | [Node invisible in `ros2 node list` when ZeroMQ context created first](#z3) | ZeroMQ |
| Z4 | [PUB/SUB race: first messages lost](#z4) | ZeroMQ |
| Z5 | [`ros2 topic list` empty despite node running (ROS_DOMAIN_ID mismatch)](#z5) | ZeroMQ |

---

## A. Python & Runtime Environment

<a id="a1"></a>
### A1 — Python 3.11 (Isaac) vs 3.12 (ROS 2) — `rclpy` import fails

**Symptom:** `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'` — the C extension is for 3.11, system is 3.12.

**Cause:** Isaac Sim 5.1 uses Python 3.11. ROS 2 Jazzy on Ubuntu 24.04 uses 3.12. The shell sourced ROS 2's `setup.bash`, polluting `PYTHONPATH`.

**Fix:** In the same shell as Isaac Sim, do NOT source ROS 2. Use separate terminals for ROS CLI. If unavoidable:

```bash
export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy
```

<a id="a2"></a>
### A2 — pythonnet: `import clr` fails or is missing

**Symptom:** `ModuleNotFoundError: No module named 'Siemens'` or `import clr` silently does nothing.

**Cause:** (a) DLL not loaded before import; (b) pythonnet 3.2.0 changed the import mechanism; (c) Python 3.14 has no wheels.

**Fix:** Use Python 3.12 + `pythonnet==3.0.5`. Always load DLL first:

```python
os.add_dll_directory(API_DIR)
clr.AddReference(os.path.join(API_DIR, "Siemens.Simatic.Simulation.Runtime.Api.x64.dll"))
```

<a id="a3"></a>
### A3 — PLCSIM instance name mismatch

**Symptom:** `-4, DoesNotExist` on `CreateInterface(name)`.

**Cause:** The name in code (`"ros2_"`) did not match the PLCSIM Control Panel instance name.

**Fix:** Read the exact name from PLCSIM Control Panel. Enumerate if unsure:

```python
for info in SimulationRuntimeManager.RegisteredInstanceInfo:
    print(info.Name)
```

---

## B. PLC Connectivity & Addressing

<a id="b1"></a>
### B1 — Wrong IP: PLC address used instead of Windows host

**Symptom:** `Errno 113, No route to host` or `TimeoutError` when connecting.

**Cause:** Two distinct IPs exist: the virtual PLC (`192.168.100.60`) and the Windows host running the TCP server (`192.168.100.18`). The bridge must connect to the **host**.

**Fix:** Verify with `ipconfig /all` on Windows. Use the host LAN IP.

<a id="b2"></a>
### B2 — Windows Firewall blocking port 5000

**Symptom:** `Errno 111, Connection refused`.

**Fix:**

```powershell
netsh advfirewall firewall add rule name="PLCSIM 5000" dir=in action=allow protocol=TCP localport=5000
```

<a id="b3"></a>
### B3 — Docker container cannot reach Windows host

**Symptom:** Works on host, fails inside container.

**Cause:** Docker bridge networking isolates the container from the LAN.

**Fix:** Run the container with `--net=host`. Verify:

```bash
docker inspect <ct> --format '{{.HostConfig.NetworkMode}}'   # → host
```

<a id="b4"></a>
### B4 — Wrong DB prefix: `L150L` vs `L150_L`

**Symptom:** Every read returns `-4, DoesNotExist`.

**Cause:** Missing underscore in the Data Block name.

**Fix:** Enumerate `instance.TagInfos` and search for a known substring (`"CmdAng"`) to discover the exact prefix. Never trust the spec sheet.

---

## C. S7comm & Protocol-Level Issues

<a id="c1"></a>
### C1 — Connection closed by peer (S7comm)

**Cause:** PLCSIM Advanced running in **Softbus** mode, not TCP/IP. Only Advanced V3+ exposes S7comm over a real Virtual Ethernet Adapter.

**Fix:** Change Online Access to `TCP/IP Single Adapter (Promiscuous Mode)`, select the Siemens virtual adapter, restart.

<a id="c2"></a>
### C2 — S7 protocol error `0x84, 0x04`

**Cause:** PUT/GET communication disabled in TIA Portal.

**Fix:** PLC Properties → Protection & Security → Connection mechanisms → ✅ *Permit access with PUT/GET*. Recompile, download.

<a id="c3"></a>
### C3 — Invalid address `0x05` — Optimized Block Access

**Cause:** DB created with default *Optimized Block Access*; variables have no fixed offsets.

**Fix:** Right-click DB → Properties → Attributes → ❌ disable *Optimized block access*. Note: this **resets all values to zero**.

<a id="c4"></a>
### C4 — CmdEnable flag erased within one scan cycle

**Symptom:** Writing `CmdEnable=True` succeeds, but it reverts to `False` instantly.

**Cause:** Byte-level race. `CmdAck` and `CmdEnable` shared byte 986; Snap7 `write_bool` does read-modify-write on the whole byte.

**Fix:** Move `CmdAck` to a separate byte (e.g. `1030.0`). Tag-level protocols (OPC UA, PLCSIM API) do not have this problem.

---

## D. Data Types & Symbol Resolution

<a id="d1"></a>
### D1 — Type mismatch `-29`: tag was DInt, not LReal

**Symptom:** `ReadFloat` and `ReadDouble` both fail, `ReadInt32` succeeds.

**Fix:** Check `info.DataType` first. Match:

| TIA Type | Python Method |
| :--- | :--- |
| Bool | ReadBool / WriteBool |
| DInt | ReadInt32 / WriteInt32 |
| Real | ReadFloat / WriteFloat |
| LReal | ReadDouble / WriteDouble |

<a id="d2"></a>
### D2 — Type change not reflected after download

**Cause:** PLCSIM caches the symbol table; a download over a RUN-state instance does not invalidate it.

**Fix:** Stop instance → delete/re-register → download → Run → `UpdateTagList()` in Python.

<a id="d3"></a>
### D3 — Stale tag list: `-47, NotUpToDate`

**Fix:** Always call `instance.UpdateTagList()` after connecting and after any project download.

<a id="d4"></a>
### D4 — OPC UA `BadNodeIdInvalid` on arrays

**Cause:** PLC arrays expose as a **single** node in OPC UA, not indexed nodes.

**Fix:** Use `"ActAng"` (no `[i]`). Write the whole list:

```python
writes[addr] = [math.degrees(x) for x in self._js_rad]
```

<a id="d5"></a>
### D5 — Empty response `{"values": {}}` from TCP server

**Cause:** Old `plc_server_win.py` on Windows silently filtered unknown tags.

**Fix:** Update the server file. Consider returning explicit errors for unknown tags during commissioning.

<a id="d6"></a>
### D6 — Missing ARRAYS support in the TCP server

**Symptom:** `{"written": {}}` when writing `ActAng` as a whole.

**Fix:** Add an `ARRAYS` dictionary and handle whole-array read/write in `read_tag` / `write_tag`.

---

## E. Motion Quality & Control Stability

<a id="e1"></a>
### E1 — Robot "bounces" at the target then settles

**Cause:** Constant gravity compensation via `set_joint_efforts()` was fighting the position drives. When the robot approached the goal, the fixed gravity torque pushed it past.

**Fix:** Remove `set_joint_efforts()` entirely. Use built-in `set_gravity_compensation(True)` (pose-aware), or none.

<a id="e2"></a>
### E2 — Sustained oscillation: damping below critical

**Cause:** For a mass-spring-damper, stability requires `D ≥ 2·√(K·I)`. Joints 3 and 4 were below this.

**Fix:** Set `D = 1.5 × 2·√(K·I)` for a safety margin. Read `JointEquivalentInertia` from the URDF prim.

<a id="e3"></a>
### E3 — joint_4 / joint_5 drift continuously

**Cause:** Very low inertia on off-axis joints — constant drift of 40–60 deg/s.

**Fix:** (a) joint_5 → free (`K=D=F=0`); (b) joint_4 → remove from `CONTROLLED_JOINTS` if it is not mission-critical.

<a id="e4"></a>
### E4 — Feedback loop between Ruckig and Isaac

**Cause:** `sync_state(actual)` called every 60 Hz cycle injects Isaac's sub-degree numerical noise back into Ruckig, closing a high-frequency loop.

**Fix:** Remove `sync_state()` from the RUNNING branch. Only sync on rising edge of enable or on detected jump (> 0.3 rad).

<a id="e5"></a>
### E5 — joint_4 stuck at ~2° steady-state error

**Cause:** PLC commands an unreachable target (gravity load prevents joint_4 from reaching 0°); Ruckig keeps generating corrections.

**Fix:** Remove joint_4 from the controlled set, or add a small deadband.

<a id="e6"></a>
### E6 — Acceleration violations 11× KR210 spec

**Cause:** No deadband — every sub-degree error triggered a full corrective command.

**Fix:** Apply deadband of 0.1° in both `krc_twin.py` and `physics_twin.py`. Cap Ruckig limits to real KR210 values:

```python
MAX_VEL  = [1.83, 1.83, 2.09, 3.12, 3.12]
MAX_ACC  = [5.2,  5.2,  6.1,  8.7,  8.7]
```

<a id="e7"></a>
### E7 — Isaac delay of 10 s before the robot moves

**Cause:** Calling both `set_joint_position_targets()` AND `set_joint_efforts()` on the same joints. PhysX allows only one control mode per joint.

**Fix:** Use position targets only. Enable built-in gravity compensation (does not conflict).

---

## F. Safety & Watchdogs

<a id="f1"></a>
### F1 — PLC forces `CmdEnable := FALSE` continuously

**Cause:** Heartbeat protocol working as designed. The bridge was not acknowledging `Heartbeat_Counter`, so `Comm_State → Dead`.

**Fix:** In the bridge, echo the counter back to `ROS_Ack_Counter` on every change:

```python
if hb != self._last_ack:
    write(ROS_Ack_Counter, hb)
    self._last_ack = hb
```

<a id="f2"></a>
### F2 — Robot moves briefly on link loss

**Cause:** No watchdog — Isaac kept following stale setpoints.

**Fix:** Add watchdogs at every layer:

- **Bridge:** publish `/plc/emergency_stop = True` on `_set_connected(False)`.
- **KRC twin:** reject commands older than 0.5 s.
- **Physics:** ignore setpoints older than 0.5 s.

---

## G. Isaac Sim Physics Loop

<a id="g1"></a>
### G1 — `/joint_states` stuck at 20–25 Hz instead of 60

**Cause:** `create_timer(1/60, cb)` is unreliable inside Isaac Sim's async frame loop; physics runs at 25–30 Hz on the test hardware.

**Fix:** Publish directly from the frame loop, not via a timer:

```python
while True:
    node.step()
    node.publish_states()
    rclpy.spin_once(node, timeout_sec=0.0)
    await omni.kit.app.get_app().next_update_async()
```

<a id="g2"></a>
### G2 — Script Editor: `'await' outside function`

**Fix:** Wrap the async loop in `async def main():` and launch with `asyncio.ensure_future(main())`.

<a id="g3"></a>
### G3 — Extension registry 404

**Cause:** NVIDIA deprecated the old registry server (Oct 2025).

**Fix:**

```bash
isaacsim --ext-folder /isaac-sim/extscache \
         --/app/extensions/registryEnabled=false \
         --/app/extensions/onlyEnabled=true
```

---

## H. Tooling & Environment

<a id="h1"></a>
### H1 — `.bashrc` corrupted by aggressive `sed`

**Symptom:** `bash: /root/.bashrc: line 100: syntax error: unexpected end of file`

**Fix:** Restore from backup or rewrite. Validate with `bash -n /root/.bashrc` before saving.

<a id="h2"></a>
### H2 — Empty `plc_server_win.py` on Windows

**Cause:** Saved as 0 bytes (Notepad failure or wrong path).

**Fix:** Verify size:

```powershell
python -c "print(len(open('plc_server_win.py').read()))"
```

Save into a project folder, not `C:\Windows\system32`.

<a id="h3"></a>
### H3 — Multi-line paste mangled in terminal

**Symptom:** Fragments of one line appear inside another.

**Fix:** Save scripts to a file and run the file. Use bracketed paste mode. Never paste multi-line code from chat directly into a shell.

<a id="h4"></a>
### H4 — `ros2 topic list` shows stale topics

**Cause:** DDS keeps topic names cached briefly after the publisher dies.

**Fix:** Use `ros2 topic info <topic>` and check *Publisher count*. Also try `--no-daemon` if the daemon is stale.

<a id="h5"></a>
### H5 — `rclpy.shutdown()` double-call warning

**Fix:** Wrap in try/except — cosmetic only.

---

## Z. ZeroMQ Integration

> **Context:** An attempt was made to add ZeroMQ as a fourth transport alongside `tcp_json`, `opcua`, and `plc_api`. The transport itself worked (60 Hz received from Windows), but integration with rclpy inside the bridge process failed. The feature was abandoned. These notes are preserved so future attempts can avoid the same traps.

<a id="z1"></a>
### Z1 — `No thread available` when creating `zmq.Context()` inside rclpy

**Symptom:**

```
zmq.error.ZMQError: No thread available (addr='tcp://192.168.100.18:5555')
```

Emitted every ~2 s from `ZmqDriver.connect()` inside the bridge.

**Cause:** `pyzmq 27.2.0` interacts poorly with the thread pool that FastRTPS creates inside an rclpy process. When `zmq.Context()` is called from within a class method after `rclpy.init()`, ZeroMQ cannot allocate its I/O thread.

**Fix attempted:** `zmq.Context(io_threads=0)` — asked ZeroMQ not to allocate an I/O thread. This worked in an isolated script but not inside the full bridge.

<a id="z2"></a>
### Z2 — `No thread available` persists even with `io_threads=0`

**Symptom:** Same error, despite `io_threads=0`.

**Cause:** The ZeroMQ context was being created **after** many rclpy publishers/subscriptions existed. Even with `io_threads=0`, ZeroMQ requires an internal structure that was blocked by the FastRTPS thread pool.

**Fix attempted:** Create the ZeroMQ context in `main()` **before** `rclpy.init()`, then reuse it via a module-level global.

```python
_ZMQ_GLOBAL_CTX = None

def _init_zmq():
    global _ZMQ_GLOBAL_CTX
    if _ZMQ_GLOBAL_CTX is None:
        import zmq
        _ZMQ_GLOBAL_CTX = zmq.Context(io_threads=1)
    return _ZMQ_GLOBAL_CTX

def main():
    _init_zmq()          # BEFORE rclpy.init()
    rclpy.init()
    node = PlcRosBridge()
    ...
```

This removed the `No thread available` error, but introduced Z3.

<a id="z3"></a>
### Z3 — Node invisible in `ros2 node list` when ZeroMQ context created first

**Symptom:** Bridge starts cleanly, `[ZmqDriver] first msg` appears, `driver connected (early)` prints — but `ros2 node list --no-daemon` does not show `/plc_ros_bridge`.

**Cause:** Creating the ZeroMQ context **before** `rclpy.init()` changed the process state in a way that prevented rclpy from completing DDS discovery. The exact mechanism was not identified — likely a multicast socket or memory-pool conflict in `libzmq` vs `libfastrtps`.

**Diagnosis:**

```
$ ros2 node list --no-daemon
/physics_twin
/trajectory_twin
        ← /plc_ros_bridge missing
```

**Workaround tried:** None that succeeded while keeping ZeroMQ in the same process.

**Resolution:** Abandoned the ZeroMQ transport. Kept `tcp_json` as the production transport.

<a id="z4"></a>
### Z4 — PUB/SUB race: first messages lost

**Symptom:** Bridge logged `signal 'cmd_ang' = None -> no value` on the first tick, then recovered.

**Cause:** ZeroMQ PUB does not queue messages for subscribers that are not yet connected. The bridge's SUB socket needed ~100–500 ms to connect, but the Windows PUB was already streaming.

**Fix attempted:**

- On Windows: `pub.setsockopt(zmq.SNDHWM, 1000)` and `time.sleep(1.0)` after `bind()`.
- On Linux: `sub.setsockopt(zmq.RCVHWM, 1000)` and `sub.setsockopt(zmq.RECONNECT_IVL, 100)`.

This reduced the warm-up period but did not eliminate the initial warnings. It is a known ZeroMQ semantics issue, not a bug.

<a id="z5"></a>
### Z5 — `ros2 topic list` empty despite node running (ROS_DOMAIN_ID mismatch)

**Symptom:** Every `ros2 topic list` from a new terminal showed only `/parameter_events` and `/rosout`. The bridge was running and reading from ZeroMQ.

**Cause:** The bridge was launched on `ROS_DOMAIN_ID=45`, but the new terminal had `ROS_DOMAIN_ID=0` (the default). ROS 2 DDS isolates traffic by domain ID — the two terminals could not see each other.

**Diagnosis:**

```bash
# Terminal with bridge running
$ echo $ROS_DOMAIN_ID
45

# New terminal
$ echo $ROS_DOMAIN_ID
0
```

**Fix:** Export the same `ROS_DOMAIN_ID` in every terminal:

```bash
export ROS_DOMAIN_ID=45
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**Note:** This issue is not specific to ZeroMQ. It affects any multi-terminal ROS 2 session. It was discovered during the ZeroMQ debugging but is documented here because it masqueraded as a ZeroMQ problem for several hours.

---

## Summary Table

| Category | Count | Most critical |
| :--- | :--- | :--- |
| A — Python & Runtime | 3 | A1 — Isaac/ROS Python ABI mismatch |
| B — Connectivity | 4 | B4 — DB prefix |
| C — S7 Protocol | 4 | C4 — byte-level race on CmdEnable |
| D — Types & Symbols | 6 | D4 — OPC UA arrays |
| E — Motion Quality | 7 | E4 — Ruckig ↔ Isaac feedback loop |
| F — Safety | 2 | F1 — heartbeat contract |
| G — Isaac Physics | 3 | G1 — frame loop vs timer |
| H — Tooling | 5 | H3 — paste mangling |
| Z — ZeroMQ Integration | 5 | Z3 — node invisible in DDS |
| **Total** | **39** | |

---

## Key Takeaways

| # | Rule |
| :--- | :--- |
| 1 | Verify, don't assume. `ps aux \| grep` before reasoning about "what should be running". |
| 2 | Silent filtering hides bugs. Return explicit errors during commissioning. |
| 3 | Exact strings matter — `L150L` ≠ `L150_L`. Enumerate the runtime, don't trust the spec. |
| 4 | Safety logic forcing FALSE is not a bug. Fulfill the protocol contract; don't bypass it. |
| 5 | Never inject measured state into an online trajectory generator every cycle. |
| 6 | Always add a deadband near the target (0.1°). |
| 7 | Tag-level protocols (OPC UA, PLCSIM API) avoid the byte-race class entirely. |
| 8 | One dedicated terminal per long-running process. Paste into an editor first. |
| 9 | Not every symptom needs a fix. Document the limitation if accuracy is already sufficient. |
| 10 | Derive damping from inertia (`D ≥ 2·√(K·I)`), never guess. |
| 11 | `ROS_DOMAIN_ID` must match in every terminal — this bug masquerades as many other failures. |
| 12 | Do not mix ZeroMQ and rclpy in the same process. If ZeroMQ is needed, isolate it in a separate process. |

---

**KR210 Digital Twin — Troubleshooting Log**
KUKA KR210 · Siemens S7-1500 · PLCSIM Advanced v6.0 · ROS 2 Jazzy · Isaac Sim 5.1
Last updated: 2026-10-10