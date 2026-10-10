# Migration to `vendor-neutral/`

This document explains how the current layout was created from the previous generations and what changed at the file level.

---

## What moved

| Before (flat repo) | After (vendor-neutral/) |
| :--- | :--- |
| `plc_server_win.py` (root) | `windows/plc_server_win.py` |
| `plc_ros_bridge.py` (root) | `linux/plc_ros_bridge.py` |
| `krc_twin.py` (root) | `linux/krc_twin.py` |
| `physics_twin.py` (root) | `isaac/physics_twin.py` |
| `Main_Robot_L_URDF_tool.urdf` (root) | `isaac/Main_Robot_L_URDF_tool.urdf` |
| `Full_Control.usd` (root) | `isaac/Full_Control.usd` |
| `robot.yaml` (root) | `config/robot.yaml` |
| `plc_ros_bridge.yaml` (root) | `config/plc_ros_bridge.yaml` |
| `latency_simpel.py` (root) | `tools/latency_simpel.py` |
| `latency_academic.py` (root) | `tools/latency_academic.py` |
| `srv.py` (root) | `tools/srv.py` |
| `cli.py` (root) | `tools/cli.py` |
| `info.txt` (root) | `tools/info.txt` |
| `SCL_heartbeat_OB30.txt` (root) | `plc/SCL_heartbeat_OB30.txt` |
| `SCL_MAIN.txt` (root) | `plc/SCL_MAIN.txt` |
| `SCL_heartbeat_OB30-init.txt` (root) | `plc/SCL_heartbeat_OB30-init.txt` |
| `SCL_MAIN-init.txt` (root) | `plc/SCL_MAIN-init.txt` |
| `DB.txt` (root) | `plc/DB.txt` |

---

## What did NOT change

The following files keep the same logic and content as before. Only their paths changed:

- `windows/plc_server_win.py` — same code, moved to `windows/`.
- `isaac/physics_twin.py` — same code, moved to `isaac/`.
- `linux/krc_twin.py` — same code, moved to `linux/`.
- All `tools/*.py` — unchanged.
- All `plc/*.txt` — unchanged.
- `config/plc_ros_bridge.yaml` — unchanged.

---

## What was extracted

Two new files were introduced inside `linux/`:

### `linux/plc_drivers.py`

Contains:
- `PlcDriver` abstract base class (the interface).
- `TcpJsonDriver` — talks to `plc_server_win.py`.
- `OpcUaDriver` — native OPC UA client (requires `asyncua`).
- `PlcApiDriver` — direct PLCSIM Advanced API (Windows-only).
- `DRIVERS` dictionary used as a factory.

### `linux/plc_config.py`

Contains:
- `make_addr(raw, driver_name, ns, db)` — builds the full PLC address according to the selected transport.
- `SignalExtractor` — class that normalizes raw PLC values into standard Python types, with warn-once semantics.

Both modules are imported by `linux/plc_ros_bridge.py`. Nothing else uses them at the moment.

---

## What was NOT split (deliberately)

- `linux/krc_twin.py` — Ruckig wrapper and state machine remain in a single file. This is intentional: the file is under 300 lines and self-contained.
- `linux/plc_ros_bridge.py` — heartbeats, deg/rad conversion, and topic publish/subscribe remain in a single file. Only the drivers and the signal extraction were extracted.
- `isaac/physics_twin.py` — unchanged.
- `windows/plc_server_win.py` — unchanged.

A deeper split (domain / application / infrastructure) was considered but rejected because the marginal benefit did not justify the added complexity for a four-file project.

---

## Running after migration

```bash
# terminal 1 (Windows)
cd vendor-neutral/windows
python plc_server_win.py

# terminal 2 (Isaac Sim Script Editor)
# open and run vendor-neutral/isaac/physics_twin.py

# terminal 3 (Linux)
cd vendor-neutral/linux
python3 plc_ros_bridge.py --ros-args -p config:=../config/plc_ros_bridge.yaml

# terminal 4 (Linux)
cd vendor-neutral/linux
python3 krc_twin.py
```

---

## Rollback

If anything goes wrong, revert to the previous commit:

```bash
git log --oneline | head -5
git revert <commit-hash>
```

Because every change is a `git mv` plus two new files, rollback is mechanical — no data loss.

---
*Maintained by Nebras — nebras4u@gmail.com*
