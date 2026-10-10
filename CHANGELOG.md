# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [3.0.0] — 2026-10-10 — Vendor-Neutral Platform

**Location:** [`vendor-neutral/`](vendor-neutral/) — Gen 3

### Changed (Breaking)

- **New folder `vendor-neutral/`** — the platform is now decoupled from any
  specific robot or PLC vendor.
- **Repository reorganized by environment:** `windows/`, `linux/`, `isaac/`,
  `config/`, `tools/`, `plc/`, `docs/`.
- **Transport abstraction:** all three transports (`tcp_json`, `opcua`,
  `plc_api`) live in `linux/plc_drivers.py` behind a single `PlcDriver`
  interface.
- **Signal abstraction:** address mapping + value extraction moved to
  `linux/plc_config.py`.
- **`plc_api` transport added** — direct PLCSIM Advanced API (Windows-only),
  selectable via `driver: plc_api` in `plc_ros_bridge.yaml`.

### Added

- `vendor-neutral/linux/plc_drivers.py` — `PlcDriver` ABC + 3 implementations
- `vendor-neutral/linux/plc_config.py` — `make_addr` + `SignalExtractor`
- `vendor-neutral/config/robot.yaml` — placeholder for future migration of
  the KRC twin's inline config
- Reorganized `vendor-neutral/tools/` (latency, clock drift)

### Preserved

- All previous generations moved to `archive/`:
  - `archive/v1.0-kuka/` — KUKA KR210 (TCP/JSON)
  - `archive/v0.2-yaskawa/` — Yaskawa GP110 (OPC UA)

### Migration Notes

| Old (v2.0.0) | New (v3.0.0) |
| :--- | :--- |
| `linux/plc_ros_bridge.py` (inline drivers) | `vendor-neutral/linux/plc_drivers.py` + `vendor-neutral/linux/plc_ros_bridge.py` |
| `linux/plc_config` (implicit) | `vendor-neutral/linux/plc_config.py` |
| flat repository | folder-by-environment inside `vendor-neutral/` |

### Known Limitations (v3.0.0)

| ID | Limitation |
| :--- | :--- |
| VL-01 | KRC twin still reads config inline; `robot.yaml` is not consumed yet |
| VL-02 | Yaskawa GP110 end-to-end verification on this branch is pending |
| VL-03 | No Cartesian motion or inverse kinematics layer |
| VL-04 | No real-time guarantees (no PREEMPT_RT) |
| VL-05 | No authentication on `tcp_json` transport |

---

## [2.0.0] — 2026-10-07 — Yaskawa Modular

**Location:** [`archive/v0.2-yaskawa/`](archive/v0.2-yaskawa/) — Gen 2

### Changed (Breaking)

- Monolithic bridge split into 6 focused modules under `src/plc_bridge/`:
  - `config.py`, `joint_map.py`, `plc_client.py`, `ros_bridge.py`,
    `main.py`, `plc_gui.py`
- Historical versions moved to `archive/`

### Added

- `plc_gui.py` — standalone tkinter GUI for manual control
- `docs/README_v2_full.md` — 742-line documentation with Appendices A–F

### Fixed

- **Hidden A5 problem** — root cause: `A5 != 0` introduces horizontal offset
  = `LF * sin(A5)`. Mathematical verification: 4.66 mm (theory) vs 4.68 mm
  (measured). Solution: `A5 = -(A1 + A4)` for tool-down orientation.

---

## [1.0.0] — 2026-09-15 — KUKA KR210 Full Digital Twin

**Location:** [`archive/v1.0-kuka/`](archive/v1.0-kuka/) — Gen 1

### Added

- **PLC Layer:** OB30 heartbeat (20 ms), OB1 motion logic (MC_PickUp,
  MC_Stkng_L/R, MC_Go2HomePos), TO_Kinematics
- **Windows Layer:** `plc_server_win.py` — TCP/JSON bridge over PLCSIM
- **Linux Layer:** `plc_ros_bridge.py`, `krc_twin.py`, `check_accuracy.py`
- **Isaac Layer:** `physics_twin.py`, modified URDF, `Full_Control.usd`
- **Tools:** clock drift measurement (`srv.py`, `cli.py`)
- **Docs:** complete 18-section project documentation

### Performance

| Joint | Setpoint | Actual | Error |
| :--- | :--- | :--- | :--- |
| joint_1 | 30.0002° | 30.0002° | 0.0001° |
| joint_2 | -29.9603° | -29.9349° | 0.0254° |
| joint_3 | 60.0097° | 60.0133° | 0.0036° |
| joint_4 | -0.0025° | -0.0033° | 0.0008° |
| joint_5 | -0.4668° | -0.4668° | 0.0000° |

Maximum error: **0.0254°** (joint_2, full gravity load).

### Clock Drift (15 min)

- Drift: **-0.065 ppm**
- Standard Error: 0.00016 ppm
- Asymmetry Bound: ±16.0 µs
- RTT min: 32.0 µs

---

## [0.1.0] — 2026-09-10 — Initial Prototype

**Location:** archived

### Added

- First working proof-of-concept for a KUKA KR210 digital twin
- Single monolithic Python file (`plc_ros_bridge4.py`)
- Basic TCP/JSON transport between PLCSIM Advanced and Linux

### Notes

- Superseded by v0.2.0 (Yaskawa migration)
- Preserved as historical reference

---

## Summary of Generations

| Version | Robot | Transport | Architecture | Location |
| :--- | :--- | :--- | :--- | :--- |
| v0.1.0 | KUKA KR210 | TCP/JSON | Monolithic | (superseded) |
| v1.0.0 | KUKA KR210 | TCP/JSON | 5 processes | `archive/v1.0-kuka/` |
| v0.2.0 | Yaskawa GP110 | OPC UA | Modular (6 files) | `archive/v0.2-yaskawa/` |
| **v3.0.0** | **Any robot** | **Any transport** | **Vendor-neutral** | **`vendor-neutral/`** |

---

## Versioning Policy

This project uses [Semantic Versioning](https://semver.org/):

- **MAJOR** (X.0.0) — incompatible API or architecture changes
- **MINOR** (0.X.0) — new features, backward compatible
- **PATCH** (0.0.X) — bug fixes, backward compatible

---

## Links

- **Repository:** https://github.com/Nebras4u/isaac-plc-digital-twin
- **Gen 3 (active):** [`vendor-neutral/`](vendor-neutral/)
- **Gen 2 (reference):** [`archive/v0.2-yaskawa/`](archive/v0.2-yaskawa/)
- **Gen 1 (reference):** [`archive/v1.0-kuka/`](archive/v1.0-kuka/)

***

*Maintained by [Nebras](https://github.com/Nebras4u) — nebras4u@gmail.com*
