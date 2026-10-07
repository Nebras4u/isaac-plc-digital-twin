# Yaskawa GP110 v0.2 — Monolithic Prototype

This folder preserves the monolithic implementation of the Yaskawa GP110
digital twin, before the v2.0 modular refactoring.

## Files

- `plc_ros_bridge4.py` — Single-file bridge (OPC UA + ROS 2 + joint mapping)
- `README_v0.2.md` — Full v0.2 documentation

## History

- **v0.2.0** — Initial Yaskawa migration (from KUKA KR210)
- **v0.2.1** — URDF modified for tool-down orientation
- **v0.2.2** — Hidden A5 diagnosed

## Successor

The monolithic file was refactored in **v2.0.0** into six modules under
`src/plc_bridge/`. Runtime behavior was preserved exactly.

See [`docs/README_v2_full.md`](../../docs/README_v2_full.md) Appendix F
for the refactoring rationale.
