#!/usr/bin/env python3
"""PLC <-> ROS 2 bridge (vendor neutral).

v4.5 — auto-start measurement:
  * Measurement starts automatically after N warm-up ticks
    (default 10) and stops on CmdEnable=False or shutdown.
  * motion_test.py does NOT touch CmdEnable (it stays True so the
    robot holds position); only CmdAng is written.
  * cycle_ns = write_done_ns - t_rs = true PLC->ROS->Isaac->ROS->PLC loop.

Implementation lives in the plc_bridge package. This file is the
stable entry point (ros2 run / python3 plc_ros_bridge.py).
"""
from plc_bridge.constants import (  # noqa: F401
    MSG_TYPES, CAST, JS_STALE_AGE_S, ACT_ANG_DEADBAND_DEG, ACT_ANG_REFRESH_S,
    PUB_STREAM_QOS, SUB_STREAM_QOS, _SERVER_TYPE_MAP, _HAS_DIAG, _INT_TOL,
)
from plc_bridge.wire import (  # noqa: F401
    _wire_type_check, _to_finite_float, _to_int_checked,
    _to_bool_from_str, _to_bool_from_num,
)
from plc_bridge.latency import _LatencyProbe  # noqa: F401
from plc_bridge.drivers import (  # noqa: F401
    PlcDriver, TcpJsonDriver, AdsDriver, OpcUaDriver, PlcApiDriver,
    Snap7Driver, DRIVERS,
)
from plc_bridge.drivers.snap7 import _HAS_SNAP7  # noqa: F401
from plc_bridge.node import PlcRosBridge, main  # noqa: F401

if __name__ == "__main__":
    main()
