from plc_bridge.drivers.base import PlcDriver
from plc_bridge.drivers.tcp_json import TcpJsonDriver
from plc_bridge.drivers.ads import AdsDriver
from plc_bridge.drivers.opcua import OpcUaDriver
from plc_bridge.drivers.plc_api import PlcApiDriver
from plc_bridge.drivers.snap7 import Snap7Driver

DRIVERS = {
    "tcp_json": TcpJsonDriver,
    "opcua":    OpcUaDriver,
    "plc_api":  PlcApiDriver,
    "ads":      AdsDriver,
    "snap7":    Snap7Driver,
}

__all__ = [
    "PlcDriver",
    "TcpJsonDriver",
    "AdsDriver",
    "OpcUaDriver",
    "PlcApiDriver",
    "Snap7Driver",
    "DRIVERS",
]
