# plc_client.py
"""OPC UA layer. منطق منقول حرفياً من plc_ros_bridge4.py"""

import logging
from asyncua import Client, ua
from config import PLC_URL, NS, DB_TCP

log = logging.getLogger("bridge.plc")

# ---------- Node IDs ----------
NODE_CMD_ENABLE = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_CmdEnable"'
NODE_CMD_ACK    = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_CmdAck"'
NODE_ACT_ANG    = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActAng"'
NODE_ACT_POS    = f'ns={NS};s="{DB_TCP}"."MainRobotLeft_ActPos"'
NODE_CMD_ANG    = NODE_ACT_ANG


async def opc_write(node, value, variant_type):
    wv = ua.WriteValue()
    wv.NodeId = node.nodeid
    wv.AttributeId = ua.AttributeIds.Value
    wv.Value = ua.DataValue(ua.Variant(value, variant_type))
    wv.Value.StatusCode = None
    wv.Value.SourceTimestamp = None
    wv.Value.ServerTimestamp = None

    params = ua.WriteParameters()
    params.NodesToWrite = [wv]
    await node.write_params(params)