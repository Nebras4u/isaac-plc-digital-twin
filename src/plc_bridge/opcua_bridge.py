# opcua_bridge.py
"""طبقة الاتصال بـ OPC UA — قراءة وكتابة فقط."""
import asyncio
from asyncua import Client
from config import PLC_URL, PLC_ARRAYS, PLC_FLAGS


class OpcUaBridge:
    """جسر OPC UA بسيط: connect / read / write / close."""

    def __init__(self, url: str = PLC_URL):
        self.url = url
        self.client = None
        self.arr_nodes = {}
        self.flag_nodes = {}

    async def connect(self):
        """فتح الاتصال وتحضير Node IDs مرة واحدة فقط."""
        self.client = Client(url=self.url)
        await self.client.connect()
        self.arr_nodes = {k: self.client.get_node(v) for k, v in PLC_ARRAYS.items()}
        self.flag_nodes = {k: self.client.get_node(v) for k, v in PLC_FLAGS.items()}
        print(f"[OPC UA] Connected to {self.url}")

    async def close(self):
        if self.client:
            await self.client.disconnect()
            print("[OPC UA] Disconnected.")

    async def read_arrays(self) -> dict:
        """قراءة كل المصفوفات. يعيد dict {name: [v1, v2, ...]}"""
        try:
            vals = await self.client.read_values(list(self.arr_nodes.values()))
            return dict(zip(self.arr_nodes.keys(), vals))
        except Exception as e:
            return {"error": str(e)}

    async def read_flags(self) -> dict:
        """قراءة كل الأعلام (Booleans)."""
        try:
            vals = await self.client.read_values(list(self.flag_nodes.values()))
            return dict(zip(self.flag_nodes.keys(), vals))
        except Exception as e:
            return {"error": str(e)}

    async def read_all(self) -> tuple[dict, dict]:
        """قراءة كل شيء دفعة واحدة — يُستخدم في الحلقة الرئيسية."""
        arrays = await self.read_arrays()
        flags = await self.read_flags()
        return arrays, flags

    async def write(self, node_id: str, value):
        """كتابة قيمة إلى Node. جاهزة للاستخدام المستقبلي (مثلاً Go2Pos)."""
        node = self.client.get_node(node_id)
        await node.write_value(value)