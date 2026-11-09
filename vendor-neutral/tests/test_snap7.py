# tests/test_snap7.py
"""Tests for Snap7 driver and schema (Siemens S7-1500 over S7comm).

Coverage
--------
- Schema (``PlcConfig.rack`` / ``slot``; ``SignalConfig.db_num`` /
  ``byte_offset`` / ``bit_offset`` / ``s7_type`` consistency).
- ``Snap7Driver._resolve_s7_type`` (explicit + fallback).
- ``Snap7Driver._decode`` for ``bool``, ``int16``, ``int32``, ``lreal``.
- ``Snap7Driver._write_one`` for the same four types, verifying
  big-endian layout and boolean bit-set/bit-clear semantics.
- ``Snap7Driver.read`` groups reads by DB (one ``db_read`` per DB).

None of these tests require a live PLC. Driver tests use a
``MockSnap7Client`` that records calls. Tests requiring ``snap7.util``
are skipped if ``python-snap7`` is not installed; the schema tests run
unconditionally.

Run with::

    pytest test_snap7.py -v
    # or, only the schema part (no snap7 dependency):
    pytest test_snap7.py -k "PlcConfig or SignalConfig" -v
"""
from __future__ import annotations

import struct

import pytest
from pydantic import ValidationError

from bridge_config import PlcConfig, SignalConfig


# ---------------------------------------------------------------------------
# Optional python-snap7 dependency
# ---------------------------------------------------------------------------
try:
    import snap7  # noqa: F401
    from snap7 import util as s7util  # noqa: F401

    HAS_SNAP7 = True
except ImportError:
    HAS_SNAP7 = False

requires_snap7 = pytest.mark.skipif(
    not HAS_SNAP7, reason="python-snap7 not installed",
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
class _Cfg:
    """Minimal ``PlcConfig``-shaped object accepted by ``Snap7Driver``."""

    host = "192.168.1.1"
    rack = 0
    slot = 1
    timeout = 1.0


def _meta(db=2, byte=0, bit=0, typ="float", s7_type=None):
    """Build a ``_addr_meta`` entry as the bridge would."""
    return {
        "db": db, "byte": byte, "bit": bit,
        "type": typ, "s7_type": s7_type,
    }


class MockSnap7Client:
    """Stand-in for ``snap7.client.Client``.

    ``db_read`` returns zero-filled bytes unless the test pre-seeds a
    region via ``_data[(db, start, size)] = bytes``. ``db_write`` records
    every call and also updates ``_data`` so a read after a write returns
    the written value (useful for roundtrip tests).
    """

    def __init__(self):
        self.reads: list[tuple[int, int, int]] = []
        self.writes: list[tuple[int, int, bytes]] = []
        self._data: dict[tuple[int, int, int], bytes] = {}

    def db_read(self, db, start, size):
        self.reads.append((db, start, size))
        return bytearray(self._data.get((db, start, size), bytes(size)))

    def db_write(self, db, start, data):
        self.writes.append((db, start, bytes(data)))
        self._data[(db, start, len(data))] = bytes(data)


def _make_driver():
    """Instantiate a ``Snap7Driver`` with a mock client, or skip."""
    if not HAS_SNAP7:
        pytest.skip("python-snap7 not installed")
    from plc_ros_bridge import Snap7Driver

    drv = Snap7Driver(_Cfg())
    drv.client = MockSnap7Client()
    return drv


# ===========================================================================
# Schema — PlcConfig
# ===========================================================================
class TestPlcConfigSnap7:
    def test_snap7_requires_host(self):
        with pytest.raises(ValidationError, match="requires 'plc.host'"):
            PlcConfig(driver="snap7", rack=0, slot=1)

    def test_snap7_default_rack_slot(self):
        cfg = PlcConfig(driver="snap7", host="1.2.3.4")
        assert cfg.rack == 0
        assert cfg.slot == 1

    def test_snap7_custom_rack_slot(self):
        cfg = PlcConfig(driver="snap7", host="1.2.3.4", rack=0, slot=2)
        assert cfg.slot == 2

    @pytest.mark.parametrize("rack", [8, -1])
    def test_snap7_rack_out_of_range(self, rack):
        with pytest.raises(ValidationError):
            PlcConfig(driver="snap7", host="1.2.3.4", rack=rack)

    @pytest.mark.parametrize("slot", [32, -1])
    def test_snap7_slot_out_of_range(self, slot):
        with pytest.raises(ValidationError):
            PlcConfig(driver="snap7", host="1.2.3.4", slot=slot)


# ===========================================================================
# Schema — SignalConfig
# ===========================================================================
class TestSignalConfigSnap7:
    def test_snap7_fields_optional(self):
        sig = SignalConfig(addr="DB2.DBD0", type="float", dir="read")
        assert sig.db_num is None
        assert sig.byte_offset is None
        assert sig.s7_type is None

    def test_snap7_fields_accepted(self):
        sig = SignalConfig(
            addr="DB2.DBD216", type="float", dir="read",
            db_num=2, byte_offset=216, bit_offset=0, s7_type="lreal",
        )
        assert sig.db_num == 2
        assert sig.byte_offset == 216
        assert sig.s7_type == "lreal"

    def test_bit_offset_default(self):
        sig = SignalConfig(
            addr="DB2.DBX0.0", type="bool", dir="read",
            db_num=2, byte_offset=0, s7_type="bool",
        )
        assert sig.bit_offset == 0

    @pytest.mark.parametrize("db_num", [0, 65536])
    def test_db_num_out_of_range(self, db_num):
        with pytest.raises(ValidationError):
            SignalConfig(
                addr="DB2.DBD0", type="float", dir="read",
                db_num=db_num, byte_offset=0,
            )

    @pytest.mark.parametrize("byte_offset", [-1, 65536])
    def test_byte_offset_out_of_range(self, byte_offset):
        with pytest.raises(ValidationError):
            SignalConfig(
                addr="DB2.DBD0", type="float", dir="read",
                db_num=2, byte_offset=byte_offset,
            )

    @pytest.mark.parametrize("bit_offset", [-1, 8])
    def test_bit_offset_out_of_range(self, bit_offset):
        with pytest.raises(ValidationError):
            SignalConfig(
                addr="DB2.DBX0.0", type="bool", dir="read",
                db_num=2, byte_offset=0, bit_offset=bit_offset,
            )

    def test_s7_type_bool_requires_type_bool(self):
        with pytest.raises(ValidationError, match="s7_type='bool'"):
            SignalConfig(
                addr="DB2.DBX0.0", type="int", dir="read",
                db_num=2, byte_offset=0, s7_type="bool",
            )

    def test_s7_type_int16_requires_type_int(self):
        with pytest.raises(ValidationError, match="s7_type='int16'"):
            SignalConfig(
                addr="DB2.DBW0", type="float", dir="read",
                db_num=2, byte_offset=0, s7_type="int16",
            )

    def test_s7_type_int32_requires_type_int(self):
        with pytest.raises(ValidationError, match="s7_type='int32'"):
            SignalConfig(
                addr="DB2.DBD0", type="bool", dir="read",
                db_num=2, byte_offset=0, s7_type="int32",
            )

    def test_s7_type_lreal_requires_type_float(self):
        with pytest.raises(ValidationError, match="s7_type='lreal'"):
            SignalConfig(
                addr="DB2.DBD0", type="int", dir="read",
                db_num=2, byte_offset=0, s7_type="lreal",
            )

    def test_s7_type_omitted_ok(self):
        for typ in ("bool", "int", "float"):
            SignalConfig(
                addr="DB2.DBX0.0", type=typ, dir="read",
                db_num=2, byte_offset=0,
            )


# ===========================================================================
# Driver — type resolution
# ===========================================================================
@requires_snap7
class TestSnap7DriverResolve:
    def test_resolve_explicit(self):
        drv = _make_driver()
        for st in ("bool", "int16", "int32", "lreal"):
            assert drv._resolve_s7_type({"s7_type": st}) == st

    @pytest.mark.parametrize("typ,expected", [
        ("bool", "bool"),
        ("int", "int16"),
        ("float", "lreal"),
    ])
    def test_resolve_fallback(self, typ, expected):
        drv = _make_driver()
        assert drv._resolve_s7_type({"type": typ}) == expected

    def test_resolve_empty_defaults_to_lreal(self):
        drv = _make_driver()
        assert drv._resolve_s7_type({}) == "lreal"

    def test_resolve_explicit_none_falls_back(self):
        drv = _make_driver()
        assert drv._resolve_s7_type(
            {"s7_type": None, "type": "int"}) == "int16"

    def test_size_map(self):
        from plc_ros_bridge import Snap7Driver
        assert Snap7Driver._SIZE == {
            "bool": 1, "int16": 2, "int32": 4, "lreal": 8,
        }


# ===========================================================================
# Driver — decode (raw bytes -> python values)
# ===========================================================================
@requires_snap7
class TestSnap7DriverDecode:
    def test_decode_bool_true(self):
        from plc_ros_bridge import Snap7Driver
        assert Snap7Driver._decode(bytes([0x01]), 0, {"bit": 0}, "bool") is True

    def test_decode_bool_false(self):
        from plc_ros_bridge import Snap7Driver
        assert Snap7Driver._decode(bytes([0xFE]), 0, {"bit": 0}, "bool") is False

    def test_decode_bool_bit_3(self):
        from plc_ros_bridge import Snap7Driver
        # 0x08 = bit 3 set
        assert Snap7Driver._decode(bytes([0x08]), 0, {"bit": 3}, "bool") is True
        assert Snap7Driver._decode(bytes([0x08]), 0, {"bit": 2}, "bool") is False

    def test_decode_int16_positive(self):
        from plc_ros_bridge import Snap7Driver
        assert Snap7Driver._decode(
            bytes([0x00, 0x64]), 0, {}, "int16") == 100

    def test_decode_int16_negative(self):
        from plc_ros_bridge import Snap7Driver
        # 0xFF9C = -100 (two's complement, big-endian)
        assert Snap7Driver._decode(
            bytes([0xFF, 0x9C]), 0, {}, "int16") == -100

    def test_decode_int32_positive(self):
        from plc_ros_bridge import Snap7Driver
        assert Snap7Driver._decode(
            bytes([0x00, 0x00, 0x01, 0x00]), 0, {}, "int32") == 256

    def test_decode_int32_negative(self):
        from plc_ros_bridge import Snap7Driver
        # 0xFFFFFF00 = -256
        assert Snap7Driver._decode(
            bytes([0xFF, 0xFF, 0xFF, 0x00]), 0, {}, "int32") == -256

    def test_decode_lreal(self):
        from plc_ros_bridge import Snap7Driver
        data = struct.pack(">d", 1.5)
        assert Snap7Driver._decode(data, 0, {}, "lreal") == 1.5

    def test_decode_lreal_negative(self):
        from plc_ros_bridge import Snap7Driver
        data = struct.pack(">d", -3.14159)
        val = Snap7Driver._decode(data, 0, {}, "lreal")
        assert abs(val + 3.14159) < 1e-9


# ===========================================================================
# Driver — write (python values -> bytes)
# ===========================================================================
@requires_snap7
class TestSnap7DriverWrite:
    def test_write_bool_sets_bit(self):
        drv = _make_driver()
        meta = _meta(byte=0, bit=3, typ="bool", s7_type="bool")
        drv._write_one(meta, True)
        assert len(drv.client.writes) == 1
        db, start, data = drv.client.writes[0]
        assert (db, start) == (2, 0)
        assert (data[0] >> 3) & 1 == 1

    def test_write_bool_preserves_other_bits(self):
        drv = _make_driver()
        # Bits 0,1,2,4,5,6,7 set; bit 3 clear
        drv.client._data[(2, 0, 1)] = bytes([0xF7])
        meta = _meta(byte=0, bit=3, typ="bool", s7_type="bool")
        drv._write_one(meta, True)
        _, _, data = drv.client.writes[0]
        assert data[0] == 0xFF

    def test_write_bool_clears_bit(self):
        drv = _make_driver()
        # Bit 3 set; must be cleared
        drv.client._data[(2, 0, 1)] = bytes([0x08])
        meta = _meta(byte=0, bit=3, typ="bool", s7_type="bool")
        drv._write_one(meta, False)
        _, _, data = drv.client.writes[0]
        assert (data[0] >> 3) & 1 == 0

    def test_write_int16_big_endian(self):
        drv = _make_driver()
        meta = _meta(byte=10, typ="int", s7_type="int16")
        drv._write_one(meta, 100)
        db, start, data = drv.client.writes[0]
        assert (db, start) == (2, 10)
        assert data == bytes([0x00, 0x64])

    def test_write_int16_negative(self):
        drv = _make_driver()
        meta = _meta(byte=0, typ="int", s7_type="int16")
        drv._write_one(meta, -100)
        _, _, data = drv.client.writes[0]
        assert data == bytes([0xFF, 0x9C])

    def test_write_int32_big_endian(self):
        drv = _make_driver()
        meta = _meta(byte=0, typ="int", s7_type="int32")
        drv._write_one(meta, 256)
        _, _, data = drv.client.writes[0]
        assert data == bytes([0x00, 0x00, 0x01, 0x00])

    def test_write_int32_negative(self):
        drv = _make_driver()
        meta = _meta(byte=0, typ="int", s7_type="int32")
        drv._write_one(meta, -256)
        _, _, data = drv.client.writes[0]
        assert data == bytes([0xFF, 0xFF, 0xFF, 0x00])

    def test_write_lreal_big_endian_ieee754(self):
        drv = _make_driver()
        meta = _meta(byte=0, typ="float", s7_type="lreal")
        drv._write_one(meta, 1.5)
        _, _, data = drv.client.writes[0]
        assert data == struct.pack(">d", 1.5)

    @pytest.mark.parametrize("value", [
        0.0, 1.0, -1.0, 3.14159265358979, 1e-10, -1e10, 123456.789,
    ])
    def test_write_lreal_roundtrip(self, value):
        drv = _make_driver()
        meta = _meta(byte=0, typ="float", s7_type="lreal")
        drv._write_one(meta, value)
        _, _, data = drv.client.writes[0]
        assert struct.unpack(">d", data)[0] == pytest.approx(value)


# ===========================================================================
# Driver — read grouping
# ===========================================================================
@requires_snap7
class TestSnap7DriverRead:
    def test_read_empty(self):
        drv = _make_driver()
        assert drv.read([]) == {}

    def test_read_groups_by_db(self):
        drv = _make_driver()
        drv._addr_meta = {
            "a1": _meta(db=2, byte=0, typ="float", s7_type="lreal"),
            "a2": _meta(db=2, byte=8, typ="float", s7_type="lreal"),
            "a3": _meta(db=3, byte=0, typ="float", s7_type="lreal"),
        }
        drv.read(["a1", "a2", "a3"])
        # One db_read per DB: DB2 (span 0..16) and DB3 (span 0..8)
        assert len(drv.client.reads) == 2
        dbs = sorted(r[0] for r in drv.client.reads)
        assert dbs == [2, 3]

    def test_read_span_uses_min_max(self):
        drv = _make_driver()
        drv._addr_meta = {
            "a1": _meta(db=2, byte=100, typ="float", s7_type="lreal"),
            "a2": _meta(db=2, byte=50, typ="float", s7_type="lreal"),
        }
        drv.read(["a1", "a2"])
        # One read: lo=50, span = (100+8) - 50 = 58
        assert len(drv.client.reads) == 1
        db, lo, span = drv.client.reads[0]
        assert db == 2
        assert lo == 50
        assert span == 58

    def test_read_missing_meta_returns_none(self):
        drv = _make_driver()
        drv._addr_meta = {}
        out = drv.read(["missing"])
        assert out["missing"] is None

    def test_read_bool_uses_bit(self):
        drv = _make_driver()
        drv.client._data[(2, 0, 1)] = bytes([0x08])  # bit 3 set
        drv._addr_meta = {
            "a1": _meta(db=2, byte=0, bit=3, typ="bool", s7_type="bool"),
            "a2": _meta(db=2, byte=0, bit=2, typ="bool", s7_type="bool"),
        }
        out = drv.read(["a1", "a2"])
        assert out["a1"] is True
        assert out["a2"] is False


# ===========================================================================
# Module-level availability
# ===========================================================================
def test_has_snap7_flag_consistent():
    """``plc_ros_bridge._HAS_SNAP7`` must match reality."""
    from plc_ros_bridge import _HAS_SNAP7
    assert _HAS_SNAP7 is HAS_SNAP7


@pytest.mark.skipif(HAS_SNAP7, reason="snap7 installed; skip unavailable path")
def test_snap7_driver_raises_without_snap7():
    from plc_ros_bridge import Snap7Driver
    with pytest.raises(RuntimeError, match="python-snap7"):
        Snap7Driver(_Cfg())
