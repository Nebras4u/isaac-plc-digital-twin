# tests/test_beckhoff_ads_connection.py
# اختبارات TwinCAT 3 (ADS) - البديل الرسمي لاختبارات Siemens

import pyads
import pytest

# --- العناوين الحقيقية التي نجحت في بيئتك ---
TARGET_AMS = "192.168.122.215.1.1"
TARGET_IP = "192.168.122.215"
LOCAL_AMS = "192.168.122.1.1.1"
PORT = pyads.PORT_TC3PLC1  # 851

# --- أسماء المتغيرات في مشروع TwinCAT (تأكد من مطابقتها) ---
VAR_BOOL = "MAIN.bRosOnline"
VAR_REAL = "GVL.CmdAng"  # لاحظ أننا غيرناها إلى REAL


@pytest.fixture(scope="module")
def plc_connection():
    """فتح اتصال ADS مرة واحدة لكل جلسة اختبار."""
    pyads.open_port()
    pyads.set_local_address(LOCAL_AMS)
    plc = pyads.Connection(TARGET_AMS, PORT, TARGET_IP)
    plc.open()
    if not plc.is_open:
        pytest.fail(f"فشل فتح الاتصال بـ {TARGET_AMS} على {TARGET_IP}")
    yield plc
    plc.close()
    pyads.close_port()


def test_connection_is_open(plc_connection):
    """التأكد من أن الاتصال مفتوح."""
    assert plc_connection.is_open is True


def test_read_bool(plc_connection):
    """اختبار قراءة متغير Boolean."""
    try:
        val = plc_connection.read_by_name(VAR_BOOL, pyads.PLCTYPE_BOOL)
        assert isinstance(val, bool)
        print(f"\n{VAR_BOOL} = {val}")
    except Exception as e:
        pytest.fail(f"فشلت قراءة {VAR_BOOL}: {e}")


def test_read_real(plc_connection):
    """اختبار قراءة متغير Real (32-bit)."""
    try:
        val = plc_connection.read_by_name(VAR_REAL, pyads.PLCTYPE_REAL)
        assert isinstance(val, float)
        print(f"\n{VAR_REAL} = {val}")
    except Exception as e:
        pytest.fail(f"فشلت قراءة {VAR_REAL}: {e}")


def test_write_real_and_read_back(plc_connection):
    """اختبار كتابة متغير Real والتحقق من القراءة."""
    try:
        original = plc_connection.read_by_name(VAR_REAL, pyads.PLCTYPE_REAL)
        new_value = original + 1.0
        plc_connection.write_by_name(VAR_REAL, new_value, pyads.PLCTYPE_REAL)
        read_back = plc_connection.read_by_name(VAR_REAL, pyads.PLCTYPE_REAL)
        assert abs(read_back - new_value) < 1e-3, (
            f"الكتابة فشلت: كتبنا {new_value} لكن قرأنا {read_back}"
        )
        print(f"\n{VAR_REAL}: {original} -> {new_value} -> {read_back}")
    except Exception as e:
        pytest.fail(f"فشلت كتابة/قراءة {VAR_REAL}: {e}")
