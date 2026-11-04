# vendor-neutral/tests/test_beckhoff_ads_connection.py
# اختبارات TwinCAT 3 (ADS) - مبنية على tc_R.py و tc_W.py

import pyads
import pytest

# --- العناوين التي نجحت في الاختبار ---
TARGET_AMS = "111.111.111.3.1.1"
TARGET_IP = "192.168.50.1"
LOCAL_AMS = "192.168.50.2.1.1"
PORT = pyads.PORT_TC3PLC1  # 851

# --- أسماء المتغيرات في مشروع TwinCAT ---
VAR_BOOL = "MAIN.bRosOnline"
VAR_LREAL = "GVL.MainRobotLeft.CmdAng[1]"


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


def test_read_lreal_array_element(plc_connection):
    """اختبار قراءة عنصر من مصفوفة LREAL."""
    try:
        val = plc_connection.read_by_name(VAR_LREAL, pyads.PLCTYPE_LREAL)
        assert isinstance(val, float)
        print(f"\n{VAR_LREAL} = {val}")
    except Exception as e:
        pytest.fail(f"فشلت قراءة {VAR_LREAL}: {e}")


def test_write_lreal_array_element(plc_connection):
    """اختبار كتابة عنصر من مصفوفة LREAL والتحقق من القراءة."""
    try:
        original = plc_connection.read_by_name(VAR_LREAL, pyads.PLCTYPE_LREAL)
        new_value = original + 1.0
        plc_connection.write_by_name(VAR_LREAL, new_value, pyads.PLCTYPE_LREAL)
        read_back = plc_connection.read_by_name(VAR_LREAL, pyads.PLCTYPE_LREAL)
        assert abs(read_back - new_value) < 1e-6, (
            f"الكتابة فشلت: كتبنا {new_value} لكن قرأنا {read_back}"
        )
        print(f"\n{VAR_LREAL}: {original} -> {new_value} -> {read_back}")
    except Exception as e:
        pytest.fail(f"فشلت كتابة/قراءة {VAR_LREAL}: {e}")
