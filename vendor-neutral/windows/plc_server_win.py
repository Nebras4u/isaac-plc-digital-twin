# plc_server_win.py
# Runs on Windows. Serves PLCSIM tags over TCP as JSON.
#
# v4.0 changes:
#   - All replies serialized with allow_nan=False (defense in depth).
#   - read_many() sanitizes each value: NaN/Inf -> "ERR:non_finite".
#   - New "types" command: returns declared type per tag/array.
#     Used by the bridge at boot for a type handshake.
#
# Implementation lives in the plcsim package. This file is the
# stable Windows entry point.

from plcsim.runtime import connect, API_DIR, DLL_PATH  # noqa: F401
import plcsim.runtime as _rt
from plcsim.tags import (  # noqa: F401
    DB, BOOL, LREAL, DINT, INT, TAGS, ARRAYS, INSTANCE_NAME,
)
from plcsim.timing import TIMING, _rec, _timing_reporter  # noqa: F401
from plcsim.api import _read_one, _write_one  # noqa: F401
from plcsim.batch import (  # noqa: F401
    BATCH_ENABLED, _MAX_BATCH_FAILS, _B, _TYPE_NAMES, _VALUE_ATTR, _PY_CAST,
    _ARR_CACHE, _ARR_LOCK, _clr_members, _code_ok, _init_batch, _get_value,
    _build_array, _fill_values, _cached_array, _batch_read, _batch_write,
    _batch_failed, _selftest_batch, setup_batch,
)
from plcsim.io import (  # noqa: F401
    read_tag, write_tag, _expand, read_many, write_many, _sanitize,
)
from plcsim.protocol import _send_json, handle_client  # noqa: F401
from plcsim.server import main  # noqa: F401

connect()
instance = _rt.instance
SimulationRuntimeException = _rt.SimulationRuntimeException

if __name__ == "__main__":
    main()
