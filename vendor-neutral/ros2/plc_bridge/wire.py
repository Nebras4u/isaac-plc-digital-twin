import math

from plc_bridge.constants import _INT_TOL


def _wire_type_check(expected, v):
    if v is None:
        return None
    if isinstance(v, str) and v.startswith("ERR:"):
        return v
    if expected is None:
        return v
    if isinstance(v, (list, tuple)):
        if expected == "int":
            for x in v:
                if isinstance(x, bool) or not isinstance(x, int):
                    return f"ERR:TYPE:int_array:{type(x).__name__}"
            return list(v)
        if expected == "float":
            for x in v:
                if isinstance(x, bool) or not isinstance(x, (int, float)):
                    return f"ERR:TYPE:float_array:{type(x).__name__}"
            return list(v)
        if expected == "bool":
            for x in v:
                if not isinstance(x, bool):
                    return f"ERR:TYPE:bool_array:{type(x).__name__}"
            return list(v)
        if expected == "string":
            for x in v:
                if not isinstance(x, str):
                    return f"ERR:TYPE:string_array:{type(x).__name__}"
            return list(v)
        return v
    if expected == "bool":
        return v if isinstance(v, bool) else f"ERR:TYPE:bool:{type(v).__name__}"
    if expected == "int":
        return v if (isinstance(v, int) and not isinstance(v, bool)) \
                 else f"ERR:TYPE:int:{type(v).__name__}"
    if expected == "float":
        return v if (isinstance(v, (int, float)) and not isinstance(v, bool)) \
                 else f"ERR:TYPE:float:{type(v).__name__}"
    if expected == "string":
        return v if isinstance(v, str) else f"ERR:TYPE:string:{type(v).__name__}"
    return v


def _to_finite_float(v):
    f = float(v)
    if not math.isfinite(f):
        raise ValueError(f"non_finite:{f!r}")
    return f


def _to_int_checked(v):
    if isinstance(v, bool):
        return int(v)
    f = float(v)
    if not math.isfinite(f):
        raise ValueError(f"non_finite:{f!r}")
    r = round(f)
    if abs(f - r) > _INT_TOL:
        raise ValueError(f"fractional:{f!r}")
    return int(r)


def _to_bool_from_str(v):
    s = v.strip().lower()
    if s in ("true", "1", "yes"):
        return True
    if s in ("false", "0", "no"):
        return False
    raise ValueError(f"bad_bool:{v!r}")


def _to_bool_from_num(v):
    f = float(v)
    if f == 0.0:
        return False
    if f == 1.0:
        return True
    raise ValueError(f"non_bool_numeric:{f!r}")
