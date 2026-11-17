import math
import time

from plc_bridge.wire import (
    _to_finite_float, _to_int_checked, _to_bool_from_str, _to_bool_from_num,
)


class PlcRosBridgeValidate:
    def _bump(self, name, reason):
        key = reason.split(":", 1)[0] if isinstance(reason, str) else str(reason)
        if not key:
            key = "bad_value"
        self._stats[name][key] += 1
        self._stats[name]["rejected_total"] += 1

    def _range_check(self, s, v):
        lo, hi = s.get("min"), s.get("max")
        if lo is not None and v < lo:
            raise ValueError(f"below_min:{v}")
        if hi is not None and v > hi:
            raise ValueError(f"above_max:{v}")

    def _validate(self, s, v):
        if v is None:
            raise ValueError("no_value")
        if isinstance(v, str):
            if v.startswith("ERR:"):
                raise ValueError(v)
            if s["type"] == "string":
                return v
            raise ValueError(f"expected_{s['type']}:str")
        if isinstance(v, (list, tuple)):
            return self._validate_array(s, v)
        t = s["type"]
        if t == "float":
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise ValueError(f"expected_float:{type(v).__name__}")
            if isinstance(v, int) and not s.get("allow_int_for_float", True):
                raise ValueError("expected_float:int_disallowed")
            f = float(v)
            if s.get("reject_non_finite", True) and not math.isfinite(f):
                raise ValueError("non_finite")
            self._range_check(s, f)
            return f
        if t == "int":
            if isinstance(v, bool) or not isinstance(v, int):
                raise ValueError(f"expected_int:{type(v).__name__}")
            self._range_check(s, v)
            return v
        if t == "bool":
            if not isinstance(v, bool):
                raise ValueError(f"expected_bool:{type(v).__name__}")
            return v
        raise ValueError(f"unknown_type:{t}")

    def _validate_array(self, s, v):
        t = s["type"]
        out = []
        for x in v:
            if t == "float":
                if isinstance(x, bool) or not isinstance(x, (int, float)):
                    raise ValueError(f"expected_float_elem:{type(x).__name__}")
                if isinstance(x, int) and not s.get("allow_int_for_float", True):
                    raise ValueError("expected_float_elem:int_disallowed")
                f = float(x)
                if s.get("reject_non_finite", True) and not math.isfinite(f):
                    raise ValueError("non_finite_elem")
                self._range_check(s, f)
                out.append(f)
            elif t == "int":
                if isinstance(x, bool) or not isinstance(x, int):
                    raise ValueError(f"expected_int_elem:{type(x).__name__}")
                self._range_check(s, x)
                out.append(x)
            elif t == "bool":
                if not isinstance(x, bool):
                    raise ValueError(f"expected_bool_elem:{type(x).__name__}")
                out.append(x)
            elif t == "string":
                if not isinstance(x, str):
                    raise ValueError(f"expected_string_elem:{type(x).__name__}")
                out.append(x)
            else:
                raise ValueError(f"unknown_type_array:{t}")
        return out

    def _coerce_legacy(self, s, v):
        if v is None:
            raise ValueError("no_value")
        if isinstance(v, (list, tuple)):
            if s["type"] == "int":
                return [_to_int_checked(x) for x in v]
            return [_to_finite_float(x) for x in v]
        if isinstance(v, str):
            if v.startswith("ERR:"):
                raise ValueError(v)
            if s["type"] == "float":
                return _to_finite_float(v)
            if s["type"] == "int":
                return _to_int_checked(v)
            if s["type"] == "bool":
                return _to_bool_from_str(v)
            return v
        if s["type"] == "float":
            return _to_finite_float(v)
        if s["type"] == "int":
            return _to_int_checked(v)
        if s["type"] == "bool":
            return _to_bool_from_num(v)
        return str(v)

    def _get(self, name, raw):
        s = self.sig[name]
        v = raw.get(s["addr"])
        strict = s.get("strict")
        if strict is None:
            strict = self.strict_types
        try:
            result = self._validate(s, v) if strict else self._coerce_legacy(s, v)
        except (TypeError, ValueError) as e:
            reason = str(e.args[0]) if e.args else "bad_value"
            self._bump(name, reason)
            now = time.monotonic()
            if now - self._warned.get(name, 0.0) > self._warn_cooldown:
                self._warned[name] = now
                self.log.warning(
                    "signal_rejected",
                    extra={
                        "signal": name,
                        "addr": s["addr"],
                        "value": repr(v),
                        "reason": reason,
                        "counts": dict(self._stats[name]),
                    })
            return None
        self._stats[name]["good"] += 1
        return result
