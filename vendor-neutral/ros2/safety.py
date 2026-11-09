"""Pure safety helpers (no ROS imports, so they are unit-testable anywhere)."""

ESTOP_TOPIC = "/plc/emergency_stop"


def resolve_estop(value):
    """Fail-safe interpretation of the PLC E-stop signal.

    An unreadable value (None) means we cannot prove the E-stop is released,
    so it is treated as ACTIVE (True). Only an explicit False releases it.
    """
    if value is None:
        return True
    return bool(value)
