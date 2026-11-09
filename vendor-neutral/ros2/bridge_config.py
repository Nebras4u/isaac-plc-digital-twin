"""Declarative configuration schema for plc_ros_bridge.

v2.5 — Snap7 support:
  * New DriverName 'snap7'.
  * PlcConfig: rack/slot for S7.
  * SignalConfig: db_num/byte_offset/bit_offset for absolute S7 addressing.
  * SignalConfig.s7_type: explicit S7 primitive type
    ('bool' | 'int16' | 'int32' | 'lreal'), overriding the generic YAML
    type when decoding. Required for distinguishing Int vs DInt, and
    for picking the right snap7.util decoder.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Tuple

import yaml
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    field_validator,
    model_validator,
)

CURRENT_SCHEMA_VERSION = "2.3"

SignalType = Literal["bool", "int", "float", "string"]
SignalDir = Literal["read", "write"]
DriverName = Literal["tcp_json", "opcua", "ads", "snap7"]
S7Type = Literal["bool", "int16", "int32", "lreal"]


class ConfigError(ValueError):
    """Raised when a config file fails validation or migration."""


class SignalConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    addr: str = Field(min_length=1, max_length=256)
    type: SignalType = "float"
    dir: SignalDir = "read"

    length: int = Field(default=1, ge=1, le=65535)

    min: Optional[float] = None
    max: Optional[float] = None
    strict: Optional[bool] = None
    reject_non_finite: bool = True
    allow_int_for_float: bool = True

    # --- v2.5: Snap7 absolute addressing ---
    db_num: Optional[int] = Field(default=None, ge=1, le=65535)
    byte_offset: Optional[int] = Field(default=None, ge=0, le=65535)
    bit_offset: Optional[int] = Field(default=0, ge=0, le=7)

    # --- v2.5: Snap7 primitive type (overrides YAML 'type' at decode) ---
    s7_type: Optional[S7Type] = None

    @field_validator("addr")
    @classmethod
    def _addr_no_path_traversal(cls, v: str) -> str:
        if any(c in v for c in ("\x00", "\n", "\r")):
            raise ValueError("addr contains control characters")
        if ".." in v:
            raise ValueError("addr contains '..' (path traversal pattern)")
        return v

    @model_validator(mode="after")
    def _range_sanity(self) -> "SignalConfig":
        if self.min is not None and not math.isfinite(self.min):
            raise ValueError("min must be finite")
        if self.max is not None and not math.isfinite(self.max):
            raise ValueError("max must be finite")
        if self.min is not None and self.max is not None and self.min > self.max:
            raise ValueError(f"min ({self.min}) > max ({self.max})")
        if (self.min is not None or self.max is not None) \
                and self.type not in ("int", "float"):
            raise ValueError(
                f"min/max only valid for int/float signals, not {self.type!r}"
            )
        if self.type == "bool" and self.byte_offset is not None:
            if self.bit_offset is None or not (0 <= self.bit_offset <= 7):
                raise ValueError("bool signal requires bit_offset in 0..7")

        # --- v2.5: s7_type consistency checks ---
        if self.s7_type is not None:
            if self.s7_type == "bool" and self.type != "bool":
                raise ValueError(
                    f"s7_type='bool' requires type='bool', got type='{self.type}'"
                )
            if self.s7_type in ("int16", "int32") and self.type != "int":
                raise ValueError(
                    f"s7_type='{self.s7_type}' requires type='int', "
                    f"got type='{self.type}'"
                )
            if self.s7_type == "lreal" and self.type != "float":
                raise ValueError(
                    f"s7_type='lreal' requires type='float', "
                    f"got type='{self.type}'"
                )
        return self


class PlcConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    driver: DriverName

    host: Optional[str] = None
    port: Optional[int] = Field(default=None, ge=1, le=65535)
    timeout: float = Field(default=1.0, gt=0, le=60)

    endpoint: Optional[str] = None
    user: Optional[str] = None
    password: Optional[str] = None

    # TwinCAT 3 ADS
    ams_net_id: Optional[str] = None
    ams_port: int = Field(default=851, ge=1, le=65535)
    local_ams_net_id: Optional[str] = None

    # --- v2.5: Siemens S7 (Snap7) ---
    rack: int = Field(default=0, ge=0, le=7)
    slot: int = Field(default=1, ge=0, le=31)

    @model_validator(mode="after")
    def _driver_specific_required(self) -> "PlcConfig":
        if self.driver in ("tcp_json", "snap7"):
            if not self.host:
                raise ValueError(f"{self.driver} driver requires 'plc.host'")
        if self.driver == "opcua":
            if not self.endpoint:
                raise ValueError("opcua driver requires 'plc.endpoint'")
        if self.driver == "ads":
            if not self.ams_net_id:
                raise ValueError("ads driver requires 'plc.ams_net_id'")
            if not self.host:
                raise ValueError("ads driver requires 'plc.host'")
            if not self.local_ams_net_id:
                raise ValueError("ads driver requires 'plc.local_ams_net_id'")
        return self


class VarsConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    ns: str = ""
    db: str = ""
    opcua_addr_template: str = '{ns}."{raw}"'


class RobotConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    joints: List[str] = Field(default_factory=list)
    cmd_ang: List[str] = Field(default_factory=list)
    act_ang: List[str] = Field(default_factory=list)
    cmd_ack: Optional[str] = None
    feedback_valid: Optional[str] = None
    cmd_ang_topic: str = "/plc/cmd_ang"
    joint_states_topic: str = "/joint_states"


class HeartbeatConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    counter: Optional[str] = None
    ack: Optional[str] = None


class PublishConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    signal: str = Field(min_length=1)
    topic: str = Field(min_length=1)
    type: SignalType


class BridgeConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    schema_version: str = Field(default="1.0")
    plc: PlcConfig
    vars: VarsConfig = Field(default_factory=VarsConfig)
    signals: Dict[str, SignalConfig]
    robot: RobotConfig = Field(default_factory=RobotConfig)
    heartbeat: HeartbeatConfig = Field(default_factory=HeartbeatConfig)
    publish: List[PublishConfig] = Field(default_factory=list)

    poll_hz: float = Field(default=20.0, gt=0, le=1000)
    watchdog_timeout: float = Field(default=4.0, gt=0.1, le=60)
    backoff_max: float = Field(default=30.0, gt=0, le=600)
    warn_cooldown: float = Field(default=60.0, ge=0, le=3600)

    latency_probe: bool = False
    latency_probe_file: str = "latency_probe.csv"

    logging: Optional[Dict[str, Any]] = None

    strict_types: bool = True
    strict_type_handshake: bool = False

    estop_forwarding_required: bool = True

    @field_validator("signals")
    @classmethod
    def _signals_not_empty(
        cls, v: Dict[str, SignalConfig]
    ) -> Dict[str, SignalConfig]:
        if not v:
            raise ValueError("'signals' must be a non-empty mapping")
        return v

    @model_validator(mode="after")
    def _require_estop_forwarding(self) -> "BridgeConfig":
        if not self.estop_forwarding_required:
            return self
        if not any(
            p.topic == "/plc/emergency_stop" and p.type == "bool"
            for p in self.publish
        ):
            raise ValueError(
                "publish must forward the PLC E-stop signal to "
                "'/plc/emergency_stop'. Add:\n"
                "  publish:\n"
                "    - {signal: estop, topic: /plc/emergency_stop, type: bool}\n"
                "(or set 'estop_forwarding_required: false' for tests)"
            )
        return self

    @model_validator(mode="after")
    def _forbid_ruckig_bypass(self) -> "BridgeConfig":
        # CmdAng must reach the twin via trajectory_twin (Ruckig), which
        # listens on /plc/cmd_ang and is the only publisher of
        # /servo_setpoints.
        if self.robot.cmd_ang and self.robot.cmd_ang_topic == "/servo_setpoints":
            raise ValueError(
                "robot.cmd_ang_topic must not be '/servo_setpoints' "
                "(bypasses Ruckig). Use '/plc/cmd_ang'."
            )
        return self

    @model_validator(mode="after")
    def _validate_cross_references(self) -> "BridgeConfig":
        names = set(self.signals.keys())

        def _check(sig: str, location: str) -> None:
            if sig not in names:
                raise ValueError(
                    f"signal '{sig}' referenced in '{location}' "
                    f"is not defined in 'signals'"
                )

        for i, s in enumerate(self.robot.cmd_ang):
            _check(s, f"robot.cmd_ang[{i}]")
        for i, s in enumerate(self.robot.act_ang):
            _check(s, f"robot.act_ang[{i}]")
        if self.robot.cmd_ack:
            _check(self.robot.cmd_ack, "robot.cmd_ack")
        if self.robot.feedback_valid:
            _check(self.robot.feedback_valid, "robot.feedback_valid")
        if self.heartbeat.counter:
            _check(self.heartbeat.counter, "heartbeat.counter")
        if self.heartbeat.ack:
            _check(self.heartbeat.ack, "heartbeat.ack")
        for i, p in enumerate(self.publish):
            _check(p.signal, f"publish[{i}].signal")

        for i, s in enumerate(self.robot.cmd_ang):
            if self.signals[s].dir != "read":
                raise ValueError(
                    f"robot.cmd_ang[{i}] -> signal '{s}' must be dir='read'"
                )
        for i, s in enumerate(self.robot.act_ang):
            if self.signals[s].dir != "write":
                raise ValueError(
                    f"robot.act_ang[{i}] -> signal '{s}' must be dir='write'"
                )
        if self.heartbeat.ack and self.signals[self.heartbeat.ack].dir != "write":
            raise ValueError(
                f"heartbeat.ack -> signal '{self.heartbeat.ack}' must be dir='write'"
            )
        if self.robot.cmd_ack and self.signals[self.robot.cmd_ack].dir != "write":
            raise ValueError(
                f"robot.cmd_ack -> signal '{self.robot.cmd_ack}' must be dir='write'"
            )
        if (self.robot.feedback_valid
                and self.signals[self.robot.feedback_valid].dir != "write"):
            raise ValueError(
                f"robot.feedback_valid -> signal "
                f"'{self.robot.feedback_valid}' must be dir='write'"
            )
        for i, p in enumerate(self.publish):
            if self.signals[p.signal].dir != "read":
                raise ValueError(
                    f"publish[{i}].signal -> signal '{p.signal}' must be dir='read'"
                )

        for i, p in enumerate(self.publish):
            declared = self.signals[p.signal].type
            if p.type != declared:
                raise ValueError(
                    f"publish[{i}].type='{p.type}' does not match "
                    f"signals['{p.signal}'].type='{declared}'"
                )

        return self


_MIGRATIONS: Dict[str, Any] = {}


def _migrate(cfg: Dict[str, Any]) -> Dict[str, Any]:
    version = str(cfg.get("schema_version", "1.0"))
    while version != CURRENT_SCHEMA_VERSION:
        fn = _MIGRATIONS.get(version)
        if fn is None:
            if version in ("1.0", "2.0", "2.1", "2.2"):
                cfg = dict(cfg)
                cfg["schema_version"] = CURRENT_SCHEMA_VERSION
                version = CURRENT_SCHEMA_VERSION
                break
            raise ConfigError(
                f"no migration path from schema_version '{version}' "
                f"to '{CURRENT_SCHEMA_VERSION}'"
            )
        cfg = fn(cfg)
        version = str(cfg.get("schema_version", CURRENT_SCHEMA_VERSION))
    return cfg


def load_bridge_config(path: str | Path) -> Tuple[BridgeConfig, str]:
    path = Path(path)
    try:
        with path.open("rb") as f:
            raw_bytes = f.read()
        cfg_dict = yaml.safe_load(raw_bytes)
    except FileNotFoundError:
        raise ConfigError(f"config file not found: {path}")
    except yaml.YAMLError as e:
        raise ConfigError(f"invalid YAML in {path}: {e}")

    if not isinstance(cfg_dict, dict):
        raise ConfigError(f"top-level of {path} must be a mapping")

    migrated = _migrate(cfg_dict)

    selected_driver = migrated.get("driver")
    if selected_driver and selected_driver in migrated:
        driver_block = migrated.pop(selected_driver)
        merged = {**migrated.get("defaults", {}), **driver_block}
        migrated.update(merged)
        migrated.pop("defaults", None)
        migrated.pop("tcp_json", None)
        migrated.pop("ads", None)
        migrated.pop("opcua", None)
        migrated.pop("snap7", None)
        migrated.pop("driver", None)

    canonical = json.dumps(migrated, sort_keys=True, separators=(",", ":"))
    cfg_hash = hashlib.sha256(canonical.encode("utf-8")).hexdigest()

    try:
        cfg = BridgeConfig(**migrated)
    except Exception as e:
        raise ConfigError(f"config validation failed for {path}:\n{e}") from e

    if cfg.schema_version != CURRENT_SCHEMA_VERSION:
        raise ConfigError(
            f"schema_version '{cfg.schema_version}' did not reach "
            f"'{CURRENT_SCHEMA_VERSION}' after migration (bug)"
        )

    return cfg, cfg_hash
