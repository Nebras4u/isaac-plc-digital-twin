"""Declarative configuration schema for plc_ros_bridge."""
from __future__ import annotations

import hashlib
import json
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

CURRENT_SCHEMA_VERSION = "2.1"

SignalType = Literal["bool", "int", "float", "string"]
SignalDir = Literal["read", "write"]
DriverName = Literal["tcp_json", "opcua", "plc_api"]


class ConfigError(ValueError):
    """Raised when a config file fails validation or migration."""


class SignalConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    addr: str = Field(min_length=1, max_length=256)
    type: SignalType = "float"
    dir: SignalDir = "read"

    @field_validator("addr")
    @classmethod
    def _addr_no_path_traversal(cls, v: str) -> str:
        if any(c in v for c in ("\x00", "\n", "\r")):
            raise ValueError("addr contains control characters")
        if ".." in v:
            raise ValueError("addr contains '..' (path traversal pattern)")
        return v


class PlcConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    driver: DriverName

    host: Optional[str] = None
    port: Optional[int] = Field(default=None, ge=1, le=65535)
    timeout: float = Field(default=1.0, gt=0, le=60)

    endpoint: Optional[str] = None
    user: Optional[str] = None
    password: Optional[str] = None

    instance: Optional[str] = None

    @model_validator(mode="after")
    def _driver_specific_required(self) -> "PlcConfig":
        if self.driver == "tcp_json":
            if not self.host:
                raise ValueError("tcp_json driver requires 'plc.host'")
        elif self.driver == "opcua":
            if not self.endpoint:
                raise ValueError("opcua driver requires 'plc.endpoint'")
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

    # Simulation / test escape hatch: disable the mandatory E-stop
    # forwarding rule. Production YAMLs must leave this True (default).
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
        """Fail-fast if the PLC E-stop is not forwarded to ROS.

        Without this rule, a YAML that omits the estop entry from
        `publish:` silently stops forwarding the real PLC E-stop to
        /plc/emergency_stop. The bridge would then only publish its
        own estop events (disconnect, watchdog), never the PLC value.

        Set estop_forwarding_required: false ONLY for unit tests or
        simulation without a PLC E-stop signal.
        """
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

        return self


_MIGRATIONS: Dict[str, Any] = {}


def _migrate(cfg: Dict[str, Any]) -> Dict[str, Any]:
    version = str(cfg.get("schema_version", "1.0"))
    while version != CURRENT_SCHEMA_VERSION:
        fn = _MIGRATIONS.get(version)
        if fn is None:
            if version in ("1.0", "2.0"):
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