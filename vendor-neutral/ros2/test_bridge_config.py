import pytest
from pathlib import Path
from bridge_config import (
    load_bridge_config, ConfigError, CURRENT_SCHEMA_VERSION,
)


def _write(tmp_path, content):
    """Write a YAML test config.

    Injects `estop_forwarding_required: false` unless the test
    explicitly sets it. Production YAMLs leave the rule on; test
    configs are too small to include the estop signal.
    """
    if "estop_forwarding_required" not in content:
        content = "estop_forwarding_required: false\n" + content
    p = tmp_path / "cfg.yaml"
    p.write_text(content)
    return p


# ---- happy path ----
def test_minimal_valid(tmp_path):
    cfg_path = _write(tmp_path, """
plc:
  driver: tcp_json
  host: 127.0.0.1
signals:
  cmd_ang: {addr: A, type: float, dir: read}
  act_ang: {addr: B, type: float, dir: write}
robot:
  cmd_ang: [cmd_ang]
  act_ang: [act_ang]
poll_hz: 50
""")
    cfg, h = load_bridge_config(cfg_path)
    assert cfg.schema_version == CURRENT_SCHEMA_VERSION
    assert cfg.poll_hz == 50
    assert len(h) == 64  # sha256 hex


def test_v1_migration(tmp_path):
    """Old config without schema_version must still load."""
    cfg_path = _write(tmp_path, """
plc:
  driver: tcp_json
  host: 127.0.0.1
signals:
  x: {addr: A}
""")
    cfg, _ = load_bridge_config(cfg_path)
    assert cfg.schema_version == CURRENT_SCHEMA_VERSION


# ---- structural failures ----
def test_missing_driver(tmp_path):
    cfg_path = _write(tmp_path, "signals: {x: {addr: A}}")
    with pytest.raises(ConfigError, match="plc"):
        load_bridge_config(cfg_path)


def test_unknown_field(tmp_path):
    cfg_path = _write(tmp_path, """
plc: {driver: tcp_json, host: h}
signals: {x: {addr: A, typo_field: 1}}
""")
    with pytest.raises(ConfigError, match="typo_field"):
        load_bridge_config(cfg_path)


def test_empty_signals(tmp_path):
    cfg_path = _write(tmp_path, """
plc: {driver: tcp_json, host: h}
signals: {}
""")
    with pytest.raises(ConfigError, match="non-empty"):
        load_bridge_config(cfg_path)


# ---- numeric ranges ----
@pytest.mark.parametrize("hz", [0, -1, 1001, 100000])
def test_poll_hz_out_of_range(tmp_path, hz):
    cfg_path = _write(tmp_path, f"""
plc: {{driver: tcp_json, host: h}}
signals: {{x: {{addr: A}}}}
poll_hz: {hz}
""")
    with pytest.raises(ConfigError, match="poll_hz"):
        load_bridge_config(cfg_path)


# ---- driver-specific ----
def test_tcpjson_requires_host(tmp_path):
    cfg_path = _write(tmp_path, """
plc: {driver: tcp_json}
signals: {x: {addr: A}}
""")
    with pytest.raises(ConfigError, match="host"):
        load_bridge_config(cfg_path)


def test_opcua_requires_endpoint(tmp_path):
    cfg_path = _write(tmp_path, """
plc: {driver: opcua}
signals: {x: {addr: A}}
""")
    with pytest.raises(ConfigError, match="endpoint"):
        load_bridge_config(cfg_path)


# ---- cross-reference with location ----
def test_cross_ref_includes_location(tmp_path):
    cfg_path = _write(tmp_path, """
plc: {driver: tcp_json, host: h}
signals: {real: {addr: A}}
robot:
  cmd_ang: [real, missing_sig]
""")
    with pytest.raises(ConfigError, match=r"robot\.cmd_ang\[1\]"):
        load_bridge_config(cfg_path)


def test_publish_cross_ref_location(tmp_path):
    cfg_path = _write(tmp_path, """
plc: {driver: tcp_json, host: h}
signals: {real: {addr: A}}
publish:
  - {signal: real, topic: /a, type: float}
  - {signal: missing, topic: /b, type: float}
""")
    with pytest.raises(ConfigError, match=r"publish\[1\]\.signal"):
        load_bridge_config(cfg_path)


# ---- direction sanity ----
def test_cmd_ang_must_be_read(tmp_path):
    cfg_path = _write(tmp_path, """
plc: {driver: tcp_json, host: h}
signals:
  cmd: {addr: A, dir: write}
robot:
  cmd_ang: [cmd]
""")
    with pytest.raises(ConfigError, match="dir='read'"):
        load_bridge_config(cfg_path)


def test_act_ang_must_be_write(tmp_path):
    cfg_path = _write(tmp_path, """
plc: {driver: tcp_json, host: h}
signals:
  act: {addr: A, dir: read}
robot:
  act_ang: [act]
""")
    with pytest.raises(ConfigError, match="dir='write'"):
        load_bridge_config(cfg_path)


# ---- addr safety ----
@pytest.mark.parametrize("addr", ["a\x00b", "a\nb", "a\rb", "a..b"])
def test_addr_rejects_control_chars(addr):
    from bridge_config import SignalConfig
    with pytest.raises(Exception):
        SignalConfig(addr=addr)


# ---- estop forwarding rule (schema v2.1) ----
def test_estop_forwarding_required(tmp_path):
    """A YAML without estop in publish: must be rejected (production)."""
    cfg_path = tmp_path / "cfg.yaml"
    cfg_path.write_text("""
estop_forwarding_required: true
plc: {driver: tcp_json, host: h}
signals:
  cmd_ang: {addr: A, dir: read}
  estop:   {addr: E, dir: read}
robot:
  cmd_ang: [cmd_ang]
publish:
  - {signal: cmd_ang, topic: /plc/cmd_ang, type: float}
""")
    with pytest.raises(ConfigError, match="E-stop"):
        load_bridge_config(cfg_path)


def test_estop_forwarding_optout(tmp_path):
    """estop_forwarding_required: false allows missing estop in publish."""
    cfg_path = _write(tmp_path, """
plc: {driver: tcp_json, host: h}
signals:
  cmd_ang: {addr: A, dir: read}
""")
    cfg, _ = load_bridge_config(cfg_path)
    assert cfg.estop_forwarding_required is False


def test_estop_forwarding_present(tmp_path):
    """A YAML with estop in publish: passes the rule."""
    cfg_path = tmp_path / "cfg.yaml"
    cfg_path.write_text("""
estop_forwarding_required: true
plc: {driver: tcp_json, host: h}
signals:
  cmd_ang: {addr: A, dir: read}
  estop:   {addr: E, dir: read}
robot:
  cmd_ang: [cmd_ang]
publish:
  - {signal: cmd_ang, topic: /plc/cmd_ang, type: float}
  - {signal: estop,   topic: /plc/emergency_stop, type: bool}
""")
    cfg, _ = load_bridge_config(cfg_path)
    assert cfg.estop_forwarding_required is True
    assert any(p.topic == "/plc/emergency_stop" for p in cfg.publish)