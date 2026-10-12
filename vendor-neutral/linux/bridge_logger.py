#!/usr/bin/env python3
"""Structured logging for the PLC <-> ROS 2 bridge.

Two rotating file outputs plus optional console:
  - human-readable log file
  - JSON Lines file for machine analysis

Falls back to plain console logging if config is missing.
"""
import json
import logging
import logging.handlers
import os
import sys
import time


_SKIP_FIELDS = {
    "args", "asctime", "created", "exc_info", "exc_text",
    "filename", "funcName", "levelname", "levelno",
    "lineno", "module", "msecs", "message", "msg", "name",
    "pathname", "process", "processName", "relativeCreated",
    "stack_info", "thread", "threadName",
    "taskName",
}


class JsonFormatter(logging.Formatter):
    def format(self, record):
        payload = {
            "ts": time.strftime(
                "%Y-%m-%dT%H:%M:%S", time.gmtime(record.created)
            ) + f".{int(record.msecs):03d}Z",
            "level": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        for k, v in record.__dict__.items():
            if k in _SKIP_FIELDS:
                continue
            try:
                json.dumps(v)
                payload[k] = v
            except (TypeError, ValueError):
                payload[k] = repr(v)
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload, ensure_ascii=False)


class HumanFormatter(logging.Formatter):
    def format(self, record):
        base = super().format(record)
        extras = []
        for k, v in record.__dict__.items():
            if k in _SKIP_FIELDS:
                continue
            extras.append(f"{k}={v}")
        if extras:
            base += " | " + " ".join(extras)
        return base


class RosBridgeHandler(logging.Handler):
    """Forward log records to a ROS 2 logger."""

    def __init__(self, ros_logger):
        super().__init__()
        self.ros_logger = ros_logger

    def emit(self, record):
        try:
            msg = self.format(record)
        except Exception:
            self.handleError(record)
            return
        if record.levelno >= logging.ERROR:
            self.ros_logger.error(msg)
        elif record.levelno >= logging.WARNING:
            self.ros_logger.warn(msg)
        elif record.levelno >= logging.INFO:
            self.ros_logger.info(msg)
        else:
            self.ros_logger.debug(msg)


def setup_logging(cfg, logger_name="plc_ros_bridge", ros_logger=None):
    """Configure logging from the top-level YAML dict.

    Reads cfg.get("logging", {}):
      level        : DEBUG | INFO | WARNING | ERROR  (default INFO)
      console      : bool                            (default True)
      file         : path to human-readable log file (optional)
      json_file    : path to JSON Lines log file     (optional)
      max_bytes    : rotation size                   (default 10 MB)
      backup_count : number of rotated files         (default 5)
      to_ros       : forward to ROS 2 logger         (default True)
    """
    log_cfg = cfg.get("logging", {}) or {}
    level_name = str(log_cfg.get("level", "INFO")).upper()
    level = getattr(logging, level_name, logging.INFO)

    logger = logging.getLogger(logger_name)
    logger.setLevel(level)
    logger.propagate = False

    for h in list(logger.handlers):
        logger.removeHandler(h)

    # ---- console ----
    if log_cfg.get("console", True):
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(level)
        ch.setFormatter(HumanFormatter(
            "%(asctime)s [%(levelname)s] %(message)s",
            datefmt="%H:%M:%S",
        ))
        logger.addHandler(ch)

    # ---- human-readable rotating file ----
    file_path = log_cfg.get("file")
    if file_path:
        try:
            d = os.path.dirname(os.path.expanduser(file_path))
            if d:
                os.makedirs(d, exist_ok=True)
            fh = logging.handlers.RotatingFileHandler(
                os.path.expanduser(file_path),
                maxBytes=int(log_cfg.get("max_bytes", 10 * 1024 * 1024)),
                backupCount=int(log_cfg.get("backup_count", 5)),
            )
            fh.setLevel(level)
            fh.setFormatter(HumanFormatter(
                "%(asctime)s [%(levelname)s] %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            ))
            logger.addHandler(fh)
        except OSError as e:
            print(f"[bridge_logger] cannot open {file_path}: {e}",
                  file=sys.stderr)

    # ---- JSON Lines rotating file ----
    json_path = log_cfg.get("json_file")
    if json_path:
        try:
            d = os.path.dirname(os.path.expanduser(json_path))
            if d:
                os.makedirs(d, exist_ok=True)
            jh = logging.handlers.RotatingFileHandler(
                os.path.expanduser(json_path),
                maxBytes=int(log_cfg.get("max_bytes", 10 * 1024 * 1024)),
                backupCount=int(log_cfg.get("backup_count", 5)),
            )
            jh.setLevel(level)
            jh.setFormatter(JsonFormatter())
            logger.addHandler(jh)
        except OSError as e:
            print(f"[bridge_logger] cannot open {json_path}: {e}",
                  file=sys.stderr)

    # ---- forward to ROS 2 logger ----
    if ros_logger is not None and log_cfg.get("to_ros", True):
        rh = RosBridgeHandler(ros_logger)
        rh.setLevel(level)
        rh.setFormatter(HumanFormatter("%(message)s"))
        logger.addHandler(rh)

    return logger