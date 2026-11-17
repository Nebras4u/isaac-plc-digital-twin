import time

from std_msgs.msg import Bool

from plc_bridge.constants import _HAS_DIAG, DiagnosticStatus


class PlcRosBridgeLifecycle:
    def _set_connected(self, state):
        if state == self.connected:
            return
        if state:
            self._last_tick_ok = time.monotonic()
            self._last_pub.clear()
            self._last_cmd_ang = None
            self._last_written_act_ang = None
            self._last_act_write_t = 0.0
            self._pending_writes = {}
            self._pending_ack = None
            self._js_tag = ""
            self._js_recv_ns = 0
            self.log.info(
                "plc_connected",
                extra={"driver": self.driver_name, "backoff_s": self._backoff})
            if self._estop_from_plc:
                self.log.info("plc_reconnected_estop_from_plc")
            else:
                self._publish_estop(False)
                self.log.info("plc_reconnected_estop_cleared")
        else:
            self.log.warning(
                "plc_disconnected",
                extra={"driver": self.driver_name})
            self._publish_estop(True)
            self._last_pub["/plc/emergency_stop"] = True
            self.log.warning("plc_lost_estop_published")
        self.connected = state
        self.pub_conn.publish(Bool(data=state))

    def _run_type_handshake(self):
        if not hasattr(self.driver, "get_server_types"):
            return True
        try:
            server_types = self.driver.get_server_types()
        except Exception as e:
            self.log.warning("type_handshake_failed", extra={"error": str(e)})
            return True
        if not server_types:
            self.log.info("type_handshake_unavailable",
                          extra={"driver": self.driver_name})
            return True
        mismatch = {}
        for name, s in self.sig.items():
            st = server_types.get(s["addr"])
            if st is not None and st != s["type"]:
                mismatch[name] = {"addr": s["addr"],
                                  "config": s["type"], "server": st}
        if not mismatch:
            self.log.info("type_handshake_ok",
                          extra={"checked": len(server_types)})
            return True
        self.log.error("config_type_mismatch", extra={"mismatch": mismatch})
        if getattr(self.cfg, "strict_type_handshake", False):
            self.log.fatal("type_handshake_aborting")
            return False
        return True

    def _try_connect(self):
        now = time.monotonic()
        if now - self._last_try < self._backoff:
            return False
        self._last_try = now
        try:
            self.driver.connect()
        except Exception as e:
            self.log.warning(
                "plc_connect_failed",
                extra={"error": str(e), "next_retry_in_s": self._backoff})
            self._set_connected(False)
            self._backoff = min(self._backoff * 2.0, self._backoff_max)
            return False
        if not self._run_type_handshake():
            try:
                self.driver.close()
            except Exception:
                pass
            self._backoff = min(self._backoff * 2.0, self._backoff_max)
            return False
        self._backoff = self._backoff_min
        self._last_ack = None
        self._set_connected(True)
        return True

    def _io_loop(self):
        next_t = time.monotonic()
        while not self._io_stop.is_set():
            try:
                self.tick()
            except Exception as e:
                try:
                    self.log.error("io_loop_error", extra={"error": str(e)})
                except Exception:
                    pass
            next_t += self._poll_period
            now = time.monotonic()
            delay = next_t - now
            if delay > 0:
                self._io_stop.wait(delay)
            else:
                self._deadline_miss += 1
                overrun_ms = -delay * 1000.0
                if overrun_ms > self._max_overrun_ms:
                    self._max_overrun_ms = overrun_ms
                next_t = now

    def _watchdog_loop(self):
        while not self._watchdog_stop.wait(0.5):
            try:
                self._check_watchdog()
            except Exception as e:
                try:
                    self.log.error("watchdog_error", extra={"error": str(e)})
                except Exception:
                    pass

    def _check_watchdog(self):
        now = time.monotonic()
        if self.connected and now - self._last_tick_ok > self._watchdog_timeout:
            stalled = round(now - self._last_tick_ok, 2)
            self.log.error("watchdog_tick_stalled",
                           extra={"stalled_s": stalled})
            self._publish_estop(True)
            self._last_pub["/plc/emergency_stop"] = True
            self._watchdog_fired = True

    def _diag(self, stat):
        if not _HAS_DIAG:
            return stat
        if self.connected:
            stat.summary(DiagnosticStatus.OK, "connected")
        else:
            stat.summary(DiagnosticStatus.ERROR, "disconnected")
        stat.add("driver", self.driver_name)
        stat.add("backoff_s", f"{self._backoff:.1f}")
        stat.add("last_tick_age_s",
                 f"{time.monotonic() - self._last_tick_ok:.2f}")
        stat.add("config_hash", self._cfg_hash[:12] + "…")
        stat.add("schema_version", self.cfg.schema_version)
        stat.add("strict_types", str(self.strict_types))
        stat.add("measure_active", str(self._measure_active))
        stat.add("measure_samples", str(self._measure_samples))
        total_rej = sum(c.get("rejected_total", 0) for c in self._stats.values())
        stat.add("signal_rejections_total", str(total_rej))
        return stat

    def destroy_node(self):
        self._io_stop.set()
        try:
            self._io_thread.join(timeout=2.0)
        except Exception:
            pass
        self._watchdog_stop.set()
        try:
            self._watchdog_thread.join(timeout=1.0)
        except Exception:
            pass
        if getattr(self, "_probe", None) is not None:
            self._probe.close()
        try:
            self.driver.close()
        except Exception:
            pass
        try:
            self.log.info("bridge_shutdown")
        except Exception:
            pass
        super().destroy_node()
