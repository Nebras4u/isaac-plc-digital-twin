import math
import time

from plc_bridge.constants import JS_STALE_AGE_S


class PlcRosBridgeTick:
    def on_joint_states(self, msg):
        with self._js_lock:
            if self._probe is not None:
                tag = msg.header.frame_id
                if tag.startswith("p;"):
                    self._js_tag = tag
                    self._js_recv_ns = time.monotonic_ns()
            for name, pos in zip(msg.name, msg.position):
                i = self.joint_idx.get(name)
                if i is not None:
                    self._js_rad[i] = float(pos)
            self._last_js_t = time.monotonic()
            self._js_ok = True

    def tick(self):
        if not self.connected and not self._try_connect():
            return
        tick_id = int(time.monotonic() * 1000) % 1000000
        t_rs = time.monotonic_ns()
        try:
            writes_to_send = self._pending_writes
            ack_to_send = self._pending_ack
            self._pending_writes = {}
            self._pending_ack = None

            probe_tag, probe_recv = "", 0
            act_written = False
            with self._js_lock:
                js_fresh = (
                    self._js_ok
                    and self._last_js_t > 0
                    and (time.monotonic() - self._last_js_t) <= JS_STALE_AGE_S)
                if self.act_ang:
                    if js_fresh:
                        ang_deg = [math.degrees(float(x)) for x in self._js_rad]
                        if not all(math.isfinite(a) for a in ang_deg):
                            self._bump("__act_ang__", "non_finite")
                            self.log.error(
                                "joint_angles_non_finite",
                                extra={"ang": ang_deg})
                            js_fresh = False
                    if js_fresh:
                        probe_tag, probe_recv = self._js_tag, self._js_recv_ns
                        needs_write = (self._act_ang_needs_write(ang_deg, time.monotonic())
                            or (self._probe is not None and bool(probe_tag)))
                        if needs_write:
                            if len(self.act_ang) == 1:
                                sig = self.act_ang[0]
                                writes_to_send[self.sig[sig]["addr"]] = ang_deg
                            else:
                                for sig, deg in zip(self.act_ang, ang_deg):
                                    writes_to_send[self.sig[sig]["addr"]] = deg
                            self._act_ang_mark_written(ang_deg, time.monotonic())
                            act_written = True
                            self._js_tag = ""
                        else:
                            act_written = False
                        if self.cmd_ack:
                            writes_to_send[self.sig[self.cmd_ack]["addr"]] = True
                        if self.feedback_valid:
                            writes_to_send[self.sig[self.feedback_valid]["addr"]] = True
                    else:
                        if self.cmd_ack:
                            writes_to_send[self.sig[self.cmd_ack]["addr"]] = False
                        if self.feedback_valid:
                            writes_to_send[self.sig[self.feedback_valid]["addr"]] = False

            if self.driver_name == "tcp_json":
                raw = self.driver.read(self._read_addrs)
                failed = self.driver.write(writes_to_send) if writes_to_send else []
            elif hasattr(self.driver, "sync"):
                raw, failed = self.driver.sync(self._read_addrs, writes_to_send)
            else:
                raw = self.driver.read(self._read_addrs)
                failed = self.driver.write(writes_to_send) if writes_to_send else []

            t_rd = time.monotonic_ns()
            vals = {n: self._get(n, raw) for n in self.read_names}
            read_ms = (t_rd - t_rs) / 1e6
            sock_ms = getattr(self.driver, "_last_sock_ms", 0.0)
            gap_ms = max(0.0, read_ms - sock_ms)

            if read_ms > 200.0:
                self._slow_count += 1
                self._slow_sock_sum_ms += sock_ms
                self._slow_gap_sum_ms += gap_ms
                self.log.warning(
                    "slow_read",
                    extra={
                        "tick_id": tick_id,
                        "read_ms": round(read_ms, 1),
                        "sock_ms": round(sock_ms, 2),
                        "gap_ms": round(gap_ms, 1),
                    })

            self._last_tick_ok = time.monotonic()
            if self._watchdog_fired:
                self._last_pub.clear()
                self._watchdog_fired = False

            # Auto-start measurement after warm-up ticks
            if (not self._measure_active
                    and self._tick_count >= self._measure_warmup_ticks):
                self._measure_active = True
                self._measure_start_ns = time.monotonic_ns()
                self._measure_samples = 0
                self.log.info(
                    "measurement_started",
                    extra={"trigger": "warmup_done",
                           "tick": self._tick_count,
                           "t_start_ns": self._measure_start_ns})

            if writes_to_send:
                if not failed:
                    if (self._measure_active
                            and act_written
                            and probe_tag
                            and self._probe is not None):
                        hb_val = (vals.get(self.hb_counter)
                                  if self.hb_counter else None)
                        hb_out = int(hb_val) if isinstance(hb_val, int) else 0
                        ok = self._probe.record(
                            probe_tag, probe_recv, t_rd, hb_out)
                        if ok:
                            self._measure_samples += 1
                    if ack_to_send is not None:
                        self._last_ack = ack_to_send
                else:
                    self._act_ang_write_failed()
                    for a in failed:
                        self.log.warning(
                            "write_failed",
                            extra={"tick_id": tick_id, "addr": a})

            self._publish(vals, t_rs, t_rd)

            if self.hb_counter and self.hb_ack:
                hb = vals.get(self.hb_counter)
                if hb is not None and hb != self._last_ack:
                    self._pending_writes[self.sig[self.hb_ack]["addr"]] = int(hb)
                    self._pending_ack = int(hb)

            self._tick_count += 1
            if self._tick_count % self._stats_every == 0:
                avg_sock = (self._slow_sock_sum_ms / self._slow_count
                            if self._slow_count else 0.0)
                avg_gap = (self._slow_gap_sum_ms / self._slow_count
                           if self._slow_count else 0.0)
                self.log.info(
                    "tick_stats",
                    extra={
                        "ticks": self._tick_count,
                        "rate_hz": self._measured_rate_hz(),
                        **self._probe_stats(),
                        **self._deadline_stats(),
                        "measure_active": self._measure_active,
                        "measure_samples": self._measure_samples,
                        "slow": self._slow_count,
                        "slow_pct": round(
                            100.0 * self._slow_count / self._tick_count, 2),
                        "avg_slow_sock_ms": round(avg_sock, 2),
                        "avg_slow_gap_ms": round(avg_gap, 1),
                    })
                self._slow_count = 0
                self._slow_sock_sum_ms = 0.0
                self._slow_gap_sum_ms = 0.0

            self._last_tick_end_ns = time.monotonic_ns()

        except Exception as e:
            self.log.error(
                "plc_link_error",
                extra={"tick_id": tick_id, "error": str(e),
                       "driver": self.driver_name})
            self.driver.close()
            self._set_connected(False)
            self._last_try = 0.0
