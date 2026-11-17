import math
import time

import numpy as np

from trajectory.node_state import IDLE, RUNNING, STOPPING


class TrajectoryTwinServo:
    def servo_tick(self):
        now = time.monotonic()

        with self.lock:
            enable        = self._enable
            estop         = self._estop
            cmd           = list(self._cmd_deg)
            cmd_ok        = self._cmd_ok
            last_cmd_t    = self._last_cmd_t
            actual        = self._actual.copy()
            has_actual    = self._has_actual
            last_js_t     = self._last_js_t
            last_applied  = self._last_applied_cmd_deg
            tag           = self._cmd_tag
            tag_t         = self._cmd_tag_t
            test_mode     = self._test_mode

        # Planner selection: only switch while IDLE; if the wanted planner
        # differs while moving, brake first (Ruckig/pass-through never mix).
        desired = self._desired_planner(test_mode)
        mode_mismatch = desired is not self.planner
        if mode_mismatch and self._state == IDLE:
            self.planner = desired
            mode_mismatch = False
            self.log.info("planner_selected", extra={
                "planner": type(desired).__name__, "test_mode": test_mode})

        fresh = has_actual and (now - last_js_t) <= 0.5

        if (now - last_cmd_t) > 0.5:
            if cmd_ok and not self._cmd_timeout_logged:
                self.log.warning("cmd_timeout", extra={
                    "last_cmd_age_s": round(now - last_cmd_t, 2)})
                self._cmd_timeout_logged = True
            cmd_ok = False
        else:
            self._cmd_timeout_logged = False

        if not fresh and has_actual:
            if not self._no_feedback_logged:
                self.log.error("no_feedback", extra={
                    "last_js_age_s": round(now - last_js_t, 2)})
                self._no_feedback_logged = True
        else:
            self._no_feedback_logged = False

        want_run = enable and (not estop) and fresh and cmd_ok \
            and not mode_mismatch

        np.copyto(self._ctrl_actual, actual[self.ctrl_indices])

        # ---- run path -------------------------------------------------
        if want_run and self._state != STOPPING:
            target = self._ctrl_actual.copy()
            valid = True
            for i, j in enumerate(self.controlled_joints):
                deg = cmd[i]
                lo, hi = self.limits_deg[j]
                if not math.isfinite(deg) or deg < lo or deg > hi:
                    valid = False
                    self._bump(j, "below_min" if math.isfinite(deg) and deg < lo
                               else "above_max" if math.isfinite(deg)
                               else "non_finite_at_servo")
                    break
                target[i] = math.radians(deg)

            if not valid:
                if not self._bad_cmd_logged:
                    self._bad_cmd_logged = True
                    self.log.warning("bad_command", extra={"cmd_deg": cmd})
                if self._state == RUNNING:
                    self._begin_brake("bad command")
            else:
                self._bad_cmd_logged = False
                if self._state == IDLE:
                    self.planner.reset(self._ctrl_actual)
                    self.log.info("state_changed",
                                  extra={"from": "IDLE", "to": "RUNNING"})
                    self._state = RUNNING

                needs_target_update = (
                    last_applied is None or cmd != last_applied
                )
                if needs_target_update:
                    if not self.planner.set_target(target):
                        self.log.warning("invalid_target",
                                         extra={"target_rad": target.tolist()})
                        self._begin_brake("invalid target")
                    else:
                        with self.lock:
                            self._last_applied_cmd_deg = list(cmd)

                self.planner.sync_state(self._ctrl_actual)

                if not self.planner.stopped:
                    sp = self.planner.step()
                    if sp is not None:
                        self._publish(sp, tag, tag_t)

        # ---- brake path -----------------------------------------------
        elif (not want_run) and self._state == RUNNING:
            reason = ("estop"      if estop
                      else "mode change" if mode_mismatch
                      else "disable" if not enable
                      else "no feedback" if not fresh
                      else "cmd timeout")
            self._begin_brake(reason)

        # ---- stopping path --------------------------------------------
        if self._state == STOPPING:
            self.planner.sync_state(self._ctrl_actual)
            sp = self.planner.step()
            if sp is not None:
                self._publish(sp, tag, tag_t)
            if self.planner.stopped:
                self.log.info("state_changed",
                              extra={"from": "STOPPING", "to": "IDLE"})
                self._state = IDLE
                with self.lock:
                    self._last_applied_cmd_deg = None

    def _publish(self, ctrl_positions, tag, tag_t):
        # v4.0: refuse to publish non-finite to /servo_setpoints.
        # Tag is preserved (not consumed) so the next valid publish can
        # still attach it if still relevant — the latency probe will
        # reject stale chains by age, not here.
        try:
            positions_list = [float(x) for x in ctrl_positions]
        except (TypeError, ValueError):
            self._bump("__publish__", "bad_type")
            self._warn("__publish__", "non-numeric ctrl_positions")
            return
        if not all(math.isfinite(x) for x in positions_list):
            self._bump("__publish__", "non_finite")
            self._warn("__publish__", "non-finite ctrl_positions")
            return

        pos = self._msg.position
        for i, idx in enumerate(self.ctrl_indices):
            pos[idx] = positions_list[i]

        self._msg.header.stamp = self.get_clock().now().to_msg()
        if tag:
            self._msg.header.frame_id = f"{tag};{time.monotonic_ns()}"
        else:
            self._msg.header.frame_id = ""
        self.setpoint_pub.publish(self._msg)

        if tag:
            with self.lock:
                if self._cmd_tag_t == tag_t:
                    self._cmd_tag = ""
                    self._cmd_tag_t = 0.0

        now = time.monotonic()
        if now - self._last_setpoint_log >= 1.0:
            self._last_setpoint_log = now
            lines = [
                f"{'joint':<10}{'cmd_deg':>12}{'actual_deg':>14}{'diff_deg':>14}",
                "-" * 50,
            ]
            for i, j in enumerate(self.controlled_joints):
                idx = self.ctrl_indices[i]
                c = (math.degrees(float(self.planner._target_pos[i]))
                     if self.planner._has_target else None)
                a = math.degrees(float(self._actual[idx]))
                d = (c - a) if c is not None else None
                cs = f"{c:>12.3f}" if c is not None else f"{'--':>12}"
                as_ = f"{a:>14.3f}"
                ds = f"{d:>14.3f}" if d is not None else f"{'--':>14}"
                lines.append(f"{j:<10}{cs}{as_}{ds}")

            # v4.0: rejection summary
            total_rej = sum(c.get("rejected_total", 0)
                            for c in self._stats.values())
            if total_rej > 0:
                lines.append("-" * 50)
                lines.append(f"rejections_total: {total_rej}")
                for name, counts in sorted(self._stats.items()):
                    rej = {k: v for k, v in counts.items()
                           if k != "rejected_total" and v > 0}
                    if rej or counts.get("rejected_total", 0) > 0:
                        lines.append(f"  {name}: {dict(counts)}")

            self.log.info("trajectory_status",
                          extra={"table": "\n".join(lines)})

    def _begin_brake(self, reason):
        if self._state == RUNNING:
            self.log.warning("state_changed", extra={
                "from": "RUNNING", "to": "STOPPING", "reason": reason})
        self.planner.start_brake()
        self._state = STOPPING
