import math
import time


class TrajectoryTwinCallbacks:
    def _bump(self, name, reason):
        key = reason.split(":", 1)[0] if isinstance(reason, str) else str(reason)
        if not key:
            key = "bad_value"
        self._stats[name][key] += 1
        self._stats[name]["rejected_total"] += 1

    def _warn(self, name, message):
        now = time.monotonic()
        if now - self._warned.get(name, 0.0) > self._warn_cooldown:
            self._warned[name] = now
            counts = dict(self._stats.get(name, {}))
            try:
                self.log.warning(message, extra={"counts": counts})
            except Exception:
                pass

    def on_js(self, msg):
        if len(msg.name) != len(msg.position):
            self._bump("__joint_states__", "length_mismatch")
            self._warn("__joint_states__",
                       f"length mismatch: name={len(msg.name)} "
                       f"pos={len(msg.position)}")
            return

        valid_count = 0
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                i = self.name_to_idx.get(n)
                if i is None:
                    self._bump(n, "unknown_joint")
                    self._warn(n, f"unknown joint in /joint_states: {n!r}")
                    continue
                try:
                    f = float(p)
                except (TypeError, ValueError):
                    self._bump(n, "bad_type")
                    self._warn(n, f"bad joint_states type: {type(p).__name__}")
                    continue
                if not math.isfinite(f):
                    self._bump(n, "non_finite")
                    self._warn(n, f"non-finite joint_states[{n}]: {f!r}")
                    continue
                self._actual[i] = f
                valid_count += 1
            if valid_count > 0:
                self._has_actual = True
                self._last_js_t = time.monotonic()

    def on_cmd(self, msg):
        n = self.ctrl_dof
        if len(msg.position) < n:
            self._bump("__cmd__", f"short_cmd:{len(msg.position)}<{n}")
            self._warn("__cmd__",
                       f"cmd too short: {len(msg.position)} < {n}")
            return

        new_cmd = []
        for i, x in enumerate(msg.position[:n]):
            jname = self.controlled_joints[i]
            try:
                # /plc/cmd_ang carries RADIANS (bridge contract, same as
                # /joint_states); everything below this point is in degrees.
                f = math.degrees(float(x))
            except (TypeError, ValueError):
                self._bump(jname, "bad_type")
                self._warn(jname, f"bad cmd type: {type(x).__name__}")
                return
            if not math.isfinite(f):
                self._bump(jname, "non_finite")
                self._warn(jname, f"non-finite cmd[{jname}]: {f!r}")
                return
            new_cmd.append(f)

        with self.lock:
            if new_cmd != self._last_received_cmd_deg:
                self._last_received_cmd_deg = list(new_cmd)
                self.log.info("cmd_received", extra={"cmd_deg": new_cmd})
            self._cmd_deg = new_cmd
            tag = msg.header.frame_id
            if tag.startswith("p;"):
                self._cmd_tag   = tag
                self._cmd_tag_t = time.monotonic()
            else:
                self._cmd_tag   = ""
                self._cmd_tag_t = 0.0
            self._cmd_ok = True
            self._last_cmd_t = time.monotonic()

    def on_enable(self, msg):
        with self.lock:
            new = bool(msg.data)
            if new != self._enable:
                self.log.info("enable_changed", extra={"enable": new})
            self._enable = new

    def on_test_mode(self, msg):
        with self.lock:
            new = bool(msg.data)
            if new != self._test_mode:
                self.log.info("test_mode_changed", extra={"test_mode": new})
            self._test_mode = new

    def _desired_planner(self, test_mode):
        if test_mode and self.planner_pass is not None:
            return self.planner_pass
        return self.planner_ruckig

    def on_estop(self, msg):
        with self.lock:
            new = bool(msg.data)
            if new != self._estop:
                if new:
                    self.log.error("estop_active")
                else:
                    self.log.info("estop_cleared")
            self._estop = new
