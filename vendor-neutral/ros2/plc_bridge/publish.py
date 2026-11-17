import math
import time

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from safety import ESTOP_TOPIC, resolve_estop
from plc_bridge.constants import CAST, MSG_TYPES, ACT_ANG_DEADBAND_DEG, ACT_ANG_REFRESH_S


class PlcRosBridgePublish:
    def _warn_estop_unreadable(self, signal):
        now = time.monotonic()
        if now - self._warned.get("estop_unreadable", 0.0) > self._warn_cooldown:
            self._warned["estop_unreadable"] = now
            self.log.error(
                "estop_unreadable_failsafe_active",
                extra={"signal": signal})

    def _publish(self, vals, t_rs=None, t_rd=None):
        for p, pub in self.pubs:
            v = vals.get(p.signal)
            if p.topic == ESTOP_TOPIC:
                if v is None:
                    self._warn_estop_unreadable(p.signal)
                v = resolve_estop(v)
            if v is None:
                continue
            cast_v = CAST[p.type](v)
            key = p.topic
            if self._last_pub.get(key) == cast_v:
                continue
            self._last_pub[key] = cast_v
            msg = MSG_TYPES[p.type](data=cast_v)
            if pub is self.pub_estop:
                self._pub_locked(pub, msg)
            else:
                pub.publish(msg)

        if self.pub_cmd_ang:
            ang_deg = []
            for n in self.cmd_ang:
                v = vals.get(n)
                if v is None:
                    return
                if isinstance(v, (list, tuple)):
                    ang_deg.extend(v)
                else:
                    ang_deg.append(v)
            # PLC stores degrees; physics_twin/JointMapper expects radians
            ang = [math.radians(d) for d in ang_deg]
            self._last_cmd_ang = list(ang)

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(self.joints)
            js.position = ang
            if self._probe is not None and t_rs is not None:
                hb = vals.get(self.hb_counter) if self.hb_counter else None
                hb_tag = int(hb) if isinstance(hb, int) else 0
                js.header.frame_id = f"p;{hb_tag};{t_rs};{t_rd}"
                self._probe_sent += 1
            self.pub_cmd_ang.publish(js)

    def _pub_locked(self, pub, msg):
        got = self._estop_lock.acquire(timeout=0.2)
        try:
            pub.publish(msg)
        finally:
            if got:
                self._estop_lock.release()

    def _publish_estop(self, value):
        self._pub_locked(self.pub_estop, Bool(data=bool(value)))

    def _act_ang_needs_write(self, ang_deg, now):
        last = self._last_written_act_ang
        if last is None:
            return True
        if now - self._last_act_write_t >= ACT_ANG_REFRESH_S:
            return True
        return any(abs(n - o) > ACT_ANG_DEADBAND_DEG
                   for n, o in zip(ang_deg, last))

    def _act_ang_mark_written(self, ang_deg, now):
        self._last_written_act_ang = ang_deg
        self._last_act_write_t = now

    def _act_ang_write_failed(self):
        self._last_written_act_ang = None
        self._last_act_write_t = 0.0

    def _deadline_stats(self):
        out = {"deadline_miss": self._deadline_miss,
               "max_overrun_ms": round(self._max_overrun_ms, 1)}
        self._max_overrun_ms = 0.0
        return out

    def _probe_stats(self):
        out = {}
        p = self._probe
        if p is not None:
            out["probe_sent"] = self._probe_sent
            out["probe_attempted"] = p.attempted
            out["probe_rows"] = p.rows
            out["probe_rejected"] = p.rejected
            out["probe_order_violations"] = p.order_violations
        total_rej = sum(c.get("rejected_total", 0) for c in self._stats.values())
        if total_rej > 0:
            out["signal_rejections_total"] = total_rej
            out["signal_rejections"] = {
                name: {k: v for k, v in c.items() if k != "good"}
                for name, c in self._stats.items()
                if c.get("rejected_total", 0) > 0
            }
        return out

    def _measured_rate_hz(self):
        now = time.monotonic()
        dt = now - self._stats_t0
        self._stats_t0 = now
        return round(self._stats_every / dt, 1) if dt > 0 else 0.0
