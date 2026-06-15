import math
import subprocess

import rospy
import roslib.message
from rospy import AnyMsg
from diagnostic_msgs.msg import DiagnosticStatus


def _position(msg):
    """Best-effort (x, y, z) from common stamped pose types; None if not pose-like."""
    p = getattr(msg, "pose", None)
    if p is not None:
        pos = getattr(p, "position", None)
        if pos is None and hasattr(p, "pose"):
            pos = p.pose.position
        if pos is not None:
            return (pos.x, pos.y, pos.z)
    t = getattr(msg, "transform", None)
    if t is not None:
        return (t.translation.x, t.translation.y, t.translation.z)
    return None


def _norm(v):
    return math.sqrt(sum(c * c for c in v))


class TopicMonitor(object):
    """General per-topic checker.

    Configured purely from a dict: topic + which checks + thresholds. Detects
    dropout / rate / freeze / jump / nan and reports one diagnostic task.
    dropout & rate work on ANY topic (rospy.AnyMsg, no type needed); freeze/jump/nan
    need msg_type so the position can be read.
    """

    def __init__(self, name, cfg):
        self.name = name
        self.topic = cfg["topic"]
        self.checks = set(cfg.get("checks", ["dropout", "rate"]))
        self.dropout_timeout = float(cfg.get("dropout_timeout_s", 0.5))
        self.min_rate = float(cfg.get("min_rate_hz", 0.0))
        self.freeze_window = float(cfg.get("freeze_window_s", 0.3))
        self.freeze_eps = float(cfg.get("freeze_eps_m", 1e-6))
        self.jump_max_step = float(cfg.get("jump_max_step_m", 0.3))
        # low rate is WARN by default; set rate_level: error to make it a hard fault (e.g. VRPN)
        self.rate_level = (DiagnosticStatus.ERROR if cfg.get("rate_level") == "error"
                           else DiagnosticStatus.WARN)

        self.last_rx = None
        self.last_pos = None
        self.last_change_rx = None
        self.rate_ewma = None
        self.last_jump_rx = None
        self.last_jump_step = 0.0
        self.jump_count = 0
        self.nan_count = 0

        if self.checks & {"freeze", "jump", "nan"}:
            cls = roslib.message.get_message_class(cfg.get("msg_type", ""))
            if cls is None:
                rospy.logwarn("[%s] msg_type %r unknown; value checks disabled",
                              name, cfg.get("msg_type"))
                self.checks -= {"freeze", "jump", "nan"}
                rospy.Subscriber(self.topic, AnyMsg, self._on_any, queue_size=20)
            else:
                rospy.Subscriber(self.topic, cls, self._on_typed, queue_size=20)
        else:
            rospy.Subscriber(self.topic, AnyMsg, self._on_any, queue_size=20)

    def _tick_rate(self, now):
        if self.last_rx is not None:
            dt = (now - self.last_rx).to_sec()
            if dt > 0:
                inst = 1.0 / dt
                self.rate_ewma = inst if self.rate_ewma is None else 0.9 * self.rate_ewma + 0.1 * inst
        self.last_rx = now

    def _on_any(self, _msg):
        self._tick_rate(rospy.Time.now())

    def _on_typed(self, msg):
        now = rospy.Time.now()
        self._tick_rate(now)
        pos = _position(msg)
        if pos is None:
            return
        if any(math.isnan(c) or math.isinf(c) for c in pos):
            self.nan_count += 1
            return
        if self.last_pos is None:
            self.last_change_rx = now
        else:
            moved = _norm(tuple(a - b for a, b in zip(pos, self.last_pos)))
            if moved > self.freeze_eps:            # moved beyond deadband -> not frozen
                self.last_change_rx = now
            if moved > self.jump_max_step:         # too far between consecutive samples -> teleport
                self.last_jump_rx = now
                self.last_jump_step = moved
                self.jump_count += 1
        self.last_pos = pos

    def evaluate(self, now):
        """(level, message), side-effect free. Shared by run_diag (2Hz report) and the
        supervisor (fast trip decision)."""
        if self.last_rx is None:
            return DiagnosticStatus.WARN, "no messages yet on %s" % self.topic
        age = (now - self.last_rx).to_sec()
        rate = self.rate_ewma or 0.0
        freeze_dur = (now - self.last_change_rx).to_sec() if self.last_change_rx else 0.0
        jump_recent = self.last_jump_rx is not None and (now - self.last_jump_rx).to_sec() < 2.0
        if "dropout" in self.checks and age > self.dropout_timeout:
            return DiagnosticStatus.ERROR, "DROPOUT: no msg for %.2fs" % age
        if "freeze" in self.checks and freeze_dur > self.freeze_window:
            return DiagnosticStatus.ERROR, "FROZEN: value unchanged %.2fs" % freeze_dur
        if "jump" in self.checks and jump_recent:
            return DiagnosticStatus.WARN, "JUMP: %.2f m step" % self.last_jump_step
        if "rate" in self.checks and self.min_rate > 0 and rate < self.min_rate:
            return self.rate_level, "low rate %.1f Hz (< %.0f)" % (rate, self.min_rate)
        return DiagnosticStatus.OK, "ok (%.1f Hz)" % rate

    def run_diag(self, stat):
        now = rospy.Time.now()
        level, msg = self.evaluate(now)
        if self.last_rx is not None:
            stat.add("topic", self.topic)
            stat.add("age_since_last_s", "%.3f" % (now - self.last_rx).to_sec())
            stat.add("rate_hz", "%.1f" % (self.rate_ewma or 0.0))
            if "freeze" in self.checks:
                fd = (now - self.last_change_rx).to_sec() if self.last_change_rx else 0.0
                stat.add("freeze_s", "%.2f" % fd)
            if "jump" in self.checks:
                stat.add("jumps_total", self.jump_count)
                stat.add("last_jump_step_m", "%.3f" % self.last_jump_step)
            if "nan" in self.checks:
                stat.add("nan_count", self.nan_count)
        stat.summary(level, msg)


class PingMonitor(object):
    """General host-reachability probe (LINK layer). Pings on its own period so the
    blocking call never stalls the diagnostic thread; run_diag reads the last result."""

    def __init__(self, name, cfg):
        self.name = name
        self.host = cfg["host"]
        self.timeout = float(cfg.get("timeout_s", 1.0))
        self.warn_ms = float(cfg.get("warn_ms", 50.0))
        self.error_ms = float(cfg.get("error_ms", 200.0))
        self.last_ok = None
        self.last_rtt = None
        self.last_probe = None
        rospy.Timer(rospy.Duration(float(cfg.get("period_s", 1.0))), self._probe)

    def _probe(self, _evt):
        try:
            out = subprocess.check_output(
                ["ping", "-c", "1", "-W", str(int(max(1, self.timeout))), self.host],
                stderr=subprocess.STDOUT, universal_newlines=True)
            rtt = None
            for tok in out.split():
                if tok.startswith("time="):
                    rtt = float(tok.split("=")[1])
            self.last_ok, self.last_rtt = True, rtt
        except subprocess.CalledProcessError:
            self.last_ok, self.last_rtt = False, None
        except Exception as e:
            rospy.logwarn_throttle(10.0, "[%s] ping error: %s" % (self.name, e))
            self.last_ok = None
        self.last_probe = rospy.Time.now()

    def run_diag(self, stat):
        now = rospy.Time.now()
        stat.add("host", self.host)
        if self.last_probe is None:
            stat.summary(DiagnosticStatus.WARN, "no probe yet")
            return
        stat.add("probe_age_s", "%.1f" % (now - self.last_probe).to_sec())
        if self.last_ok is False:
            stat.summary(DiagnosticStatus.ERROR, "UNREACHABLE: %s" % self.host)
        elif self.last_ok is None:
            stat.summary(DiagnosticStatus.WARN, "ping unavailable")
        else:
            rtt = self.last_rtt or 0.0
            stat.add("rtt_ms", "%.1f" % rtt)
            if rtt > self.error_ms:
                stat.summary(DiagnosticStatus.ERROR, "high latency %.0f ms" % rtt)
            elif rtt > self.warn_ms:
                stat.summary(DiagnosticStatus.WARN, "latency %.0f ms" % rtt)
            else:
                stat.summary(DiagnosticStatus.OK, "reachable (%.1f ms)" % rtt)
