"""Geofence judged by VRPN position.

status(): INSIDE / APPROACHING (within margin of a boundary) / OUTSIDE / UNKNOWN (source
stale). The supervisor maps OUTSIDE -> kill; the control MUX clamps APPROACHING so the
vehicle can't cross. UNKNOWN -> do nothing (a VRPN fault response handles that case).
"""
import rospy
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticStatus

INSIDE, APPROACHING, OUTSIDE, UNKNOWN = "INSIDE", "APPROACHING", "OUTSIDE", "UNKNOWN"
_AXES = ("x", "y", "z")


class Geofence(object):
    def __init__(self, cfg):
        self.box = cfg["box"]
        self.margin = float(cfg.get("margin_m", 0.3))
        self.timeout = float(cfg.get("pose_timeout_s", 0.2))
        self.gain = float(cfg.get("clamp_gain", 1.5))    # allowed outward vel = gain * dist-to-boundary
        self.pos = None
        self.last_rx = None
        rospy.Subscriber(cfg["source"], PoseStamped, self._on_pose, queue_size=5)

    def _on_pose(self, m):
        self.pos = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        self.last_rx = rospy.Time.now()

    def status(self, now):
        if self.last_rx is None or (now - self.last_rx).to_sec() > self.timeout:
            return UNKNOWN
        worst = float("inf")
        for i, ax in enumerate(_AXES):
            lo, hi = self.box[ax]
            c = self.pos[i]
            if c < lo or c > hi:
                return OUTSIDE
            worst = min(worst, c - lo, hi - c)
        return APPROACHING if worst < self.margin else INSIDE

    def clamp(self, sp):
        """CBF-style clamp of a velocity setpoint: outward speed -> 0 at the boundary.
        (Position-setpoint fields are mask-ignored, so leaving them is harmless.)"""
        if self.pos is None:
            return sp
        v = [sp.velocity.x, sp.velocity.y, sp.velocity.z]
        for i, ax in enumerate(_AXES):
            lo, hi = self.box[ax]
            toward_hi = max(0.0, self.gain * (hi - self.pos[i]))
            toward_lo = max(0.0, self.gain * (self.pos[i] - lo))
            if v[i] > toward_hi:
                v[i] = toward_hi
            elif v[i] < -toward_lo:
                v[i] = -toward_lo
        sp.velocity.x, sp.velocity.y, sp.velocity.z = v
        return sp

    def run_diag(self, stat):
        """Observe-only diagnostic (used by monitor_node; no actuation)."""
        st = self.status(rospy.Time.now())
        if self.pos is not None:
            stat.add("pos_xyz_m", "[%.2f %.2f %.2f]" % self.pos)
        stat.add("status", st)
        level = {INSIDE: DiagnosticStatus.OK, APPROACHING: DiagnosticStatus.WARN,
                 UNKNOWN: DiagnosticStatus.WARN, OUTSIDE: DiagnosticStatus.ERROR}[st]
        stat.summary(level, st)
