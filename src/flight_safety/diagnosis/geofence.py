"""L1 diagnosis: geofence box health from VRPN position.
INSIDE -> OK, APPROACHING (within margin) -> WARN, OUTSIDE -> ERROR, no pose -> WARN.
Pure reporter to /diagnostics (no actuation, no clamp). Severity IS the policy:
the L3 response maps WARN->land, ERROR->kill.
"""
import rospy
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticStatus

_AXES = ("x", "y", "z")


class GeofenceDiag(object):
    def __init__(self, cfg):
        self.box = cfg["box"]
        self.margin = float(cfg.get("margin_m", 0.3))
        self.timeout = float(cfg.get("pose_timeout_s", 0.2))
        self.pos = None
        self.last_rx = None
        rospy.Subscriber(cfg["source"], PoseStamped, self._on_pose, queue_size=5)

    def _on_pose(self, m):
        self.pos = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        self.last_rx = rospy.Time.now()

    def run_diag(self, stat):
        now = rospy.Time.now()
        if self.last_rx is None or (now - self.last_rx).to_sec() > self.timeout:
            stat.summary(DiagnosticStatus.WARN, "UNKNOWN: no pose")
            return
        stat.add("pos_xyz_m", "[%.2f %.2f %.2f]" % self.pos)
        worst = float("inf")
        for i, ax in enumerate(_AXES):
            lo, hi = self.box[ax]
            c = self.pos[i]
            if c < lo or c > hi:
                stat.summary(DiagnosticStatus.ERROR, "OUTSIDE %s=%.2f" % (ax, c))
                return
            worst = min(worst, c - lo, hi - c)
        if worst < self.margin:
            stat.summary(DiagnosticStatus.WARN, "APPROACHING (%.2fm)" % worst)
        else:
            stat.summary(DiagnosticStatus.OK, "INSIDE (%.2fm)" % worst)
