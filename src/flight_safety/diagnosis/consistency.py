"""L1 diagnosis: VRPN vs EKF2 local_position agreement.
Same frame/origin -> compare raw position error directly (no offset removal).
<warn_m -> OK, warn..error -> WARN, >error_m -> ERROR, stale/no-pair -> WARN.
A small transient during fast motion is EKF2 filter lag; the thresholds tolerate it.
"""
import math
import rospy
import message_filters
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticStatus

from flight_safety.diagnosis import add_measurement


def _xyz(m):
    p = m.pose.position
    return (p.x, p.y, p.z)


class ConsistencyDiag(object):
    def __init__(self, cfg):
        self.pair_timeout = float(cfg.get("pair_timeout_s", 0.5))
        self.warn = float(cfg.get("warn_m", 0.10))
        self.error = float(cfg.get("error_m", 0.25))
        self.err = None
        self.last_pair = None
        a = message_filters.Subscriber(cfg["topic_a"], PoseStamped)
        b = message_filters.Subscriber(cfg["topic_b"], PoseStamped)
        sync = message_filters.ApproximateTimeSynchronizer(
            [a, b], queue_size=50, slop=float(cfg.get("sync_slop_s", 0.05)))
        sync.registerCallback(self._on_pair)

    def _on_pair(self, a, b):
        pa, pb = _xyz(a), _xyz(b)
        self.err = math.sqrt(sum((x - y) ** 2 for x, y in zip(pa, pb)))
        self.last_pair = rospy.Time.now()

    def run_diag(self, stat):
        # No input -> ERROR: can't verify localization integrity (no vrpn or no EKF2 pose to compare).
        now = rospy.Time.now()
        if self.last_pair is None:
            stat.summary(DiagnosticStatus.ERROR, "no synced pairs")
            return
        age = (now - self.last_pair).to_sec()
        if age > self.pair_timeout:
            stat.summary(DiagnosticStatus.ERROR, "stale: no pair %.1fs" % age)
            return
        add_measurement(stat, self.err, "m", warn=self.warn, error=self.error)
        if self.err > self.error:
            stat.summary(DiagnosticStatus.ERROR, "MISMATCH %.2fm" % self.err)
        elif self.err > self.warn:
            stat.summary(DiagnosticStatus.WARN, "diff %.2fm" % self.err)
        else:
            stat.summary(DiagnosticStatus.OK, "match (%.3fm)" % self.err)
