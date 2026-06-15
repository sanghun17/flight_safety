import rospy
import message_filters
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticStatus

from flight_safety.checks import _position, _norm


class PairConsistencyMonitor(object):
    """Cross-check two pose streams (VRPN vs EKF2 local_position).

    They must be the SAME (same frame/origin) — any difference is the fault we want
    to catch. So we compare raw position error directly and alarm on it; no offset
    removal. A small transient error during fast motion is EKF2 filter lag, which is
    all the warn/error thresholds are meant to tolerate.
    """

    def __init__(self, name, cfg):
        self.name = name
        self.pair_timeout = float(cfg.get("pair_timeout_s", 0.5))
        self.err_warn = float(cfg.get("error_warn_m", 0.10))
        self.err_error = float(cfg.get("error_error_m", 0.25))

        self.last_pair = None
        self.err_mag = None
        self.err_vec = None
        self.max_err = 0.0

        a = message_filters.Subscriber(cfg["topic_a"], PoseStamped)
        b = message_filters.Subscriber(cfg["topic_b"], PoseStamped)
        sync = message_filters.ApproximateTimeSynchronizer(
            [a, b], queue_size=50, slop=float(cfg.get("sync_slop_s", 0.05)))
        sync.registerCallback(self._on_pair)

    def _on_pair(self, a, b):
        pa, pb = _position(a), _position(b)
        if pa is None or pb is None:
            return
        self.err_vec = tuple(x - y for x, y in zip(pa, pb))
        self.err_mag = _norm(self.err_vec)
        self.max_err = max(self.max_err, self.err_mag)
        self.last_pair = rospy.Time.now()

    def evaluate(self, now):
        if self.last_pair is None:
            return DiagnosticStatus.WARN, "no synced pairs (topic down or time desync)"
        age = (now - self.last_pair).to_sec()
        if age > self.pair_timeout:
            return DiagnosticStatus.WARN, "stale: no pair for %.2fs" % age
        if self.err_mag is None:
            return DiagnosticStatus.WARN, "no data yet"
        if self.err_mag > self.err_error:
            return DiagnosticStatus.ERROR, "MISMATCH %.2fm" % self.err_mag
        if self.err_mag > self.err_warn:
            return DiagnosticStatus.WARN, "diff %.2fm" % self.err_mag
        return DiagnosticStatus.OK, "match (%.3fm)" % self.err_mag

    def run_diag(self, stat):
        now = rospy.Time.now()
        level, msg = self.evaluate(now)
        if self.last_pair is not None:
            stat.add("pair_age_s", "%.3f" % (now - self.last_pair).to_sec())
            stat.add("err_m", "%.3f" % (self.err_mag or 0.0))
            if self.err_vec is not None:
                stat.add("err_xyz_m", "[%.3f %.3f %.3f]" % self.err_vec)
            stat.add("max_err_m", "%.3f" % self.max_err)
        stat.summary(level, msg)
