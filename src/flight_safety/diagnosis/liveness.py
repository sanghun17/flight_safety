"""L1 diagnosis: bare topic liveness for a source that does NOT self-report to /diagnostics
(e.g. mavros /mavros/local_position/pose). This is the 'watch the source directly' half of the
design: derived checks (consistency) stay neutral on missing input, and THIS flags the input
actually being gone. OK while fresh; ERROR once it has been seen and then goes silent > stale_s.
Never-seen -> OK/neutral (not up yet; you don't arm before sources are up).
"""
import rospy
from rospy import AnyMsg
from diagnostic_msgs.msg import DiagnosticStatus

from flight_safety.diagnosis import add_measurement


class TopicLiveness(object):
    def __init__(self, cfg):
        self.topic = cfg["topic"]
        self.stale = float(cfg.get("stale_s", 0.5))
        self.last_rx = None
        rospy.Subscriber(self.topic, AnyMsg, self._on_msg, queue_size=10)

    def _on_msg(self, _m):
        self.last_rx = rospy.Time.now()

    def run_diag(self, stat):
        stat.add("topic", self.topic)
        if self.last_rx is None:
            stat.summary(DiagnosticStatus.OK, "no msg yet (neutral)")
            return
        age = (rospy.Time.now() - self.last_rx).to_sec()
        add_measurement(stat, age, "s", error=self.stale)
        if age > self.stale:
            stat.summary(DiagnosticStatus.ERROR, "DEAD: no msg %.1fs" % age)
        else:
            stat.summary(DiagnosticStatus.OK, "alive (%.2fs)" % age)
