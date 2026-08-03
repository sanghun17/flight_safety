"""L2 monitor: listen to ALL L1 diagnosis output (/diagnostics), find the most severe
fault (possibly several at the same worst level). A WATCHED source that stops reporting
(node death) is synthesized as ERROR by default. An explicit per-source ``max_level``
may reduce that source's response authority while retaining its diagnostic visibility.
Observe-only; emits a verdict the L3 response layer acts on.
"""
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR


class Monitor(object):
    def __init__(self, sources):
        # max_level optionally caps one source's authority.  This lets VRPN
        # stream diagnostics request LAND while the geofence+local-position
        # pair owns the conditional KILL decision.
        self.sources = []
        for source in sources:
            max_level = int(source.get("max_level", ERROR))
            if max_level not in (OK, WARN, ERROR):
                raise ValueError("max_level must be 0, 1, or 2")
            self.sources.append((
                source["match"], float(source.get("stale_s", 2.0)), max_level))
        self.last = {}   # match -> (level, message, stamp)
        rospy.Subscriber("/diagnostics", DiagnosticArray, self._on_diag, queue_size=20)

    def _on_diag(self, arr):
        now = rospy.Time.now()
        for st in arr.status:
            for match, _, _ in self.sources:
                if match in st.name:
                    self.last[match] = (st.level, st.message, now)

    def worst(self, now):
        """(level, names, messages) over watched sources. Never-seen -> WARN; gone stale
        after being seen -> ERROR(dead), then apply max_level. names/messages are the
        sources at the resulting worst level."""
        per = []
        level = OK
        for match, stale_s, max_level in self.sources:
            rec = self.last.get(match)
            if rec is None:
                lv, msg = WARN, "no report yet"
            elif (now - rec[2]).to_sec() > stale_s:
                lv, msg = ERROR, "DEAD: silent %.1fs" % (now - rec[2]).to_sec()
            else:
                lv, msg = rec[0], rec[1]
            if lv > max_level:
                msg = "%s [policy cap: %d->%d]" % (msg, lv, max_level)
                lv = max_level
            per.append((match, lv, msg))
            level = max(level, lv)
        names = [m for m, lv, _ in per if lv == level and level > OK]
        messages = [msg for _, lv, msg in per if lv == level and level > OK]
        return level, names, messages
