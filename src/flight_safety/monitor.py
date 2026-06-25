"""L2 monitor: listen to ALL L1 diagnosis output (/diagnostics), find the most severe
fault (possibly several at the same worst level). A WATCHED source that stops reporting
(node death) is synthesized as ERROR -- one mechanism covers every subsystem's death.
Observe-only; emits a verdict the L3 response layer acts on.
"""
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR


class Monitor(object):
    def __init__(self, sources):
        # sources: [{match: <substring of /diagnostics name>, stale_s: <death timeout>}]
        self.sources = [(s["match"], float(s.get("stale_s", 2.0))) for s in sources]
        self.last = {}   # match -> (level, message, stamp)
        rospy.Subscriber("/diagnostics", DiagnosticArray, self._on_diag, queue_size=20)

    def _on_diag(self, arr):
        now = rospy.Time.now()
        for st in arr.status:
            for match, _ in self.sources:
                if match in st.name:
                    self.last[match] = (st.level, st.message, now)

    def worst(self, now):
        """(level, names, messages) over watched sources. Never-seen -> WARN; gone stale
        after being seen -> ERROR(dead). names/messages = the sources AT the worst level."""
        per = []
        level = OK
        for match, stale_s in self.sources:
            rec = self.last.get(match)
            if rec is None:
                lv, msg = WARN, "no report yet"
            elif (now - rec[2]).to_sec() > stale_s:
                lv, msg = ERROR, "DEAD: silent %.1fs" % (now - rec[2]).to_sec()
            else:
                lv, msg = rec[0], rec[1]
            per.append((match, lv, msg))
            level = max(level, lv)
        names = [m for m, lv, _ in per if lv == level and level > OK]
        messages = [msg for _, lv, msg in per if lv == level and level > OK]
        return level, names, messages
