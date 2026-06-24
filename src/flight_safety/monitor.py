"""L2 monitor: listen to ALL L1 diagnosis output (/diagnostics), find the most severe
fault (possibly several at the same worst level). A WATCHED source that stops reporting
(node death) is synthesized as ERROR -- one mechanism covers every subsystem's death.
Observe-only; emits a verdict the L3 response layer acts on.
"""
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR


class Monitor(object):
    def __init__(self, sources, inject_timeout_s=1.0):
        # sources: [{match: <substring of /diagnostics name>, stale_s: <death timeout>}]
        self.sources = [(s["match"], float(s.get("stale_s", 2.0))) for s in sources]
        self.last = {}   # match -> (level, message, stamp)
        # Optional fault INJECTION (test / operational "command emergency land/kill"): a recent
        # WARN/ERROR on /flight_safety/inject_fault folds into worst() like any other source.
        # Neutral by default -- no message, or a stale/OK one, contributes nothing, so normal
        # operation is unaffected. Downstream LAND/KILL is still gated (armed + OFFBOARD) in L3.
        self.inject = None   # (level, message, stamp)
        self.inject_timeout = float(inject_timeout_s)
        rospy.Subscriber("/diagnostics", DiagnosticArray, self._on_diag, queue_size=20)
        rospy.Subscriber("/flight_safety/inject_fault", DiagnosticStatus, self._on_inject, queue_size=1)

    def _on_diag(self, arr):
        now = rospy.Time.now()
        for st in arr.status:
            for match, _ in self.sources:
                if match in st.name:
                    self.last[match] = (st.level, st.message, now)

    def _on_inject(self, st):
        self.inject = (st.level, st.message or "injected fault", rospy.Time.now())

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
        if (self.inject is not None and self.inject[0] > OK and
                (now - self.inject[2]).to_sec() <= self.inject_timeout):
            per.append(("inject", self.inject[0], self.inject[1]))
            level = max(level, self.inject[0])
        names = [m for m, lv, _ in per if lv == level and level > OK]
        messages = [msg for _, lv, msg in per if lv == level and level > OK]
        return level, names, messages
