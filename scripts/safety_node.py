#!/usr/bin/env python
"""M2 reaction tier: event-driven supervisor with KILL authority.

Separate from monitor_node.py (observe-only) on purpose — only run this when you want
active intervention, not during bench tests. Acts only while armed (require_armed).
"""
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, RCIn

from flight_safety.checks import TopicMonitor
from flight_safety.consistency import PairConsistencyMonitor
from flight_safety.actions import MavrosActions

ERROR = DiagnosticStatus.ERROR


class SafetySupervisor(object):
    def __init__(self):
        vrpn = rospy.get_param("~subsystems", {}).get("vrpn", {})
        self.stream = TopicMonitor("vrpn/stream", vrpn["stream"]) if "stream" in vrpn else None
        self.consistency = (PairConsistencyMonitor("vrpn/consistency", vrpn["consistency"])
                            if "consistency" in vrpn else None)

        sc = rospy.get_param("~scenarios", {})
        self.require_armed = bool(sc.get("require_armed", True))
        self.s1 = sc.get("vrpn_fault", {})
        self.s2 = sc.get("geofence_breach", {})
        self.s3 = sc.get("rc_override", {})

        self.actions = MavrosActions()
        self.armed = False
        self.mode = ""
        self.position = None
        self.rc = []
        self.fired = set()

        rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=5)
        if self.s2.get("enable"):
            rospy.Subscriber(self.s2["pose_topic"], PoseStamped, self._on_pose, queue_size=5)
        if self.s3.get("enable"):
            rospy.Subscriber(self.s3.get("rc_topic", "/mavros/rc/in"), RCIn, self._on_rc, queue_size=5)

        hz = float(sc.get("eval_rate_hz", 50.0))
        rospy.Timer(rospy.Duration(1.0 / hz), self._tick)
        rospy.logwarn("[safety] supervisor up (eval %.0f Hz, require_armed=%s) -- KILL authority",
                      hz, self.require_armed)

    def _on_state(self, m):
        self.armed, self.mode = m.armed, m.mode

    def _on_pose(self, m):
        self.position = (m.pose.position.x, m.pose.position.y, m.pose.position.z)

    def _on_rc(self, m):
        self.rc = list(m.channels)

    def _tick(self, _evt):
        now = rospy.Time.now()
        active = self.armed or not self.require_armed

        # S1: VRPN stream or VRPN<->local_position broken -> kill/land
        if active and self.s1.get("enable") and "s1" not in self.fired:
            bad, reason = False, ""
            if self.stream is not None:
                lvl, m = self.stream.evaluate(now)
                if lvl >= ERROR:
                    bad, reason = True, m
            if not bad and self.consistency is not None:
                lvl, m = self.consistency.evaluate(now)
                if lvl >= ERROR:
                    bad, reason = True, m
            if bad:
                self._fire("s1", self.s1.get("action", "kill"), "VRPN fault: " + reason)

        # S2: outside OptiTrack zone -> kill
        if active and self.s2.get("enable") and "s2" not in self.fired and self.position is not None:
            if self._outside_box(self.position, self.s2["box"]):
                self._fire("s2", self.s2.get("action", "kill"),
                           "geofence breach @ (%.2f, %.2f, %.2f)" % self.position)

        # S3: RC input during OFFBOARD -> hand to pilot (Position mode). Re-arms when back in OFFBOARD.
        if active and self.s3.get("enable") and self.mode == "OFFBOARD" and self._rc_deflected():
            if "s3" not in self.fired:
                self.actions.set_mode(self.s3.get("mode", "POSCTL"))
                self.fired.add("s3")
                rospy.logwarn("[safety] S3 RC override -> %s", self.s3.get("mode", "POSCTL"))
        elif self.mode != "OFFBOARD":
            self.fired.discard("s3")

    def _fire(self, key, action, reason):
        self.fired.add(key)
        rospy.logfatal("[safety] %s -> %s (%s)", key, action.upper(), reason)
        if action == "kill":
            self.actions.kill()
        elif action == "land":
            self.actions.set_mode("AUTO.LAND")

    @staticmethod
    def _outside_box(p, box):
        return not (box["x"][0] <= p[0] <= box["x"][1] and
                    box["y"][0] <= p[1] <= box["y"][1] and
                    box["z"][0] <= p[2] <= box["z"][1])

    def _rc_deflected(self):
        if not self.rc:
            return False
        center = float(self.s3.get("center_us", 1500))
        rng = float(self.s3.get("range_us", 500))
        thr = float(self.s3.get("deflection", 0.25))
        for i in self.s3.get("channels", [0, 1, 3]):
            if i < len(self.rc) and abs(self.rc[i] - center) / rng > thr:
                return True
        return False


def main():
    rospy.init_node("flight_safety_supervisor")
    SafetySupervisor()
    rospy.spin()


if __name__ == "__main__":
    main()
