#!/usr/bin/env python
"""M2 reaction tier — supervisor + control MUX. Has KILL authority; acts only while armed
(require_armed). Run separately from monitor_node (observe-only).

Flow each cycle (react_rate_hz):
  1. evaluate fault rules -> candidate responses (monitor ERROR -> mapped response)
  2. geofence OUTSIDE -> kill candidate
  3. pick highest-severity candidate
  4. terminal (kill) -> disarm; else hand the response to the control MUX
"""
import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from diagnostic_msgs.msg import DiagnosticStatus

from flight_safety.checks import TopicMonitor
from flight_safety.consistency import PairConsistencyMonitor
from flight_safety.actions import MavrosActions
from flight_safety.responses import build_responses
from flight_safety.geofence import Geofence, OUTSIDE
from flight_safety.control_mux import ControlMux

ERROR = DiagnosticStatus.ERROR


class VehicleState(object):
    def __init__(self):
        self.armed = False
        self.mode = ""
        self.position = None
        self.yaw = 0.0


def build_monitors(subsystems):
    mons = {}
    for sub, layers in subsystems.items():
        for layer, cfg in layers.items():
            name = "%s/%s" % (sub, layer)
            if cfg.get("kind") == "consistency":
                mons[name] = PairConsistencyMonitor(name, cfg)
            elif cfg.get("kind", "topic") == "topic":
                mons[name] = TopicMonitor(name, cfg)
    return mons


class Supervisor(object):
    def __init__(self):
        self.monitors = build_monitors(rospy.get_param("~subsystems", {}))
        self.responses = build_responses(rospy.get_param("~responses", {}))
        self.rules = rospy.get_param("~rules", [])
        self.require_armed = bool(rospy.get_param("~require_armed", True))

        self.actions = MavrosActions()
        self.geofence = Geofence(rospy.get_param("~geofence"))
        self.mux = ControlMux(rospy.get_param("~control_mux"), self.actions, self.geofence)

        self.state = VehicleState()
        self.killed = False

        rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=5)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_pose, queue_size=5)

        hz = float(rospy.get_param("~react_rate_hz", 50.0))
        rospy.Timer(rospy.Duration(1.0 / hz), self._tick)
        rospy.logwarn("[safety] supervisor up @ %.0fHz (require_armed=%s) -- KILL authority",
                      hz, self.require_armed)

    def _on_state(self, m):
        self.state.armed, self.state.mode = m.armed, m.mode

    def _on_pose(self, m):
        self.state.position = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        q = m.pose.orientation
        self.state.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _select_response(self, now):
        best = None
        for rule in self.rules:
            mon = self.monitors.get(rule.get("monitor"))
            if mon is not None and mon.evaluate(now)[0] >= ERROR:
                r = self.responses.get(rule.get("response"))
                if r is not None and (best is None or r.severity > best.severity):
                    best = r
        if self.geofence.status(now) == OUTSIDE:
            k = self.responses.get("kill")
            if k is not None and (best is None or k.severity > best.severity):
                best = k
        return best

    def _tick(self, _evt):
        if self.killed:
            return
        now = rospy.Time.now()
        if self.require_armed and not self.state.armed:
            return

        resp = self._select_response(now)
        for r in self.responses.values():       # re-arm latched responses (e.g. hold capture)
            if r is not resp:
                r.reset()

        # control priority ladder: 4 KILL > 3 Manual > 2 Response > 1 Normal
        if resp is not None and getattr(resp, "terminal", False):          # 4 KILL (overrides pilot)
            self.killed = True
            rospy.logfatal("[safety] KILL")
            resp.execute(self.actions)
            return
        if self.state.mode == "OFFBOARD" and self.mux.manual_active():     # 3 Manual (RC override)
            self.mux.to_manual(now)
            return
        if self.state.mode == "OFFBOARD":                                  # 2 Response / 1 Normal
            self.mux.publish(now, self.state, resp)


def main():
    rospy.init_node("flight_safety_supervisor")
    Supervisor()
    rospy.spin()


if __name__ == "__main__":
    main()
