#!/usr/bin/env python
"""L1 diagnosis node: runs OUR per-subsystem reporters -> /diagnostics (OK/WARN/ERROR). Two kinds:
  - derived verdicts (geofence, consistency): judge only when they HAVE input, neutral otherwise.
  - source liveness (~liveness/*, e.g. mavros local_position): flags a source that does NOT
    self-report to /diagnostics going silent. (vrpn/mavros plugins self-report from their own nodes.)
Observe-only -- no actuation.
"""
import rospy
import diagnostic_updater

from flight_safety.diagnosis.geofence import GeofenceDiag
from flight_safety.diagnosis.consistency import ConsistencyDiag
from flight_safety.diagnosis.liveness import TopicLiveness


def main():
    rospy.init_node("flight_safety_diagnosis")
    up = diagnostic_updater.Updater()
    up.setHardwareID("flight_safety")

    gf = rospy.get_param("~geofence", None)
    if gf:
        up.add("geofence", GeofenceDiag(gf).run_diag)
    co = rospy.get_param("~consistency", None)
    if co:
        up.add("consistency", ConsistencyDiag(co).run_diag)
    for name, cfg in (rospy.get_param("~liveness", {}) or {}).items():
        up.add(name, TopicLiveness(cfg).run_diag)

    hz = float(rospy.get_param("~rate_hz", 2.0))
    rospy.Timer(rospy.Duration(1.0 / hz), lambda _e: up.update())
    rospy.loginfo("[flight_safety_diagnosis] up -> /diagnostics")
    rospy.spin()


if __name__ == "__main__":
    main()
