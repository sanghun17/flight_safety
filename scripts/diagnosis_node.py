#!/usr/bin/env python
"""L1 diagnosis node: runs OUR derived per-subsystem reporters (geofence, consistency)
and publishes their OK/WARN/ERROR to /diagnostics. vrpn and mavros self-report from inside
their own nodes (vrpn via stream_diagnostic.h, mavros via its built-in plugins), so they
need no file here. Observe-only -- no actuation.
"""
import rospy
import diagnostic_updater

from flight_safety.diagnosis.geofence import GeofenceDiag
from flight_safety.diagnosis.consistency import ConsistencyDiag


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

    hz = float(rospy.get_param("~rate_hz", 2.0))
    rospy.Timer(rospy.Duration(1.0 / hz), lambda _e: up.update())
    rospy.loginfo("[flight_safety_diagnosis] up (geofence, consistency -> /diagnostics)")
    rospy.spin()


if __name__ == "__main__":
    main()
