#!/usr/bin/env python
import rospy
import diagnostic_updater

from flight_safety.checks import TopicMonitor, PingMonitor
from flight_safety.consistency import PairConsistencyMonitor
from flight_safety.geofence import Geofence

KINDS = {
    "ping": PingMonitor,
    "topic": TopicMonitor,
    "consistency": PairConsistencyMonitor,
}


def main():
    rospy.init_node("flight_safety_monitor")
    updater = diagnostic_updater.Updater()
    updater.setHardwareID("flight_safety")

    # config: subsystems -> layers (link/stream/consistency/...) -> {kind, ...}
    for sub, layers in rospy.get_param("~subsystems", {}).items():
        for layer, cfg in layers.items():
            cls = KINDS.get(cfg.get("kind"))
            if cls is None:
                rospy.logwarn("[%s/%s] unknown kind %r", sub, layer, cfg.get("kind"))
                continue
            name = "%s/%s" % (sub, layer)
            updater.add(name, cls(name, cfg).run_diag)

    # geofence: observe-only status (INSIDE/APPROACHING/OUTSIDE/UNKNOWN), no actuation here
    gf_cfg = rospy.get_param("~geofence", None)
    if gf_cfg:
        updater.add("geofence", Geofence(gf_cfg).run_diag)

    hz = float(rospy.get_param("~update_rate_hz", 2.0))
    rospy.Timer(rospy.Duration(1.0 / hz), lambda _e: updater.update())
    rospy.loginfo("[flight_safety_monitor] up (observe-only)")
    rospy.spin()


if __name__ == "__main__":
    main()
