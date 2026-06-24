#!/usr/bin/env python
"""L2 monitor node: /diagnostics -> worst severity -> /flight_safety/fault. Observe-only.
The most-severe fault (level + names) is the single verdict the L3 response layer consumes.
"""
import rospy

from flight_safety.monitor import Monitor
from flight_safety.msg import Fault


def main():
    rospy.init_node("flight_safety_monitor")
    mon = Monitor(rospy.get_param("~sources", []), rospy.get_param("~inject_timeout_s", 1.0))
    pub = rospy.Publisher("/flight_safety/fault", Fault, queue_size=1)
    hz = float(rospy.get_param("~publish_rate_hz", 10.0))

    def tick(_e):
        now = rospy.Time.now()
        level, names, messages = mon.worst(now)
        m = Fault()
        m.header.stamp = now
        m.level = level
        m.names = names
        m.messages = messages
        pub.publish(m)

    rospy.Timer(rospy.Duration(1.0 / hz), tick)
    rospy.loginfo("[flight_safety_monitor] up (/diagnostics -> /flight_safety/fault @%.0fHz)", hz)
    rospy.spin()


if __name__ == "__main__":
    main()
