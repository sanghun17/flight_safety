#!/usr/bin/python3
"""IMU touchdown -> KILL during an emergency LAND.

NOTE shebang is /usr/bin/python3 (NOT `env python`): roslaunch runs this straight from
source (not yet catkin-installed), and `python` is python2.7 here (no yaml -> rospy import
fails). The installed sibling nodes work only because catkin rewrote their shebang to py3.

While the L3 response is LANDing (descending in place), watch the IMU: the first
ground contact is a sharp WORLD-VERTICAL specific-force spike + airframe roll
(see flight_safety.touchdown). On a confirmed touchdown, inject an ERROR onto
/flight_safety/inject_fault -> the L2 monitor surfaces ERROR -> the L3 response
(the SOLE kill authority) force-disarms. We never disarm directly; routing through
the response keeps one kill authority and reuses the existing inject hook.

Gated to control_lane==LAND, so a hard maneuver in normal flight can't trip it; and
the feature is the EKF-vertical component (not |a|, not raw body-z), so lateral accel
doesn't inflate it and a tip-on-contact doesn't hide it. Separate process from the
response node (an IMU stall must not touch the 50 Hz kill loop).

Validate thresholds against bags first: tools/flight_safety/validate_touchdown.py
"""
import rospy
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State
from diagnostic_msgs.msg import DiagnosticStatus

from flight_safety.msg import FlightState
from flight_safety.touchdown import TouchdownDetector


def world_vert(q, a):
    """ENU-up component of body specific-force a, given body->world quat q (x,y,z,w)."""
    return 2 * (q.x * q.z + q.w * q.y) * a.x + \
           2 * (q.y * q.z - q.w * q.x) * a.y + \
           (1 - 2 * (q.x * q.x + q.y * q.y)) * a.z


class TouchdownNode(object):
    def __init__(self):
        self.det = TouchdownDetector(
            vert_thresh=float(rospy.get_param("~vert_thresh", 16.0)),
            gyro_thresh=float(rospy.get_param("~gyro_thresh", 0.3)),
            min_land_s=float(rospy.get_param("~min_land_s", 0.3)),
            confirm=int(rospy.get_param("~confirm", 2)),
            window_s=float(rospy.get_param("~window_s", 0.25)))
        self.in_land = False
        self.armed = False
        self.triggered = False     # latched once touchdown fires
        self.pub = rospy.Publisher("/flight_safety/inject_fault", DiagnosticStatus, queue_size=1)
        rospy.Subscriber("/flight_safety/state", FlightState, self._on_state, queue_size=5)
        rospy.Subscriber("/mavros/state", State, self._on_mav, queue_size=5)
        rospy.Subscriber(rospy.get_param("~imu_topic", "/mavros/imu/data"),
                         Imu, self._on_imu, queue_size=20)
        # hammer the ERROR at >monitor rate while triggered+armed so the 10 Hz monitor catches it
        rospy.Timer(rospy.Duration(0.05), self._inject_tick)
        rospy.logwarn("[touchdown] up -- LAND-gated IMU touchdown->KILL ARMED "
                      "(vert>%.1f & gyro>=%.2f, confirm %d/%.2fs)",
                      self.det.vert_thresh, self.det.gyro_thresh, self.det.confirm, self.det.window_s)

    def _on_state(self, m):
        self.in_land = (m.control_lane == "LAND")

    def _on_mav(self, m):
        self.armed = m.armed

    def _on_imu(self, m):
        vert = world_vert(m.orientation, m.linear_acceleration)
        g = m.angular_velocity
        gmag = (g.x * g.x + g.y * g.y + g.z * g.z) ** 0.5
        if self.det.update(m.header.stamp.to_sec(), vert, gmag, self.in_land) and not self.triggered:
            self.triggered = True
            rospy.logfatal("[touchdown] TOUCHDOWN detected (vert=%.1f gyro=%.2f) -> ERROR -> KILL", vert, gmag)

    def _inject_tick(self, _evt):
        if not (self.triggered and self.armed):
            return
        st = DiagnosticStatus()
        st.level = DiagnosticStatus.ERROR
        st.name = "touchdown"
        st.message = "IMU touchdown during LAND -> kill"
        self.pub.publish(st)


def main():
    rospy.init_node("flight_safety_touchdown")
    TouchdownNode()
    rospy.spin()


if __name__ == "__main__":
    main()
