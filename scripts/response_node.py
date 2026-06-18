#!/usr/bin/env python
"""L3 response/mux node -- *** KILL AUTHORITY ***. See flight_safety.response.Response.
Consumes /flight_safety/fault (L2) + /mavros/rc/in; drives /mavros/setpoint_raw/local or disarm.
Run intentionally, only when intervention is wanted (actuation gated by require_armed).
"""
import rospy

from flight_safety.response import Response


def main():
    rospy.init_node("flight_safety_response")
    Response()
    rospy.logwarn("[flight_safety_response] up -- KILL authority (require_armed gates actuation)")
    rospy.spin()


if __name__ == "__main__":
    main()
