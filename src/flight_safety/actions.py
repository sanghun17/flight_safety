import rospy
from mavros_msgs.srv import CommandLong, SetMode

MAV_CMD_COMPONENT_ARM_DISARM = 400
FORCE_MAGIC = 21196   # PX4 param2 value to force (dis)arm in flight


class MavrosActions(object):
    """Thin wrapper over the MAVROS command services the supervisor needs."""

    def __init__(self):
        self._command = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        self._set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    def kill(self):
        """Force-disarm in flight (kill switch). DESTRUCTIVE: motors stop, vehicle drops."""
        try:
            self._command(broadcast=False, command=MAV_CMD_COMPONENT_ARM_DISARM,
                          confirmation=0, param1=0.0, param2=float(FORCE_MAGIC),
                          param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0)
            rospy.logfatal("[safety] KILL: force-disarm sent")
        except rospy.ServiceException as e:
            rospy.logerr("[safety] kill failed: %s", e)

    def set_mode(self, mode):
        try:
            self._set_mode(base_mode=0, custom_mode=mode)
            rospy.logwarn("[safety] set_mode -> %s", mode)
        except rospy.ServiceException as e:
            rospy.logerr("[safety] set_mode %s failed: %s", mode, e)
