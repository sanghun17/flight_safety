"""L3 response/mux: pick ONE of 4 responses from manual-RC + the L2 worst level, drive mavros.

  manual RC present  -> MANUAL  (hand to pilot: switch to manual_mode; overrides everything)
  level OK           -> NORMAL  (passthrough the planner setpoint on normal_in)
  level WARN         -> LAND    (descend in place)
  level ERROR        -> KILL    (force-disarm; terminal)

Severity IS the policy -- there is no per-fault rule table; L1 diagnosis decides WARN vs ERROR.
Actuation is gated by require_armed; state is published every tick (even disarmed) for visibility.
"""
import rospy
from mavros_msgs.msg import State, RCIn, PositionTarget
from diagnostic_msgs.msg import DiagnosticStatus

from flight_safety.actions import MavrosActions
from flight_safety.msg import Fault, FlightState

OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR
_LANE = {OK: "NORMAL", WARN: "LAND", ERROR: "KILL"}
_RESP = {"NORMAL": "", "LAND": "land", "KILL": "kill", "MANUAL": ""}
_LVL = {OK: "OK", WARN: "WARN", ERROR: "ERR"}

_MASK_VEL = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
             PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
             PositionTarget.IGNORE_YAW)


class Response(object):
    def __init__(self):
        self.require_armed = bool(rospy.get_param("~require_armed", True))
        self.descend = abs(float(rospy.get_param("~land", {}).get("descend_mps", 0.4)))
        self.estimator = str(rospy.get_param("~estimator", "VRPN"))
        rc = rospy.get_param("~rc", {})
        self.rc_channels = rc.get("channels", [0, 1, 3])
        self.rc_center = float(rc.get("center_us", 1500))
        self.rc_range = float(rc.get("range_us", 500))
        self.rc_thr = float(rc.get("deflection", 0.25))
        self.manual_mode = rc.get("manual_mode", "POSCTL")

        self.actions = MavrosActions()
        self.out = rospy.Publisher(rospy.get_param("~setpoint_out", "/mavros/setpoint_raw/local"),
                                   PositionTarget, queue_size=1)
        self.state_pub = rospy.Publisher("/flight_safety/state", FlightState, queue_size=1)

        self.fault = Fault()      # level OK until L2 publishes
        self.armed = False
        self.mode = ""
        self.rc = []
        self.normal = None
        self.killed = False
        self._last_manual = None

        rospy.Subscriber("/flight_safety/fault", Fault, self._on_fault, queue_size=1)
        rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=5)
        rospy.Subscriber(rc.get("topic", "/mavros/rc/in"), RCIn, self._on_rc, queue_size=5)
        rospy.Subscriber(rospy.get_param("~normal_in", "/flight_safety/normal_setpoint"),
                         PositionTarget, self._on_normal, queue_size=1)

        hz = float(rospy.get_param("~react_rate_hz", 50.0))
        rospy.Timer(rospy.Duration(1.0 / hz), self._tick)

    def _on_fault(self, m):
        self.fault = m

    def _on_state(self, m):
        self.armed, self.mode = m.armed, m.mode

    def _on_rc(self, m):
        self.rc = list(m.channels)

    def _on_normal(self, m):
        self.normal = m

    def _manual(self):
        """Pilot owns the vehicle: not in OFFBOARD, or RC sticks deflected past deadband."""
        if self.mode and self.mode != "OFFBOARD":
            return True
        for i in self.rc_channels:
            if i < len(self.rc) and abs(self.rc[i] - self.rc_center) / self.rc_range > self.rc_thr:
                return True
        return False

    def _tick(self, _evt):
        now = rospy.Time.now()
        level = self.fault.level
        manual = self._manual()
        lane = "MANUAL" if manual else _LANE.get(level, "NORMAL")

        self._publish_state(now, lane, manual)   # always, even disarmed

        if self.killed:
            return
        if self.require_armed and not self.armed:
            return
        if manual:
            self._to_manual(now)
            return
        if level >= ERROR:
            self.killed = True
            rospy.logfatal("[safety] KILL: %s", ", ".join(self.fault.names) or "error")
            self.actions.kill()
            return
        if level >= WARN:
            self._publish_setpoint(self._land_sp())
            return
        if self.mode == "OFFBOARD" and self.normal is not None:   # OK -> passthrough
            self._publish_setpoint(self.normal)

    def _to_manual(self, now):
        if self._last_manual is None or (now - self._last_manual).to_sec() > 1.0:
            self.actions.set_mode(self.manual_mode)
            self._last_manual = now

    def _land_sp(self):
        t = PositionTarget()
        t.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        t.type_mask = _MASK_VEL
        t.velocity.z = -self.descend   # sign per stack convention (VERIFY-BEFORE-FLIGHT)
        return t

    def _publish_setpoint(self, sp):
        sp.header.stamp = rospy.Time.now()
        self.out.publish(sp)

    def _publish_state(self, now, lane, manual):
        m = FlightState()
        m.header.stamp = now
        m.level = self.fault.level
        m.armed = self.armed
        m.mode = self.mode
        m.control_lane = lane
        m.active_response = _RESP[lane]
        m.estimator = self.estimator
        if self.fault.names:
            m.worst_monitor = self.fault.names[0]
            m.worst_message = self.fault.messages[0] if self.fault.messages else ""
        lane_str = ("MANUAL/%s" % (self.mode or "-")) if manual else ("%s/%s" % (self.mode or "-", lane))
        mon = ("%s: %s" % (m.worst_monitor, m.worst_message)) if m.worst_monitor else "mon OK"
        m.summary = "%s | %s | est %s | %s" % (_LVL.get(m.level, "?"), lane_str, m.estimator, mon)
        self.state_pub.publish(m)
