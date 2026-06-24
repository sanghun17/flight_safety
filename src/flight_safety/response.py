"""L3 response/mux: pick ONE lane from pilot kill-switch + manual-RC + the L2 worst level, drive mavros.

  kill switch engaged -> KILL    (pilot already force-terminated via PX4; we only reflect it, never actuate)
  manual RC present    -> MANUAL  (hand to pilot: switch to manual_mode)
  level OK             -> NORMAL  (passthrough the planner setpoint on normal_in)
  level WARN           -> LAND    (descend in place)
  level ERROR          -> KILL    (force-disarm; terminal)

Priority: kill > manual > ERROR > WARN > OK. The pilot kill switch is an authority/terminal
signal, NOT a health grade -- it is read DIRECTLY from the RC kill channel (RC_MAP_KILL_SW), a
separate pipeline kept OUT of the L2 severity path AND off the mavros diagnostic, so it can never be
mistaken for a WARN-that-lands. Severity IS the policy for the health monitors; L1 decides WARN vs ERROR.
LAND/KILL actuation is gated by require_armed; the NORMAL setpoint stream is forwarded even when
disarmed (so OFFBOARD entry sees a live stream); state is published every tick for visibility.
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
        self.normal_timeout = float(rospy.get_param("~normal_timeout_s", 0.5))   # drop a stale planner setpoint
        self.estimator = str(rospy.get_param("~estimator", "VRPN"))
        rc = rospy.get_param("~rc", {})
        self.rc_channels = rc.get("channels", [0, 1, 3])
        self.rc_center = float(rc.get("center_us", 1500))
        self.rc_range = float(rc.get("range_us", 500))
        self.rc_thr = float(rc.get("deflection", 0.25))
        self.manual_mode = rc.get("manual_mode", "POSCTL")
        self.kill_channel = int(rc.get("kill_channel", 8))      # /mavros/rc/in idx for RC ch9 (RC_MAP_KILL_SW=9)
        self.kill_us = float(rc.get("kill_engaged_us", 1500))   # ch above this = kill engaged (measured 2011 on / 988 off)

        self.actions = MavrosActions()
        self.out = rospy.Publisher(rospy.get_param("~setpoint_out", "/mavros/setpoint_raw/local"),
                                   PositionTarget, queue_size=1)
        self.state_pub = rospy.Publisher("/flight_safety/state", FlightState, queue_size=1)

        self.fault = Fault()      # level OK until L2 publishes
        self.armed = False
        self.mode = ""
        self.rc = []
        self.normal = None
        self.normal_stamp = rospy.Time(0)   # last /local_controller/setpoint_raw/local arrival (staleness guard)
        self.killed = False       # OUR force-disarm latched (auto, ERROR) -- distinct from pilot kill switch
        self._last_manual = None

        rospy.Subscriber("/flight_safety/fault", Fault, self._on_fault, queue_size=1)
        rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=5)
        rospy.Subscriber(rc.get("topic", "/mavros/rc/in"), RCIn, self._on_rc, queue_size=5)
        rospy.Subscriber(rospy.get_param("~normal_in", "/local_controller/setpoint_raw/local"),
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
        self.normal_stamp = rospy.Time.now()

    def _manual(self):
        """Pilot owns the vehicle: FC is in a non-OFFBOARD mode (pilot selected it, or PX4 RC-override
        switched offboard->Position on stick). flight_safety only FOLLOWS the FC mode now."""
        if self.mode and self.mode != "OFFBOARD":
            return True
        # ② RC stick-deflection takeover DISABLED -- isolate PX4 COM_RC_OVERRIDE for testing. With this off,
        #    moving sticks in OFFBOARD does NOT flip us to MANUAL; only PX4 switching the FC mode does.
        #    TODO remove _to_manual()/manual_mode + rc deadband params (rc_channels/center/range/thr) after verified.
        # for i in self.rc_channels:
        #     if i < len(self.rc) and abs(self.rc[i] - self.rc_center) / self.rc_range > self.rc_thr:
        #         return True
        return False

    def _kill(self):
        """Pilot RC manual kill switch engaged: the kill channel (RC_MAP_KILL_SW) deflected high."""
        i = self.kill_channel
        return i < len(self.rc) and self.rc[i] > self.kill_us

    def _tick(self, _evt):
        now = rospy.Time.now()
        level = self.fault.level
        kill = self._kill()
        manual = self._manual()
        lane = "KILL" if (kill or self.killed) else ("MANUAL" if manual else _LANE.get(level, "NORMAL"))

        self._publish_state(now, lane, manual, kill)   # always, even disarmed

        if self.killed or kill:               # terminated -- reflect only, never forward/fight it
            return

        # LAND/KILL actuation: only when flight_safety owns the vehicle (OFFBOARD, not manual) and
        # armed. In a manual (non-OFFBOARD) mode the pilot owns it -- step back, no land/kill.
        if not manual and (self.armed or not self.require_armed):
            if level >= ERROR:
                self.killed = True
                rospy.logfatal("[safety] KILL: %s", ", ".join(self.fault.names) or "error")
                self.actions.kill()
                return
            if level >= WARN:
                self._publish_setpoint(self._land_sp())
                return

        # NORMAL stream: forward the FRESH planner/control setpoint to the FCU. ALWAYS (even
        # disarmed / POSCTL) so the pilot can ENTER offboard -- PX4 ignores offboard setpoints
        # unless in OFFBOARD, so forwarding pre-offboard is harmless but keeps the stream alive.
        # Staleness guard: if control_bridge stops, the stream stops -> PX4 offboard-loss failsafe
        # rather than latching the last command. This MUX is the SOLE publisher to setpoint_out.
        if self.normal is not None and (now - self.normal_stamp).to_sec() < self.normal_timeout:
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

    def _publish_state(self, now, lane, manual, kill):
        m = FlightState()
        m.header.stamp = now
        m.level = self.fault.level
        m.armed = self.armed
        m.mode = self.mode
        m.kill_switch = kill
        m.control_lane = lane
        m.active_response = _RESP[lane]
        m.estimator = self.estimator
        if self.fault.names:
            m.worst_monitor = self.fault.names[0]
            m.worst_message = self.fault.messages[0] if self.fault.messages else ""
        if kill:
            lane_str = "KILL(pilot)/%s" % (self.mode or "-")
        elif self.killed:
            lane_str = "KILL(auto)/%s" % (self.mode or "-")
        elif manual:
            lane_str = "MANUAL/%s" % (self.mode or "-")
        else:
            lane_str = "%s/%s" % (self.mode or "-", lane)
        mon = ("%s: %s" % (m.worst_monitor, m.worst_message)) if m.worst_monitor else "mon OK"
        m.summary = "%s | %s | est %s | %s" % (_LVL.get(m.level, "?"), lane_str, m.estimator, mon)
        self.state_pub.publish(m)
