#!/usr/bin/env python3
"""flight_safety status LED: /flight_safety/state + RC (kill ch9 + mode ch7) + setpoint -> APA102.

Launched (backgrounded) by this module's run.sh as a SEPARATE process from the KILL-authority
response node, so a GPIO hiccup can never stall the 50 Hz safety loop. Runs inside the privileged
container (-> /dev/gpiochip0; msgs on path). A single APA102 shows one color at a time, so a
two-color state is rendered by ALTERNATING the two colors (a 2-color blink).

Inputs, each its own freshness:
  - /mavros/rc/in read DIRECTLY (own pipeline; works even if flight_safety/response is down):
      * MANUAL KILL switch  = ch9 (RC_MAP_KILL_SW=9), >KILL_US engaged
      * FLIGHT MODE selector = ch7 (RC_MAP_FLTMODE 6-slot), >MODE_OFFB_US = offboard slot (top)
  - /mavros/setpoint_raw/local : is offboard actually being COMMANDED? (fresh vs stale)
  - /flight_safety/state : control_lane (+ mode) -- the autonomous decision

Color scheme (priority top-down):
  s/w (auto) KILL  (lane==KILL, no manual switch)         -> RED solid          (FAULT force-disarm, terminal)
  manual KILL + RC offboard                               -> RED <-> GREEN      (killed; would resume offboard)
  manual KILL + RC not-offboard (position/other)          -> RED <-> BLUE       (killed; would resume position)
  RC offboard, FC NOT offboard (offboard-loss failsafe)   -> GREEN <-> BLUE     (dropped to Position on its own)
  mission hold_failed + healthy commanded OFFBOARD      -> GREEN <-> WHITE    (failed trial, holding)
  mission landing + FC AUTO.LAND                         -> CYAN blink         (intentional landing handoff)
  ground-start enabled, disarmed POSCTL, ready            -> CYAN solid         (pilot may select OFFBOARD)
  ground-start enabled, disarmed POSCTL, preparing        -> BLUE <-> WHITE     (wait in POSCTL)
  NORMAL (offboard) + setpoint fresh                      -> GREEN solid        (autonomous, commanded)
  NORMAL (offboard) + no setpoint                         -> GREEN blink        (offboard starving)
  MANUAL  mode==POSCTL                                    -> BLUE solid         (pilot position hold)
  MANUAL  other manual mode (att/alt)                     -> BLUE blink         (less-assisted, caution)
  LAND                                                    -> AMBER <-> GREEN    (emergency descend via offboard)
  nothing fresh                                           -> OFF                (down/absent stack reads dark)

Wiring (40-pin J30): APA102 DI->pin19, CI->pin23, VCC->pin2 (5V), GND->pin6.
Colors below are plain edits -- no image rebuild needed to change them.
"""
import json
import rospy
import Jetson.GPIO as GPIO
from flight_safety.msg import FlightState
from mavros_msgs.msg import RCIn, PositionTarget
from std_msgs.msg import String

DATA, CLK = 19, 23      # BOARD pins -> APA102 DI / CI
BRIGHT = 8              # 0..31
RENDER_HZ = 20.0
TIMEOUT = 1.0           # s without /flight_safety/state -> lane unknown
RC_TIMEOUT = 1.0        # s without /mavros/rc/in -> kill/mode pipeline unknown
SP_TIMEOUT = 0.5        # s without a setpoint -> offboard "no command"

KILL_CHANNEL = 8        # /mavros/rc/in idx for RC ch9 (RC_MAP_KILL_SW=9)
KILL_US = 1500          # ch9 above this = manual kill engaged (measured 2011 on / 988 off)
MODE_CHANNEL = 6        # /mavros/rc/in idx for RC ch7 (RC_MAP_FLTMODE=7), 6-slot mode selector
MODE_OFFB_US = 1900     # ch7 above this = offboard slot (top, slot6=COM_FLTMODE6=7); measured 2011 offboard
SETPOINT_TOPIC = "/mavros/setpoint_raw/local"   # matches response.yaml setpoint_out

# blink rates (full on/off cycles per second); 0.0 == solid
KILL_MODE_HZ = 4.0      # manual kill: RED <-> mode color
FAILSAFE_HZ = 2.0       # offboard-loss fallback: GREEN <-> BLUE
NORMAL_NODATA_HZ = 2.0  # offboard but no setpoint stream: GREEN <-> off
MANUAL_BLINK_HZ = 2.0   # less-assisted manual mode: BLUE <-> off
LAND_HZ = 2.0           # emergency descend (executed via offboard): AMBER <-> GREEN

SOLID_BLUE_MODES = {"POSCTL"}   # manual modes shown as SOLID blue (else caution blink)

RED, GREEN, BLUE, AMBER = (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 80, 0)
OFF = (0, 0, 0)
WHITE, CYAN = (255, 255, 255), (0, 255, 255)


def _byte(b):
    for i in range(8):
        GPIO.output(DATA, (b >> (7 - i)) & 1)
        GPIO.output(CLK, GPIO.HIGH)
        GPIO.output(CLK, GPIO.LOW)


def _show(r, g, b):
    for _ in range(4):
        _byte(0x00)
    _byte(0xE0 | (BRIGHT & 0x1F))
    _byte(b)
    _byte(g)
    _byte(r)
    for _ in range(4):
        _byte(0xFF)


class LedNode(object):
    def __init__(self):
        self.state = None
        self.state_last = None
        self.kill_high = False
        self.mode_offb = False
        self.rc_last = None
        self.sp_last = None
        self.mission = None
        self.mission_last = None
        # Optional stack-neutral mission hint; never overrides safety/kill.
        rospy.Subscriber(rospy.get_param("~mission_status_topic", "/control/mission_status"),
                         String, self._mission_cb, queue_size=1)
        rospy.Subscriber("/flight_safety/state", FlightState, self._state_cb, queue_size=1)
        rospy.Subscriber("/mavros/rc/in", RCIn, self._rc_cb, queue_size=1)
        rospy.Subscriber(SETPOINT_TOPIC, PositionTarget, self._sp_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / RENDER_HZ), self._render)

    def _mission_cb(self, m):
        try:
            value = json.loads(m.data)
            self.mission = value if isinstance(value, dict) and value.get("dry_run") is False else None
            self.mission_last = rospy.Time.now()
        except (ValueError, TypeError):
            self.mission = None

    def _state_cb(self, m):
        self.state = m
        self.state_last = rospy.Time.now()

    def _rc_cb(self, m):
        ch = m.channels
        self.kill_high = KILL_CHANNEL < len(ch) and ch[KILL_CHANNEL] > KILL_US
        self.mode_offb = MODE_CHANNEL < len(ch) and ch[MODE_CHANNEL] > MODE_OFFB_US
        self.rc_last = rospy.Time.now()

    def _sp_cb(self, _m):
        self.sp_last = rospy.Time.now()

    def _fresh(self, stamp, timeout):
        return stamp is not None and 0 <= (rospy.Time.now() - stamp).to_sec() < timeout

    def _decide(self):
        """Return (colorA, colorB, blink_hz): alternate A/B at blink_hz; 0 hz = solid A."""
        rc = self._fresh(self.rc_last, RC_TIMEOUT)

        # 1. manual kill switch -- independent RC pipeline, highest priority
        if rc and self.kill_high:
            return RED, (GREEN if self.mode_offb else BLUE), KILL_MODE_HZ

        if not self._fresh(self.state_last, TIMEOUT) or self.state is None:
            return OFF, OFF, 0.0
        lane, mode = self.state.control_lane, self.state.mode

        # 2. s/w (auto) kill -- manual already handled above, so lane==KILL here is our force-disarm
        if lane == "KILL":
            return RED, RED, 0.0

        # Explicit mission states are optional. Only trust a fresh live hint
        # while safety is OK and the actual FC mode/setpoint agrees with it.
        mission = self.mission if self._fresh(self.mission_last, TIMEOUT) else None
        if mission and self.state.level == 0 and lane != "LAND":
            if (mission.get("state") == "hold_failed" and lane == "NORMAL"
                    and mode == "OFFBOARD" and self._fresh(self.sp_last, SP_TIMEOUT)):
                return GREEN, WHITE, 2.0
            if mission.get("state") == "landing" and mode == "AUTO.LAND":
                return CYAN, OFF, 2.0

        # 3. offboard-loss failsafe: RC still asks offboard but FC dropped to another mode
        if rc and self.mode_offb and mode != "OFFBOARD":
            return GREEN, BLUE, FAILSAFE_HZ

        # Optional generic ground-start readiness. Never masks kill/fault,
        # failsafe, an armed vehicle, or an expired mission hint.
        if (mission and mission.get("ground_start_enabled") is True
                and self.state.level == 0 and lane == "MANUAL" and mode == "POSCTL"
                and not self.state.armed):
            if mission.get("offboard_entry_ready") is True:
                return CYAN, OFF, 0.0
            return BLUE, WHITE, 2.0

        # 4. control lane
        if lane == "NORMAL":
            sp = self._fresh(self.sp_last, SP_TIMEOUT)
            return GREEN, OFF, 0.0 if sp else NORMAL_NODATA_HZ
        if lane == "MANUAL":
            return BLUE, OFF, 0.0 if mode in SOLID_BLUE_MODES else MANUAL_BLINK_HZ
        if lane == "LAND":
            return AMBER, GREEN, LAND_HZ
        return OFF, OFF, 0.0

    def _render(self, _evt):
        a, b, hz = self._decide()
        if hz > 0 and int(rospy.Time.now().to_sec() * hz * 2) % 2:
            _show(*b)
        else:
            _show(*a)


if __name__ == "__main__":
    rospy.init_node("flight_safety_led")
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(DATA, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(CLK, GPIO.OUT, initial=GPIO.LOW)
    LedNode()
    try:
        rospy.spin()
    finally:
        _show(0, 0, 0)
        GPIO.cleanup()
