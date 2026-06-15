"""Control MUX: priority arbitration of three control sources.

  3 (highest) Manual    : RC sticks moved while OFFBOARD -> hand to pilot (switch to POSCTL)
  2           Response  : hold / land setpoint from an active fault response
  1 (lowest)  Normal    : passthrough of the JAX/planner offboard setpoint

The selected setpoint is geofence-clamped at this single output point, then published to
setpoint_out. The normal source must publish to `normal_in` (control_bridge re-points there).
"""
import rospy
from mavros_msgs.msg import PositionTarget, RCIn

import flight_safety.geofence as gf


class ControlMux(object):
    def __init__(self, cfg, actions, geofence):
        self.actions = actions
        self.geofence = geofence
        self.pub = rospy.Publisher(cfg["setpoint_out"], PositionTarget, queue_size=1)
        rospy.Subscriber(cfg["normal_in"], PositionTarget, self._on_normal, queue_size=1)
        rospy.Subscriber(cfg.get("rc_topic", "/mavros/rc/in"), RCIn, self._on_rc, queue_size=5)
        self.normal = None
        self.rc = []
        self.manual_mode = cfg.get("manual_mode", "POSCTL")
        self.channels = cfg.get("rc_channels", [0, 1, 3])
        self.center = float(cfg.get("rc_center_us", 1500))
        self.rng = float(cfg.get("rc_range_us", 500))
        self.thr = float(cfg.get("rc_deflection", 0.25))
        self._last_manual = None

    def _on_normal(self, m):
        self.normal = m

    def _on_rc(self, m):
        self.rc = list(m.channels)

    def manual_active(self):
        if not self.rc:
            return False
        for i in self.channels:
            if i < len(self.rc) and abs(self.rc[i] - self.center) / self.rng > self.thr:
                return True
        return False

    def step(self, now, state, response):
        # 3. Manual (highest): RC moved while OFFBOARD -> switch to manual, stop publishing.
        if state.mode == "OFFBOARD" and self.manual_active():
            if self._last_manual is None or (now - self._last_manual).to_sec() > 1.0:
                self.actions.set_mode(self.manual_mode)
                self._last_manual = now
            return
        if state.mode != "OFFBOARD":
            return   # pilot/other mode owns the vehicle; MUX stays out

        # 2. Response offboard, else 1. Normal offboard.
        sp = response.setpoint(state) if response is not None else None
        if sp is None:
            sp = self.normal
        if sp is None:
            return

        # single geofence clamp at the output.
        if self.geofence.status(now) == gf.APPROACHING:
            sp = self.geofence.clamp(sp)
        sp.header.stamp = now
        self.pub.publish(sp)
