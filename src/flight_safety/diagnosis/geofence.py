"""L1 diagnosis: geofence box health from OptiTrack (VRPN) position, + an rviz marker.

INSIDE -> OK | APPROACHING -> WARN | OUTSIDE -> ERROR | no pose -> ERROR (lost localization -> kill).
Boundary + margin are config (diagnosis.yaml geofence.box / margin_m). The rviz marker is a
wireframe box, status-colored (green / yellow-red blink / red / gray). Severity IS the policy: WARN->land, ERROR->kill.
"""
import rospy
from geometry_msgs.msg import PoseStamped, Point
from diagnostic_msgs.msg import DiagnosticStatus
from visualization_msgs.msg import Marker

from flight_safety.diagnosis import add_measurement

_AXES = ("x", "y", "z")
INSIDE, APPROACHING, OUTSIDE, UNKNOWN = "INSIDE", "APPROACHING", "OUTSIDE", "UNKNOWN"
# fixed status semantics (not config). APPROACHING blinks _COLOR[APPROACHING] <-> _COLOR[OUTSIDE].
_COLOR = {INSIDE: (0.0, 1.0, 0.0), APPROACHING: (1.0, 1.0, 0.0),
          OUTSIDE: (1.0, 0.0, 0.0), UNKNOWN: (0.5, 0.5, 0.5)}
_BLINK_HZ, _LINE_W, _ALPHA, _MARKER_TOPIC = 3.0, 0.02, 0.9, "/flight_safety/geofence"   # line width (m)


class GeofenceDiag(object):
    def __init__(self, cfg):
        self.box = cfg["box"]                              # criteria = OptiTrack pose (cfg["source"]), NOT mavros
        self.margin = float(cfg.get("margin_m", 0.3))
        self.timeout = float(cfg.get("pose_timeout_s", 0.2))
        self.pos = None
        self.last_rx = None
        self.frame = "odom"                                # taken from the pose header once it arrives
        rospy.Subscriber(cfg["source"], PoseStamped, self._on_pose, queue_size=5)
        self.marker_pub = rospy.Publisher(_MARKER_TOPIC, Marker, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self._publish_marker)   # 10Hz -> smooth blink

    def _on_pose(self, m):
        self.pos = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        self.frame = m.header.frame_id or self.frame
        self.last_rx = rospy.Time.now()

    def status(self, now):
        # returns (state, margin_m): signed min distance to nearest wall, <0 once outside.
        if self.last_rx is None or (now - self.last_rx).to_sec() > self.timeout:
            return UNKNOWN, None
        margin = float("inf")
        for i, ax in enumerate(_AXES):
            lo, hi = self.box[ax]
            margin = min(margin, self.pos[i] - lo, hi - self.pos[i])
        if margin < 0.0:
            return OUTSIDE, margin
        return (APPROACHING if margin < self.margin else INSIDE), margin

    def run_diag(self, stat):
        st, margin = self.status(rospy.Time.now())
        if self.pos is not None:
            stat.add("pos_xyz_m", "[%.2f %.2f %.2f]" % self.pos)
        stat.add("status", st)
        if margin is not None:
            add_measurement(stat, margin, "m", warn=self.margin, error=0.0)
        if st == OUTSIDE:
            stat.summary(DiagnosticStatus.ERROR, "OUTSIDE %.2fm past wall" % -margin)
        elif st == APPROACHING:
            stat.summary(DiagnosticStatus.WARN, "APPROACHING %.2fm to wall" % margin)
        elif st == UNKNOWN:
            stat.summary(DiagnosticStatus.ERROR, "no pose (lost localization)")
        else:
            stat.summary(DiagnosticStatus.OK, "INSIDE %.2fm" % margin)

    @staticmethod
    def _box_edges(bounds):
        """12 edges of an axis-aligned box -> list of (p0, p1) corner-pair tuples."""
        corners = {(i, j, k): (bounds["x"][i], bounds["y"][j], bounds["z"][k])
                   for i in (0, 1) for j in (0, 1) for k in (0, 1)}
        return [(corners[a], corners[b]) for a in corners for b in corners
                if a < b and sum(u != v for u, v in zip(a, b)) == 1]

    def _color(self, st, t):
        if st == APPROACHING and (t * _BLINK_HZ) % 1.0 >= 0.5:
            return _COLOR[OUTSIDE]
        return _COLOR[st]

    def _publish_marker(self, _evt):
        now = rospy.Time.now()
        st, _ = self.status(now)
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = now
        m.ns, m.id, m.type, m.action = "geofence", 0, Marker.LINE_LIST, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = _LINE_W
        for p0, p1 in self._box_edges({ax: tuple(self.box[ax]) for ax in _AXES}):
            for x, y, z in (p0, p1):
                m.points.append(Point(x, y, z))
        m.color.r, m.color.g, m.color.b = self._color(st, now.to_sec())
        m.color.a = _ALPHA
        self.marker_pub.publish(m)
