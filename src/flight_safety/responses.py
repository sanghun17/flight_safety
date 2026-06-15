"""Fault responses. Three kinds, by severity: hold(1) < land(2) < kill(3).

hold/land produce a mavros PositionTarget for the control MUX (priority-2 "response
offboard" source). kill is terminal (disarm via actions).

VERIFY-BEFORE-FLIGHT: frame=FRAME_LOCAL_NED and sign/axes are NOT bench-verified (see
docs/flight_safety_architecture.md V1). Structure is final; numbers need verification.
"""
from mavros_msgs.msg import PositionTarget

_FRAME = PositionTarget.FRAME_LOCAL_NED
_MASK_POS = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
             PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
             PositionTarget.IGNORE_YAW_RATE)
_MASK_VEL = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
             PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
             PositionTarget.IGNORE_YAW)


def _pos_target(p, yaw=0.0):
    t = PositionTarget()
    t.coordinate_frame = _FRAME
    t.type_mask = _MASK_POS
    t.position.x, t.position.y, t.position.z = p
    t.yaw = yaw
    return t


def _vel_target(vx, vy, vz, yaw_rate=0.0):
    t = PositionTarget()
    t.coordinate_frame = _FRAME
    t.type_mask = _MASK_VEL
    t.velocity.x, t.velocity.y, t.velocity.z = vx, vy, vz
    t.yaw_rate = yaw_rate
    return t


class HoldResponse(object):
    """Hold the position captured at activation."""
    severity = 1
    terminal = False

    def __init__(self, cfg):
        self._captured = None

    def reset(self):
        self._captured = None

    def setpoint(self, state):
        if self._captured is None:
            if state.position is None:
                return None
            self._captured = state.position
        return _pos_target(self._captured, state.yaw or 0.0)


class LandResponse(object):
    """Descend in place: zero horizontal velocity, constant descent rate."""
    severity = 2
    terminal = False

    def __init__(self, cfg):
        self.vz = -abs(float(cfg.get("descend_mps", 0.4)))   # sign per stack convention (VERIFY)

    def reset(self):
        pass

    def setpoint(self, state):
        return _vel_target(0.0, 0.0, self.vz)


class KillResponse(object):
    """Force-disarm. Terminal — bypasses the MUX."""
    severity = 3
    terminal = True

    def __init__(self, cfg):
        pass

    def reset(self):
        pass

    def setpoint(self, state):
        return None

    def execute(self, actions):
        actions.kill()


REGISTRY = {"hold": HoldResponse, "land": LandResponse, "kill": KillResponse}


def build_responses(cfg):
    out = {}
    for name, rc in (cfg or {}).items():
        cls = REGISTRY.get((rc or {}).get("type", name))
        if cls is not None:
            out[name] = cls(rc or {})
    return out
