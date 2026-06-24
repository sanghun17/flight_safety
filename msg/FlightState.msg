# L3 consolidated state -- estimator + controller + monitor at a glance (/flight_safety/state).
std_msgs/Header header

uint8  level             # 0 OK · 1 WARN · 2 ERROR (== worst monitor)
string summary           # one-line dashboard (echo .../state/summary for just this)

# controller
bool   armed
string mode              # PX4 nav mode (/mavros/state)
bool   kill_switch       # pilot RC manual kill switch engaged (mavros: System -> "manual kill switch engaged")
string control_lane      # MANUAL · NORMAL · LAND · KILL  (active mux lane)
string active_response   # '' · land · kill

# estimator
string estimator         # VRPN · VIO

# monitor (most-severe fault surfaced; geofence folds in here)
string worst_monitor     # '' if all OK
string worst_message
