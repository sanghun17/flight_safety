#!/bin/bash
# control/flight-safety: the whole module at once -- observe (L1+L2) + response (L3, KILL AUTHORITY)
# + estimator mux in one roslaunch, plus the rqt_runtime_monitor /diagnostics view in a browser.
# Actuation gated by require_armed. Ctrl-C kills all of it.
if [ ! -f /.dockerenv ]; then
  source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)/scripts/lib/select_stack.sh"
  dsd_select_stack "control/flight-safety" || exit $?
  : "${DSD_CONTAINER:?set DSD_CONTAINER or invoke through './setup.sh run <stack> control/flight-safety'}"
fi
__C="${DSD_CONTAINER:-drone-stack-${DSD_STACK_NAME:-unknown}}"
__M="roslaunch flight_safety safety.launch"
__NODES="flight_safety_(diagnosis|monitor|response)|vision_pose_mux|robot_odom_relay|planning_odom_mux"
__killall(){ docker exec "$__C" pkill -INT -f "$__M"    >/dev/null 2>&1
             docker exec "$__C" pkill      -f "$__NODES" >/dev/null 2>&1
             docker exec "$__C" pkill      -f "fs_led_node.py" >/dev/null 2>&1   # SIGTERM -> node blanks the LED on exit
             docker exec "$__C" pkill      -f "rqt_runtime_monitor" >/dev/null 2>&1; }   # GUI window; shared VNC infra spared

if [ ! -f /.dockerenv ]; then
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/scripts/lib/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  docker exec "$__C" bash /work/modules/control/mavros/check_runtime.sh messages-only || exit $?
  docker exec "$__C" bash -lc 'source /work/config/ros_env.sh; source /work/scripts/lib/ensure_roscore.sh'  # master up before the GUI
  "$__R/scripts/lib/vnc_gui.sh" 99 5900 6080 monitor \
    rqt --standalone rqt_runtime_monitor.runtime_monitor.RuntimeMonitor || true   # diagnostic GUI (best-effort)
  __TT=$([ -t 1 ] && echo -it || echo -i)
  trap '__killall; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  __killall
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
bash /work/modules/control/mavros/check_runtime.sh messages-only
source /work/ws/flight-safety/devel/setup.bash --extend   # flight_safety pkg + Fault/FlightState msgs
source /work/config/ros_env.sh
source /work/scripts/lib/ensure_roscore.sh
# Optional Jetson status LED (control_lane -> APA102). The safety capability is
# usable on non-Jetson hosts without Jetson.GPIO or /dev/gpiochip0.
if python3 -c 'import Jetson.GPIO' >/dev/null 2>&1 && [ -e /dev/gpiochip0 ]; then
  taskset -c "${CPUS_POOL}" python3 /work/modules/control/flight-safety/fs_led_node.py >/tmp/fs_led.log 2>&1 &
fi
recorder_args=("allow_external_termination:=${FLIGHT_SAFETY_ALLOW_EXTERNAL_TERMINATION:-false}")
[ -z "${FLIGHT_SAFETY_RECORDER_CONFIG:-}" ] || recorder_args+=("recorder_config:=$FLIGHT_SAFETY_RECORDER_CONFIG")
[ -z "${FLIGHT_SAFETY_GEOFENCE_CONFIG:-}" ] || recorder_args+=("geofence_config:=$FLIGHT_SAFETY_GEOFENCE_CONFIG")
[ -z "${FLIGHT_SAFETY_ESTIMATION_SOURCE:-}" ] || recorder_args+=("estimation_source:=$FLIGHT_SAFETY_ESTIMATION_SOURCE")
[ -z "${FLIGHT_SAFETY_EXTERNAL_POSE_TOPIC:-}" ] || recorder_args+=("external_pose_topic:=$FLIGHT_SAFETY_EXTERNAL_POSE_TOPIC")
[ -z "${FLIGHT_SAFETY_CONSISTENCY_CONFIG:-}" ] || recorder_args+=("consistency_config:=$FLIGHT_SAFETY_CONSISTENCY_CONFIG")
exec taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" roslaunch flight_safety safety.launch "${recorder_args[@]}" "$@"
