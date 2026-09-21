#!/bin/bash
# catkin-build the flight-safety workspace IN-CONTAINER. Standalone overlay on noetic —
# build deps are all noetic (diagnostic_updater/roscpp/std_msgs/message_generation).
# Must build BEFORE optitrack (vrpn_client_ros #includes flight_safety/stream_diagnostic.h)
# — enforced by optitrack's `needs: [control/flight-safety]`.
# mavros_msgs is supplied by the distro package declared in module.yml.
set -e
source /opt/ros/noetic/setup.bash
cd /work/ws/flight-safety
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release >/dev/null
catkin build
echo ">> flight-safety ws built"
