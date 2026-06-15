cmake_minimum_required(VERSION 3.0.2)
project(flight_safety)

find_package(catkin REQUIRED COMPONENTS diagnostic_updater roscpp)

catkin_python_setup()

# header-only: exports include/ so C++ nodes (vrpn_client_ros, fast-livo, ...) can
# #include <flight_safety/stream_diagnostic.h>. CATKIN_DEPENDS re-exports its deps.
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_updater roscpp
)

catkin_install_python(PROGRAMS scripts/monitor_node.py scripts/safety_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
