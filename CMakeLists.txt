cmake_minimum_required(VERSION 3.0.2)
project(flight_safety)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/monitor_node.py scripts/safety_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
