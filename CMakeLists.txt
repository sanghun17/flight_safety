cmake_minimum_required(VERSION 3.0.2)
project(flight_safety)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  roscpp
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(FILES
  Fault.msg
  FlightState.msg
)
generate_messages(DEPENDENCIES std_msgs)

# header-only C++ (stream_diagnostic.h) exported via include/ for vrpn_client_ros / fast-livo;
# Python nodes + generated msgs are the rest. CATKIN_DEPENDS re-exports deps.
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_updater roscpp std_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/diagnosis_node.py
  scripts/monitor_node.py
  scripts/response_node.py
  scripts/recorder_node.py
  scripts/fcu_audit_receipt.py
  scripts/fcu_trace_capture.py
  scripts/fcu_timesync_trace_capture.py
  scripts/vio_shadow_preflight.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES
  config/fcu_audit_read_only.yaml
  config/fcu_shadow_bundle_manifest.json
  config/hybrid_imu_runtime_build_manifest.json
  config/mavros_param_runtime_build_manifest.json
  config/mavros_param_source_tree_manifest.json
  config/vio_shadow_preflight.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(FILES launch/vio_shadow_preflight.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(FILES docs/vio_shadow_preflight.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs)
