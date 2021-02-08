cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  hippocampus_common
  hippocampus_msgs
#  dynamic_reconfigure
)

catkin_python_setup()
# generate_dynamic_reconfigure_options(
#   config/CarrotControl.cfg
# )

catkin_package()

catkin_install_python(PROGRAMS
  nodes/setpoint_node
  scripts/create_test_paths.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
