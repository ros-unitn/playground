cmake_minimum_required(VERSION 3.0.2)
project(x_yolo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  x_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs sensor_msgs geometry_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/node.py
  scripts/view.py
  scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
