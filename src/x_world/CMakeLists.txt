cmake_minimum_required(VERSION 3.0.2)
project(x_world)

find_package(catkin REQUIRED rospy std_msgs gazebo_msgs)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs gazebo_msgs
)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/cleanup.py
  scripts/setup_1.py
  scripts/setup_2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)