cmake_minimum_required(VERSION 3.0.2)
project(x_robot)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  launch
  config
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
