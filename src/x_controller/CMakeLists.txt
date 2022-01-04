cmake_minimum_required(VERSION 3.0.2)
project(x_controller)

set(CMAKE_BUILD_TYPE Debug)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Suppress a boost error
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS=1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  x_linker
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/x_controller_node.cpp
  src/ur5.cpp
  src/kinematics.cpp
  src/gripper.cpp
)

add_dependencies(${PROJECT_NAME}_node x_linker_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/send_gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
