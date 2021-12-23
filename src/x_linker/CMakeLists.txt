cmake_minimum_required(VERSION 2.8.12)
project(x_linker)

## Suppress a boost error
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS=1)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  gazebo_ros
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_service_files(
  FILES
  Link.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  DEPENDS 
    roscpp
    gazebo_ros
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(x_linker src/x_linker.cpp)
target_link_libraries(x_linker
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES})
