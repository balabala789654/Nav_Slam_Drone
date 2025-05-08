cmake_minimum_required(VERSION 3.0.2)
project(my_synctime)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_filters
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES my_synctime
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_filters
 DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(synctime_node src/synctime.cpp)

target_link_libraries(synctime_node
  ${catkin_LIBRARIES}
)

