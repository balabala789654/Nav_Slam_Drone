cmake_minimum_required(VERSION 3.0.2)
project(my_mavros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  quadrotor_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES my_mavros
 CATKIN_DEPENDS roscpp rospy std_msgs quadrotor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(my_mavros_node src/my_mavros_node.cpp)

target_link_libraries(my_mavros_node
  ${catkin_LIBRARIES}
)

add_executable(my_control_node src/my_keyboard_control.cpp)

target_link_libraries(my_control_node
  ${catkin_LIBRARIES}
)

add_executable(my_vio_pose_node src/my_vio_pose.cpp)

target_link_libraries(my_vio_pose_node
  ${catkin_LIBRARIES}
)
# add_dependencies(my_mavros_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
