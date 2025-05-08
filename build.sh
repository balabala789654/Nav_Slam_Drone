echo "start build...."
#!/bin/bash

readonly VERSION_ROS1="ROS1"
cd `dirname $0`
echo "Working Path: "`pwd`
ROS_VERSION=""
ROS_VERSION=${VERSION_ROS1}
echo "ROS version is: "$ROS_VERSION
# clear `build/` folder.
# TODO: Do not clear these folders, if the last build is based on the same ROS version.
rm -rf ./build/
rm -rf ./devel/
rm -rf ./install/
# clear src/CMakeLists.txt if it exists.
if [ -f ./src/CMakeLists.txt ]; then
    rm -f ./src/CMakeLists.txt
fi
catkin_make -DROS_EDITION=ROS1 -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver2"
catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_lio"
catkin_make -DCATKIN_WHITELIST_PACKAGES="quadrotor_msgs;cmake_utils;map_generator;plan_env;path_searching;bspline_opt;pose_utils;odom_visualization;local_sensing_node;mockamap;so3_control;multi_map_server;traj_utils;ego_planner;uav_utils;so3_quadrotor_simulator;rviz_plugins;waypoint_generator"
catkin_make -DCATKIN_WHITELIST_PACKAGES="my_mavros"
catkin_make -DCATKIN_WHITELIST_PACKAGES="ar_demo;benchmark_publisher;camera_model;data_generator;feature_tracker;pose_graph;vins_estimator"
catkin_make -DCATKIN_WHITELIST_PACKAGES="my_synctime"
# if [ "$1" = "lio" ]; then
#     echo "build fast_lio"
#     catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_lio"
# elif [ "$1" = "driver" ]; then
#     echo "build livox_ros_driver2"
#     catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver2"
# elif [ "$1" = "planner" ]; then
#     echo "build ego-planner"
#     catkin_make -DCATKIN_WHITELIST_PACKAGES="quadrotor_msgs;cmake_utils;map_generator;plan_env;path_searching;bspline_opt;pose_utils;odom_visualization;local_sensing_node;mockamap;so3_control;multi_map_server;traj_utils;ego_planner;uav_utils;so3_quadrotor_simulator;rviz_plugins;waypoint_generator"
#     # catkin_make -DCATKIN_WHITELIST_PACKAGES="path_searching"
#     # catkin_make -DCATKIN_WHITELIST_PACKAGES="bspline_opt"
#     # catkin_make -DCATKIN_WHITELIST_PACKAGES="traj_utils"
#     # catkin_make -DCATKIN_WHITELIST_PACKAGES="ego-planner"
# elif [ "$1" = "all" ]; then
#     echo "build all"
#     catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver2"
#     sleep 2
#     catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_lio"
#     sleep 2
#     catkin_make -DCATKIN_WHITELIST_PACKAGES="plan_env"
#     catkin_make -DCATKIN_WHITELIST_PACKAGES="path_searching"
#     catkin_make -DCATKIN_WHITELIST_PACKAGES="bspline_opt"
#     catkin_make -DCATKIN_WHITELIST_PACKAGES="traj_utils"
#     catkin_make -DCATKIN_WHITELIST_PACKAGES="ego-planner"
# else 
#     echo "warnning...."
#     exit
# fi