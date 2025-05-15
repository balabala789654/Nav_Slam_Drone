source devel/setup.bash
sleep 2
roslaunch realsense2_camera rs_camera.launch & 
sleep 15
roslaunch mavros px4.launch &
sleep 15
rosrun vins vins_node src/VINS-Fusion/config/myconfig/myconfig.yaml &
wait
