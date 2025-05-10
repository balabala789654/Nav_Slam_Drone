#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/State.h"

class vio_pose_
{
private:
    mavros_msgs::State state;
    geometry_msgs::PoseStamped pose_msg;
    ros::Subscriber vio_sub;
    ros::Subscriber state_sub;

    ros::Publisher pose_pub;
public:
    vio_pose_();
    void init(void);
    void vio_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    
};

vio_pose_::vio_pose_()
{
    init();
}

void vio_pose_::init(void){
    ros::NodeHandle nh;
    ros::Rate r(10);
    vio_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 10, &vio_pose_::vio_callback, this);
    state_sub=nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &vio_pose_::state_callback, this);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    while(!state.connected && ros::ok()){
        ROS_INFO_STREAM("waiting mavros connect...");
        ros::spinOnce();
        r.sleep();
    }

    while (ros::ok())
    {
        ros::spinOnce();
    }
}

void vio_pose_::vio_callback(const nav_msgs::Odometry::ConstPtr& msg){
    
    pose_msg.pose = msg->pose.pose;
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = ros::Time::now();
    pose_pub.publish(pose_msg);
}

void vio_pose_::state_callback(const mavros_msgs::State::ConstPtr& msg){
    state = *msg;
}

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "vio_pose");
    vio_pose_ vio_pose;
    return 0;
}
