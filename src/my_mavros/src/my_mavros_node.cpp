#include "ros/ros.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/ParamValue.h"
#include "mavros_msgs/ParamGet.h"
#include "mavros_msgs/PositionTarget.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include <iostream>
#include "std_msgs/Char.h"
#include "std_msgs/Bool.h"

#define delta_MAX 0.1
class my_mavros{
    public:
        my_mavros();
        void Init(void);
        void state_callback(const mavros_msgs::State::ConstPtr& msg);
        void KeyBoardInput_callback(const std_msgs::Char::ConstPtr& msg);

        void DroneControl_callback(const ros::TimerEvent &e);
    private:
        char ch;
        mavros_msgs::State state;
        std::string robot, Mode, author;
        int ID;
        ros::Timer DroneControl_Timer;
        // ros::Timer KeyBoard_Timer;
        mavros_msgs::PositionTarget PosTarget;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::ParamSet Param_S;
        mavros_msgs::ParamValue Param_S_Value;
        mavros_msgs::ParamGet Param_G;
        ros::Publisher PosTarget_pub;
        ros::Publisher pub;
        ros::Publisher pub_communicate_state;
        std_msgs::Char ch_in;
        enum control_mode_ {
            vel = 1,
            pos = 2
        };
        control_mode_ control_mode;
        int set_control_mode;
        std_msgs::Bool communicate_state;

        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient Set_Param_client;
        ros::ServiceClient Get_Param_client;
};

my_mavros::my_mavros(){
    Init();
}

void my_mavros::Init(void){
    ros::NodeHandle n;
    n.param<std::string>("robot", robot, "iris");
    n.param<std::string>("Mode", Mode, "OFFBOARD");
    n.param("ID", ID, 0);
    n.param<std::string>("author", author, "user");
    n.param("control_mode", set_control_mode, 2);

    if (set_control_mode == vel)
    {
        control_mode = vel;
    }else if(set_control_mode == pos){
        control_mode = pos;
    }

    ROS_INFO_STREAM("\nrobot: "+ robot + "\nMode: " + Mode + "\nID: " + std::to_string(ID));

    pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_pos", 10);
    PosTarget_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    pub_communicate_state = n.advertise<std_msgs::Bool>("/" + author + "/control/state", 10);

    ros::Subscriber sub = n.subscribe<mavros_msgs::State>("/mavros/state", 10, &my_mavros::state_callback, this);
    ros::Subscriber sub1 = n.subscribe<std_msgs::Char>("/" + author + "/control/key", 10, &my_mavros::KeyBoardInput_callback, this);
    
    arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    Set_Param_client = n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    Get_Param_client = n.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");

    DroneControl_Timer = n.createTimer(ros::Duration(0.02), &my_mavros::DroneControl_callback, this);
    // KeyBoard_Timer = n.createTimer(ros::Duration(0.02), &my_mavros::KeyBoardInput_callback, this);

    ros::Rate r(1); // 2 times per second

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 2;
    if(control_mode == vel){
        PosTarget.velocity.x = 0;
        PosTarget.velocity.y = 0;
        PosTarget.velocity.z = 0;
        PosTarget.yaw_rate = 0;
        // PosTarget.yaw = 0;
        PosTarget.type_mask = PosTarget.IGNORE_PX + PosTarget.IGNORE_PY + PosTarget.IGNORE_PZ  + PosTarget.IGNORE_AFX + PosTarget.IGNORE_AFY + PosTarget.IGNORE_AFZ + PosTarget.IGNORE_YAW;
        PosTarget.coordinate_frame = 1;
    }else if(control_mode == pos){
        PosTarget.position.x=0;
        PosTarget.position.y=0;
        PosTarget.position.z=0;
        // PosTarget.yaw_rate = 0;
        PosTarget.yaw = 0;
        PosTarget.type_mask = PosTarget.IGNORE_VX + PosTarget.IGNORE_VY + PosTarget.IGNORE_VZ + PosTarget.IGNORE_AFX + PosTarget.IGNORE_AFY + PosTarget.IGNORE_AFZ + PosTarget.IGNORE_YAW_RATE;
        PosTarget.coordinate_frame = 1;       
    }

    offb_set_mode.request.custom_mode = std::string("AUTO.LAND");
    arm_cmd.request.value = false;

    Param_S_Value.integer=4;
    Param_S_Value.real=0.0;
    Param_S.request.param_id = std::string("COM_RCL_EXCEPT");
    Param_S.request.value = Param_S_Value;

    Param_G.request.param_id=std::string("COM_RCL_EXCEPT");
    
    while(!state.connected){
        ROS_INFO_STREAM("waiting mavros connect...");
        ros::spinOnce();
        r.sleep();
    }

    for(int i=0; i<5 && ros::ok(); i++){
        PosTarget_pub.publish(PosTarget);
        r.sleep();
    }    

    if(Set_Param_client.call(Param_S) && Param_S.response.success){
        ROS_INFO("Set Param COM_RCL_EXCEPT 4");
    }else{
        ROS_WARN("worning: can't set Param COM_RCL_EXCEPT 4");
    }

    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        ROS_INFO("set OFFBOARD mode");
    }
    if(arming_client.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("state: arm");
    }

    Get_Param_client.call(Param_G);
    ROS_INFO_STREAM("COM_RCL_EXCEPT: " + std::to_string(Param_G.response.value.integer));

    communicate_state.data = true;
    pub_communicate_state.publish(communicate_state);

    ros::spin(); //注意，spin()作用是一个死循环，但是与while的区别是只有spin()才可以进入回调函数。
    
    // while(ros::ok()){
    //     // pub.publish(pose); //在设置完offboard与arm解锁后，在发送期望位置信息。
    //     PosTarget_pub.publish(PosTarget);
    //     ros::spinOnce();
    //     r.sleep();
    // }
}

void my_mavros::state_callback(const mavros_msgs::State::ConstPtr& msg){
    state = *msg;
    return;
}

void my_mavros::DroneControl_callback(const ros::TimerEvent &e){
    PosTarget_pub.publish(PosTarget);
    return;
}

void my_mavros::KeyBoardInput_callback(const std_msgs::Char::ConstPtr& msg){
    ch_in = *msg;

    if(ch_in.data == 'l'){
        offb_set_mode.request.custom_mode = std::string("AUTO.LAND");
        set_mode_client.call(offb_set_mode);        
    }else if(ch_in.data == 'o'){
        offb_set_mode.request.custom_mode = Mode;
        set_mode_client.call(offb_set_mode);
    }

    if(ch_in.data == 't'){
        arm_cmd.request.value = true;
        arming_client.call(arm_cmd);       
    }else if(ch_in.data == 'y'){ //LOL中y键是锁定视角，因此用y键锁无人机再好不过了。
        arm_cmd.request.value = false;
        arming_client.call(arm_cmd);
    }

    if(control_mode == vel){
        if(ch_in.data == 'w'){
            PosTarget.velocity.x += delta_MAX;
        }else if(ch_in.data == 's'){
            PosTarget.velocity.x -= delta_MAX;
        }else if(ch_in.data == 'a'){
            PosTarget.velocity.y += delta_MAX;
        }else if(ch_in.data == 'd'){
            PosTarget.velocity.y -= delta_MAX;
        }else if(ch_in.data == 'i'){
            PosTarget.velocity.z += delta_MAX;
        }else if(ch_in.data == 'k'){
            PosTarget.velocity.z -= delta_MAX;
        }else{}

        if(ch_in.data != ' ')
            ROS_INFO_STREAM("\ncontrol_mode: vel" << "\nvel_x: " << PosTarget.velocity.x << "\nvel_y: " << PosTarget.velocity.y << "\nvel_z" << PosTarget.velocity.z);
        return;
    }else if(control_mode == pos){
        if(ch_in.data == 'w'){
            PosTarget.position.x += delta_MAX;
        }else if(ch_in.data == 's'){
            PosTarget.position.x -= delta_MAX;
        }else if(ch_in.data == 'a'){
            PosTarget.position.y += delta_MAX;
        }else if(ch_in.data == 'd'){
            PosTarget.position.y -= delta_MAX;
        }else if(ch_in.data == 'i'){
            PosTarget.position.z += delta_MAX;
        }else if(ch_in.data == 'k'){
            PosTarget.position.z -= delta_MAX;
        }else{}
        if(ch_in.data != ' ')
            ROS_INFO_STREAM("\ncontrol_mode: pos" << "\npos_x: " << PosTarget.position.x << "\npos_y: " << PosTarget.position.y << "\npos_z" << PosTarget.position.z);
        return;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hello");
    my_mavros mavros;
    return 0;
}





