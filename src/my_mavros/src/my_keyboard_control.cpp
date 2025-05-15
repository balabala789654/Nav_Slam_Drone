#include "ros/ros.h"
#include <iostream>
#include "std_msgs/Float32.h"
#include "std_msgs/Char.h"
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include "mavros_msgs/State.h"
#include "std_msgs/Bool.h"


class Control_{
    public:
        Control_();
        ~Control_();
        void Init(void);
        void KeyBoard_Input_callback(const ros::TimerEvent &e);
        void Output_callback(const ros::TimerEvent &e);
        bool kbhit(void);
        void setNonBlocking(bool enable);
        void state_callback(const std_msgs::Bool::ConstPtr& msg);
    private:
        ros::Subscriber state_sub;
        ros::Timer KeyBoard_timer;
        ros::Timer Output_timer;
        char ch;
        std_msgs::Char ch_o;
        ros::Publisher pub;
        std::string Author;        
        enum control_mode_ {
            vel = 1,
            pos = 2,
            auto_ego = 3
        };
        control_mode_ control_mode;
        int set_control_mode;
        std_msgs::Bool communicate_state;

};

Control_::Control_(){
    Init();
}
Control_::~Control_(){
    setNonBlocking(false);
}

void Control_::Init(void){
    
    ros::NodeHandle nh;
    ros::Rate r(30);

    setNonBlocking(true);

    nh.param<std::string>("author", Author, "user");
    nh.param("control_mode", set_control_mode, 2);

    communicate_state.data = false;
    if (set_control_mode == vel){
        control_mode = vel;
    }else if(set_control_mode == pos){
        control_mode = pos;
    }else if(set_control_mode == auto_ego){
        control_mode = auto_ego;
    }
    
    if(control_mode != pos){
        return;
    }
    
    KeyBoard_timer = nh.createTimer(ros::Duration(0.02), &Control_::KeyBoard_Input_callback, this);
    Output_timer = nh.createTimer(ros::Duration(0.02), &Control_::Output_callback, this);
    pub = nh.advertise<std_msgs::Char>("/" + Author + "/control/key", 10);
    state_sub = nh.subscribe<std_msgs::Bool>("/" + Author + "/control/state", 10, &Control_::state_callback, this);

    while(!communicate_state.data && ros::ok()){
        r.sleep();
        ros::spinOnce();
    }
    
    ros::spin();
}

void Control_::KeyBoard_Input_callback(const ros::TimerEvent &e){

    if (kbhit()) {
        ch = getchar();
        // std::cout << "Got: " << ch << std::endl;
        // ROS_INFO_STREAM("input: " << ch);

    }else{
        ch = ' ';
    }
    return;
}

void Control_::Output_callback(const ros::TimerEvent &e){
    ch_o.data = ch;
    pub.publish(ch_o);
    return;
}

bool Control_::kbhit(void) {
    struct timeval tv = {0, 0};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

void Control_::setNonBlocking(bool enable) {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    
    if (enable) {
        ttystate.c_lflag &= ~ICANON;  // 非规范模式
        ttystate.c_lflag &= ~ECHO;    // 关闭回显
        ttystate.c_cc[VMIN] = 1;     // 最小字符数
        ttystate.c_cc[VTIME] = 0;    // 无超时
    } else {
        ttystate.c_lflag |= ICANON;
        ttystate.c_lflag |= ECHO;
    }
    
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

void Control_::state_callback(const std_msgs::Bool::ConstPtr& msg){
    communicate_state = *msg;
    return;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control");
    Control_ control;

    return 0;
}


