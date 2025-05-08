#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

class sync_{
    public:
        sync_();
        void Init(void);
        void sync_callback(const sensor_msgs::Imu::ConstPtr& msg1, const sensor_msgs::Image::ConstPtr& msg2);
    private:
        ros::NodeHandle nh;
        ros::Publisher Imu_pub, Img_pub;
        float frq_imu, frq_img;
};

sync_::sync_(){
    Init();
}

void sync_::Init(void){
    frq_imu = 100.0;
    frq_img = 30.0;

    Imu_pub = nh.advertise<sensor_msgs::Imu>("/sync/mavros/imu/data", 10);
    Img_pub = nh.advertise<sensor_msgs::Image>("/sync/usb_cam/image_raw", 10);

    message_filters::Subscriber<sensor_msgs::Imu> Imu_sub(nh, "/mavros/imu/data", 1);
    message_filters::Subscriber<sensor_msgs::Image> Img_sub(nh, "/usb_cam/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Image> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), Imu_sub, Img_sub);

    sync.registerCallback(boost::bind(&sync_::sync_callback, this, _1, _2));

    ros::spin();
    return;
}

void sync_::sync_callback(const sensor_msgs::Imu::ConstPtr& msg1, const sensor_msgs::Image::ConstPtr& msg2){
    double time_diff = fabs((msg1->header.stamp - msg2->header.stamp).toSec());
    ROS_INFO_STREAM("time diff: " << time_diff);

    Imu_pub.publish(msg1);
    Img_pub.publish(msg2);

    return;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sync");
    sync_ sync;
    return 0;
}
