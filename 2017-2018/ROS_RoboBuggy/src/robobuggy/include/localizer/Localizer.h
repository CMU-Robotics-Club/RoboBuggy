#include "ros/ros.h"

#include <stdio.h>

class Localizer
{
public:
    Localizer();
    static void IMU_Callback(const robobuggy::IMU::ConstPtr& msg);
    static void GPS_Callback(const robobuggy::GPS::ConstPtr& msg);
    static void ENC_Callback(const robobuggy::ENC::ConstPtr& msg);
private:
    void update_position_estimate();
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Subscriber gps_pub;
    ros::Subscriber imu_pub;
    ros::Subscriber enc_pub;
    double current_gps_lat;
    double current_gps_lon;
    double current_encoder_ticks;
    double prev_encoder_ticks;
    double current_imu_heading;
};