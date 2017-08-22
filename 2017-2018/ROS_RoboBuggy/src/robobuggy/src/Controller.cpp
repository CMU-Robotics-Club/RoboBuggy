#include <robobuggy/ENC.h>
#include "ros/ros.h"
#include "robobuggy/IMU.h"
#include "robobuggy/GPS.h"
#include "std_msgs/String.h"
#include <string>

void IMU_Callback(const robobuggy::IMU::ConstPtr& msg)
{
    ROS_INFO("Received IMU message with accelerations: (%f, %f, %f)", msg->X_Accel, msg->Y_Accel, msg->Z_Accel);
}

void GPS_Callback(const robobuggy::GPS::ConstPtr& msg)
{
    ROS_INFO("Received GPS message with coordinates: (%fm, %fm)", msg->Lat_m, msg->Long_m);
}

void ENC_Callback(const robobuggy::ENC::ConstPtr& msg)
{
    ROS_INFO("Received ENC message with coordinates: (%fm, %fm, %fm, %fm)", msg->distance_m, msg->velocity_ms, msg->acceleration_mss, msg->raw_data_word);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "Controller");

    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe<robobuggy::IMU>("IMU", 1000, IMU_Callback);
    ros::Subscriber gps_sub = nh.subscribe<robobuggy::GPS>("GPS", 1000, GPS_Callback);
    ros::Subscriber enc_sub = nh.subscribe<robobuggy::ENC>("ENC", 1000, ENC_Callback);

    ros::spin();

    return 0;
}
