#include "ros/ros.h"
#include "robobuggy/IMU.h"
#include "robobuggy/GPS.h"

void IMU_Callback(const robobuggy::IMU::ConstPtr& msg)
{
    ROS_INFO("Received IMU message with accelerations: (%f, %f, %f)", msg->X_Accel, msg->Y_Accel, msg->Z_Accel);
}

void GPS_Callback(const robobuggy::GPS::ConstPtr& msg)
{
    ROS_INFO("Received GPS message with coordinates: (%fm, %fm)", msg->Lat_m, msg->Long_m);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "Controller");

    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe<robobuggy::IMU>("IMU", 1000, IMU_Callback);
    ros::Subscriber gps_sub = nh.subscribe<robobuggy::GPS>("GPS", 1000, GPS_Callback);

    ros::spin();

    return 0;
}
