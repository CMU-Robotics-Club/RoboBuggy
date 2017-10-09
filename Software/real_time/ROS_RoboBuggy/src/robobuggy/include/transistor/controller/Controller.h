//
// Created by bhai on 9/15/17.
//

#ifndef ROS_ROBOBUGGY_CONTROLLER_H
#define ROS_ROBOBUGGY_CONTROLLER_H

#include <robobuggy/ENC.h>
#include "ros/ros.h"
#include "robobuggy/IMU.h"
#include "robobuggy/GPS.h"
#include "std_msgs/String.h"
#include <string>

class Controller
{
public:
    static void IMU_Callback(const robobuggy::IMU::ConstPtr& msg);
    static void GPS_Callback(const robobuggy::GPS::ConstPtr& msg);
    static void ENC_Callback(const robobuggy::ENC::ConstPtr& msg);
    Controller();
    ros::NodeHandle nh;
private:
    ros::Subscriber imu_sub;
    ros::Subscriber enc_sub;
    ros::Subscriber gps_sub;
};

#endif //ROS_ROBOBUGGY_CONTROLLER_H
