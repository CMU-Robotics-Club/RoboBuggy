//
// Created by bhai on 9/15/17.
//

#ifndef ROS_ROBOBUGGY_GPS_BROADCASTER_H
#define ROS_ROBOBUGGY_GPS_BROADCASTER_H

#include "ros/ros.h"
#include "robobuggy/GPS.h"

class GPS_Broadcaster
{
public:
    GPS_Broadcaster();
    void publish_spoof_message();
    static const std::string NODE_NAME = "GPS_Broadcaster";
private:
    ros::NodeHandle nh;
    ros::Publisher gps_pub;
    float start_lat = 0.0;
    float start_lon = 0.0;
    const float increment = 0.00001;
};

#endif //ROS_ROBOBUGGY_GPS_BROADCASTER_H
