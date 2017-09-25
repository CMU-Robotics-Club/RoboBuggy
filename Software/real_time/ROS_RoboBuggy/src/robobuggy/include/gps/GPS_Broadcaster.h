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
    static const std::string NODE_NAME;
private:
    ros::NodeHandle nh;
    ros::Publisher gps_pub;
    float start_lat;
    float start_lon;
    const float increment;
};

#endif //ROS_ROBOBUGGY_GPS_BROADCASTER_H
