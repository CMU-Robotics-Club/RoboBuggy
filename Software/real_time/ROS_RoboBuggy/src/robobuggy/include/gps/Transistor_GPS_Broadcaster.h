//
// Created by bhai on 9/29/17.
//

#ifndef ROS_ROBOBUGGY_TRANSISTOR_GPS_BROADCASTER_H
#define ROS_ROBOBUGGY_TRANSISTOR_GPS_BROADCASTER_H

#include "ros/ros.h"

class Transistor_GPS_Broadcaster
{
public:
    Transistor_GPS_Broadcaster();
    static const std::string NODE_NAME;
    int handle_serial_messages();
private:
    ros::Publisher gps_pub;
};

#endif //ROS_ROBOBUGGY_TRANSISTOR_GPS_BROADCASTER_H
