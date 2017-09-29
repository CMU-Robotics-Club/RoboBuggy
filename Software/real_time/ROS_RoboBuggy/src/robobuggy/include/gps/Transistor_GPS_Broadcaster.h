//
// Created by bhai on 9/29/17.
//

#ifndef ROS_ROBOBUGGY_TRANSISTOR_GPS_BROADCASTER_H
#define ROS_ROBOBUGGY_TRANSISTOR_GPS_BROADCASTER_H

#include "ros/ros.h"
#include "serial/serial.h"

#include <sstream>

#include <robobuggy/GPS.h>

class Transistor_GPS_Broadcaster
{
public:
    Transistor_GPS_Broadcaster();
    static const std::string NODE_NAME;
    int handle_serial_messages();
private:
    ros::Publisher gps_pub;
    ros::NodeHandle nh;
    std::string serial_port;
    int serial_baud;
    std::string gps_serial_buffer;
};

#endif //ROS_ROBOBUGGY_TRANSISTOR_GPS_BROADCASTER_H
