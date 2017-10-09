//
// Created by bhai on 9/15/17.
//

#ifndef ROS_ROBOBUGGY_GPS_BROADCASTER_H
#define ROS_ROBOBUGGY_GPS_BROADCASTER_H

#include "ros/ros.h"
#include "robobuggy/GPS.h"
#include "serial/serial.h"

#include <sstream>

class GPS_Broadcaster
{
public:
    GPS_Broadcaster();
    static const std::string NODE_NAME;
    int handle_serial_messages();
private:
    ros::NodeHandle nh;
    ros::Publisher gps_pub;
    std::string serial_port;
    int serial_baud;
    std::string gps_serial_buffer;

    double convert_to_latitude(std::string str);
    double convert_to_longitude(std::string str);
};

#endif //ROS_ROBOBUGGY_GPS_BROADCASTER_H
