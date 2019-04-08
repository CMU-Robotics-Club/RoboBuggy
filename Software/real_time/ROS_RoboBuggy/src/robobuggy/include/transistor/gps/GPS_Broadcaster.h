//
// Created by bhai on 9/15/17.
//

#ifndef ROS_ROBOBUGGY_GPS_BROADCASTER_H
#define ROS_ROBOBUGGY_GPS_BROADCASTER_H

#include "ros/ros.h"
#include "robobuggy/GPS.h"
#include "serial/serial.h"
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>

#include <sstream>

class GPS_Broadcaster
{
public:
    GPS_Broadcaster();
    static const std::string NODE_NAME;

    void read_gps_message();
    robobuggy::GPS* parse_tokens(std::string tokens[]);
    int initialize_hardware();

private:
    ros::NodeHandle nh;

    ros::Publisher gps_pub;
    robobuggy::GPS gps_message;

    std::string serial_port;
    int serial_baud;
    std::string gps_serial_buffer;
    serial::Serial gps_serial;

    double convert_to_latitude(std::string str);
    double convert_to_longitude(std::string str);
};

#endif //ROS_ROBOBUGGY_GPS_BROADCASTER_H
