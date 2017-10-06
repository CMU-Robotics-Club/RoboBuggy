//
// Created by bhai on 9/29/17.
//

#ifndef ROS_ROBOBUGGY_TRANSISTOR_GPS_BROADCASTER_H
#define ROS_ROBOBUGGY_TRANSISTOR_GPS_BROADCASTER_H

#include "ros/ros.h"
#include "serial/serial.h"
#include <gtest/gtest.h>

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
    
    double convert_to_latitude(std::string str);
    double convert_to_longitude(std::string str);

    // test cases that use private
    FRIEND_TEST(GPS_Test_Suite, gps_test_lat_normal);
    FRIEND_TEST(GPS_Test_Suite, gps_test_lat_null);
    FRIEND_TEST(GPS_Test_Suite, gps_test_lat_emptystr);
    FRIEND_TEST(GPS_Test_Suite, gps_test_lat_outofworld);

    FRIEND_TEST(GPS_Test_Suite, gps_test_lon_normal);
    FRIEND_TEST(GPS_Test_Suite, gps_test_lon_null);
    FRIEND_TEST(GPS_Test_Suite, gps_test_lon_emptystr);
    FRIEND_TEST(GPS_Test_Suite, gps_test_lon_outofworld);
};

#endif //ROS_ROBOBUGGY_TRANSISTOR_GPS_BROADCASTER_H
