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

#include <geodesy/utm.h>

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

    bool gpgga_valid(std::string tokens[]);
    bool gps_locked(std::string tokens[]);

    double convert_to_latitude(std::string str);
    double convert_to_longitude(std::string str);

    geodesy::UTMPoint gps_get_location(double latitude_deg, double longitude_deg);
    robobuggy::GPS gps_gen_message(double latitude_deg, double longitude_deg, geodesy::UTMPoint utm_point);

    bool parse_gps_string(std::string gps_serial_buffer);

    // test cases that use private methods
    FRIEND_TEST(GPS_Test_Suite, gps_test_lat_normal);
    FRIEND_TEST(GPS_Test_Suite, gps_test_lat_emptystr);
    FRIEND_TEST(GPS_Test_Suite, gps_test_lon_normal);
    FRIEND_TEST(GPS_Test_Suite, gps_test_lon_emptystr);
    FRIEND_TEST(GPS_Test_Suite, gps_test_gpgga_valid);
    FRIEND_TEST(GPS_Test_Suite, gps_test_lock);
    FRIEND_TEST(GPS_Test_Suite, gps_test_message);
};

#endif //ROS_ROBOBUGGY_TRANSISTOR_GPS_BROADCASTER_H
