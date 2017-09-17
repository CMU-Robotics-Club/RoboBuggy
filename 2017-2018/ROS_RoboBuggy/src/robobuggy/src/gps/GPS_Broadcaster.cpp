#include "gps/GPS_Broadcaster.h"

const std::string GPS_Broadcaster::NODE_NAME = "GPS_Broadcaster";
GPS_Broadcaster::GPS_Broadcaster() : increment(0.00001)
{
    gps_pub = nh.advertise<robobuggy::GPS>("GPS", 1000);

    start_lat = 0.0;
    start_lon = 0.0;

    // Get parameters start lat and start lon
    if (!nh.getParam(NODE_NAME + "/start_latitude", start_lat))
    {
        ROS_ERROR("Couldn't retrieve start latitude parameter from parameter server");
    }
    if (!nh.getParam(NODE_NAME + "/start_longitude", start_lon))
    {
        ROS_ERROR("Couldn't retrieve start longitude parameter from parameter server");
    }
}

void GPS_Broadcaster::publish_spoof_message()
{
    robobuggy::GPS msg;

    int random = rand() % 10;
    float new_lat = start_lat + increment * random;
    float new_lon = start_lon + increment * random;

    msg.Lat_deg = new_lat;
    msg.Long_deg = new_lon;

    gps_pub.publish(msg);
}


