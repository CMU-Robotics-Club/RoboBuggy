//
// Created by bhai on 9/15/17.
//
#include "gps/GPS_Broadcaster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, GPS_Broadcaster::NODE_NAME);
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    GPS_Broadcaster broadcaster;

    // Infinitely spin sending dummy GPS messages
    while(ros::ok())
    {
        broadcaster.publish_spoof_message();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
