#include "ros/ros.h"
#include "robobuggy/GPS.h"

#include <stdio.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_Broadcaster");

    ros::NodeHandle nh;

    ros::Publisher gps_pub = nh.advertise<robobuggy::GPS>("GPS", 1000);

    ros::Rate loop_rate(10);

    // Infinitely spin sending dummy GPS messages
    while(ros::ok()) 
    {
        robobuggy::GPS msg;

        msg.Lat_m = 5.0;
        msg.Long_m = 15.0;

        gps_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
