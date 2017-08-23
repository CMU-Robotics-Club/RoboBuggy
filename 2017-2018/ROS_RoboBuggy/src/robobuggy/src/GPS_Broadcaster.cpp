#include "ros/ros.h"
#include "robobuggy/GPS.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_Broadcaster");

    ros::NodeHandle nh;

    ros::Publisher gps_pub = nh.advertise<robobuggy::GPS>("GPS", 1000);

    ros::Rate loop_rate(10);

    double start_lat = 40.442808;
    double start_lon = -79.943013;
    double increment = 0.00001;

    // Infinitely spin sending dummy GPS messages
    while(ros::ok()) 
    {
        robobuggy::GPS msg;

        int random = rand() % 10;
        double new_lat = start_lat + increment * random;
        double new_lon = start_lon + increment * random;

        msg.Lat_deg = new_lat;
        msg.Long_deg = new_lon;

        gps_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
