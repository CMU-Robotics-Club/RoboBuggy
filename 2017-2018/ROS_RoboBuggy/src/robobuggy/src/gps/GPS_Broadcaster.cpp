#include "ros/ros.h"
#include "robobuggy/GPS.h"

std::string NODE_NAME = "GPS_Broadcaster";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle nh;

    ros::Publisher gps_pub = nh.advertise<robobuggy::GPS>("GPS", 1000);

    float start_lat = 0.0;
    float start_lon = 0.0;
    float increment = 0.00001;

    // Get parameters start lat and start lon
    if (!nh.getParam(NODE_NAME + "/start_latitude", start_lat))
    {
        ROS_ERROR("Couldn't retrieve start latitude parameter from parameter server");
    }
    if (!nh.getParam(NODE_NAME + "/start_longitude", start_lon))
    {
        ROS_ERROR("Couldn't retrieve start longitude parameter from parameter server");
    }

    ros::Rate loop_rate(10);

    // Infinitely spin sending dummy GPS messages
    while(ros::ok()) 
    {
        robobuggy::GPS msg;

        int random = rand() % 10;
        float new_lat = start_lat + increment * random;
        float new_lon = start_lon + increment * random;

        msg.Lat_deg = new_lat;
        msg.Long_deg = new_lon;

        gps_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
