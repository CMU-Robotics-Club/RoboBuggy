//
// Created by bhai on 9/29/17.
//

#include "transistor/gps/GPS_Broadcaster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, GPS_Broadcaster::NODE_NAME);
    ros::NodeHandle nh;

    GPS_Broadcaster broadcaster;
    while (ros::ok()) {
        int result = broadcaster.read_gps_message(); 
        if (result)
        {
            return result;
        }
    }
    
    return 0;
}
