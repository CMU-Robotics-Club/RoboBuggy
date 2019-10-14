//
// Created by bhai on 9/29/17.
//

#include "transistor/gps/GPS_Broadcaster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, GPS_Broadcaster::NODE_NAME);
    ros::NodeHandle nh;
    ros::Rate loop_rate(10); //run at 10 hz

    GPS_Broadcaster broadcaster;
    int err = broadcaster.initialize_hardware();
    if (err) {
        return err;
    }

    while (ros::ok()) {
        broadcaster.read_gps_message();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
