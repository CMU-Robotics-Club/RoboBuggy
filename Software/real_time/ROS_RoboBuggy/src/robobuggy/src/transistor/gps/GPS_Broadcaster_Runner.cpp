//
// Created by bhai on 9/29/17.
//

#include "transistor/gps/GPS_Broadcaster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, GPS_Broadcaster::NODE_NAME);
    ros::NodeHandle nh;

    GPS_Broadcaster broadcaster;
    int err = broadcaster.handle_serial_messages();

    return err;
}
