//
// Created by bhai on 9/29/17.
//

#include "gps/Transistor_GPS_Broadcaster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, Transistor_GPS_Broadcaster::NODE_NAME);
    ros::NodeHandle nh;

    Transistor_GPS_Broadcaster broadcaster;
    int err = broadcaster.handle_serial_messages();

    return err;
}