//
// Created by bhai on 9/15/17.
//
#include "encoder/ENC_Broadcaster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ENC_Broadcaster");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ENC_Broadcaster broadcaster;


    // infinitely spin sending dummy encoder messages
    while (ros::ok())
    {
        broadcaster.publish_spoof_message();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
