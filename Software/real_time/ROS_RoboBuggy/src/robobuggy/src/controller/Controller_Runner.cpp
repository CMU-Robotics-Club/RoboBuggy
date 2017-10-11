//
// Created by bhai on 9/15/17.
//

#include "controller/Controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    Controller controller;

    while (ros::ok())
    {
        controller.update_steering_estimate();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
