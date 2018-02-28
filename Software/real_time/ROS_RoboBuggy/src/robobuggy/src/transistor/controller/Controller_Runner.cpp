//
// Created by bhai on 9/15/17.
//

#include "transistor/controller/Controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;

    Controller controller;

    ros::spin();

    return 0;
}
