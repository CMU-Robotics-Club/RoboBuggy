#include "ros/ros.h"
#include "robobuggy/IMU.h"

#include <stdio.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IMU_Broadcaster");

    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<robobuggy::IMU>("IMU", 1000);

    ros::Rate loop_rate(10);

    // Infinitely spin sending dummy IMU messages
    while(ros::ok())
    {
        robobuggy::IMU msg;

        msg.X_Accel = 1.5;
        msg.Y_Accel = 2.5;
        msg.Z_Accel = 1.5;

        imu_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
