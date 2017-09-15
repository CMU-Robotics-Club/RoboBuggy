//
// Created by bhai on 8/17/17.
//

#include <ros/init.h>
#include <ros/node_handle.h>
#include <robobuggy/ENC.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ENC_Broadcaster");

    ros::NodeHandle nh;

    ros::Publisher enc_pub = nh.advertise<robobuggy::ENC>("ENC", 1000);

    ros::Rate loop_rate(10);

    float distance = 0.0;

    // infinitely spin sending dummy encoder messages
    while (ros::ok())
    {
        robobuggy::ENC msg;

        msg.distance_m = distance;
        msg.velocity_ms = 1;
        msg.acceleration_mss = 0;
        msg.raw_data_word = 1337;

        distance += 0.1;

        enc_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
