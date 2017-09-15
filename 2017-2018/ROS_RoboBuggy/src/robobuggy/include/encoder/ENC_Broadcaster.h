//
// Created by bhai on 9/15/17.
//

#ifndef ROS_ROBOBUGGY_ENC_BROADCASTER_H
#define ROS_ROBOBUGGY_ENC_BROADCASTER_H

#include <ros/init.h>
#include <ros/node_handle.h>
#include <robobuggy/ENC.h>

class ENC_Broadcaster
{
public:
    ENC_Broadcaster();
    void publish_spoof_message();

private:
    ros::Publisher enc_pub;
    ros::NodeHandle nh;
    float spoof_distance = 0.0;
};

#endif //ROS_ROBOBUGGY_ENC_BROADCASTER_H
