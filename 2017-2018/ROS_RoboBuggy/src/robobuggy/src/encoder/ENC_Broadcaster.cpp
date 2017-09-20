//
// Created by bhai on 8/17/17.
//

#include "encoder/ENC_Broadcaster.h"

ENC_Broadcaster::ENC_Broadcaster()
{
    enc_pub = nh.advertise<robobuggy::ENC>("ENC", 1000);
}

void ENC_Broadcaster::publish_spoof_message()
{
    robobuggy::ENC msg;

    msg.distance_m = spoof_distance;
    msg.velocity_ms = 1;
    msg.acceleration_mss = 0;
    msg.raw_data_word = 1337;

    spoof_distance += 0.1;

    enc_pub.publish(msg);
}


