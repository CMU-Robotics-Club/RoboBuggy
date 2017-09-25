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

    msg.ticks = spoof_ticks;
    spoof_ticks++;

    enc_pub.publish(msg);
}


