#include "low_level/Transistor_LL_Broadcaster.h"

Transistor_LL_Broadcaster::Transistor_LL_Broadcaster()
{
    brake_pub = nh.advertise<robobuggy::Brake>("Brake", 1000);
    steering_pub = nh.advertise<robobuggy::Steering>("Steering", 1000);
    diagnostics_pub = nh.advertise<robobuggy::Diagnostics>("Diagnostics", 1000);
    encoder_pub = nh.advertise<robobuggy::ENC>("Encoder", 1000);
}

void Transistor_LL_Broadcaster::publish_brake_msg(robobuggy::Brake msg)
{
    brake_pub.publish(msg);
}
void Transistor_LL_Broadcaster::publish_steering_msg(robobuggy::Steering msg)
{
    steering_pub.publish(msg);
}
void Transistor_LL_Broadcaster::publish_diagnostics_msg(robobuggy::Diagnostics msg)
{
    diagnostics_pub.publish(msg);
}
void Transistor_LL_Broadcaster::publish_encoder_msg(robobuggy::ENC msg)
{
    encoder_pub.publish(msg);
}
