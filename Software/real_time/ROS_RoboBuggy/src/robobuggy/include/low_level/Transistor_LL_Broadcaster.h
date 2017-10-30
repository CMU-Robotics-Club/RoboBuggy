#ifndef _TRANSISTOR_LL_BROADCASTER_H_
#define _TRANSISTOR_LL_BROADCASTER_H_

#include <ros/ros.h>
#include <serial/serial.h>

#include <robobuggy/Steering.h>
#include <robobuggy/Brake.h>
#include <robobuggy/Diagnostics.h>
#include <robobuggy/ENC.h>
#include <robobuggy/SteeringCommand.h>

class Transistor_LL_Broadcaster
{
public:
    Transistor_LL_Broadcaster();

    static const std::string NODE_NAME;
    
    int handle_serial_messages();
    static void send_steering_command(const robobuggy::SteeringCommand::ConstPtr& msg);

private:
    ros::Publisher steering_pub;
    ros::Publisher brake_pub;
    ros::Publisher diagnostics_pub;
    ros::Publisher encoder_pub;

    ros::Subscriber steering_sub;

    robobuggy::Brake brake_msg;
    robobuggy::Steering steering_msg;
    robobuggy::Diagnostics diagnostics_msg;
    robobuggy::ENC encoder_msg;

    ros::NodeHandle nh;

    serial::Serial rb_serial;
 
    void publish_brake_msg(robobuggy::Brake msg);
    void publish_steering_msg(robobuggy::Steering msg);
    void publish_diagnostics_msg(robobuggy::Diagnostics msg);
    void publish_encoder_msg(robobuggy::ENC msg);
    void parse_serial_msg(std::string serial_msg);
};

#endif /* _TRANSISTOR_LL_BROADCASTER_H_ */
