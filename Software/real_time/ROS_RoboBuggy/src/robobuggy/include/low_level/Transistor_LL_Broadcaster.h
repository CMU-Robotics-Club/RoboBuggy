#ifndef _TRANSISTOR_LL_BROADCASTER_H_
#define _TRANSISTOR_LL_BROADCASTER_H_

#include <ros/ros.h>
#include <serial/serial.h>

#include <robobuggy/Feedback.h>
#include <robobuggy/Diagnostics.h>
#include <robobuggy/ENC.h>
#include <robobuggy/Command.h>

class Transistor_LL_Broadcaster
{
public:
    Transistor_LL_Broadcaster();

    static const std::string NODE_NAME;
    
    int handle_serial_messages();
    void send_command(const robobuggy::Command::ConstPtr& msg);

private:
    ros::Publisher feedback_pub;
    ros::Publisher diagnostics_pub;
    ros::Publisher encoder_pub;

    ros::Subscriber command_sub;

    robobuggy::Feedback feedback_msg;
    robobuggy::Diagnostics diagnostics_msg;
    robobuggy::ENC encoder_msg;

    ros::NodeHandle nh;

    serial::Serial rb_serial;
 
    void publish_feedback_msg(robobuggy::Feedback msg);
    void publish_diagnostics_msg(robobuggy::Diagnostics msg);
    void publish_encoder_msg(robobuggy::ENC msg);
    void parse_serial_msg(std::string serial_msg);
};

#endif /* _TRANSISTOR_LL_BROADCASTER_H_ */
