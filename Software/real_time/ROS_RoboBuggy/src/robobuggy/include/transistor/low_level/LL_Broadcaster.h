#ifndef _TRANSISTOR_LL_BROADCASTER_H_
#define _TRANSISTOR_LL_BROADCASTER_H_

#include <ros/ros.h>
#include <serial/serial.h>

#include <robobuggy/Steering.h>
#include <robobuggy/Brake.h>
#include <robobuggy/Diagnostics.h>
#include <robobuggy/ENC.h>

class LL_Broadcaster 
{
public:
    LL_Broadcaster();
    static const std::string NODE_NAME;
    
    int handle_serial_messages();

private:
    ros::NodeHandle nh;
    
    ros::Publisher steering_pub;
    ros::Publisher brake_pub;
    ros::Publisher diagnostics_pub;
    ros::Publisher encoder_pub;

    robobuggy::Brake brake_msg;
    robobuggy::Steering steering_msg;
    robobuggy::Diagnostics diagnostics_msg;
    robobuggy::ENC encoder_msg;
    
    serial::Serial rb_serial;
    std::string serial_port;
    int serial_baud;
    std::string ll_serial_buffer;

    void parse_serial_msg(std::string serial_msg);
};

#endif /* _TRANSISTOR_LL_BROADCASTER_H_ */
