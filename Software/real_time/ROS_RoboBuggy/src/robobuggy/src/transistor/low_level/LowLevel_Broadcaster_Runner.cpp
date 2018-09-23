#include "transistor/transistor_serial_messages.h"
#include "serial/serial.h"

#include "transistor/low_level/LowLevel_Broadcaster.h"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, LowLevel_Broadcaster::NODE_NAME);
    ros::NodeHandle nh;
    ros::Rate rate(15); // run at 15 hz, fast enough to catch everything but slow enough to not clog the system

    LowLevel_Broadcaster broadcaster;
    int err = broadcaster.initialize_hardware();
    if (err)
    {
        return err; 
    }

    while (ros::ok()) 
    {
        broadcaster.read_msgs_from_hardware();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
