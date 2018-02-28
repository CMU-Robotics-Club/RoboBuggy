#include "transistor/transistor_serial_messages.h"
#include "serial/serial.h"

#include "transistor/low_level/LowLevel_Broadcaster.h"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, LowLevel_Broadcaster::NODE_NAME);
    ros::NodeHandle nh;

    LowLevel_Broadcaster broadcaster;
    int err = broadcaster.handle_serial_messages();

    return err;
}
