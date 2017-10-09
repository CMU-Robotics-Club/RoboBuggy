#include "transistor/transistor_serial_messages.h"
#include "serial/serial.h"

#include "transistor/low_level/LL_Broadcaster.h"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, LL_Broadcaster::NODE_NAME);
    ros::NodeHandle nh;

    LL_Broadcaster broadcaster;
    int err = broadcaster.handle_serial_messages();

    return err;
}
