#include "transistor_serial_messages.h"
#include "serial/serial.h"

#include "low_level/Transistor_LL_Broadcaster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, Transistor_LL_Broadcaster::NODE_NAME);
    ros::NodeHandle nh;

    Transistor_LL_Broadcaster broadcaster;
    int err = broadcaster.handle_serial_messages();

    return err;
}
