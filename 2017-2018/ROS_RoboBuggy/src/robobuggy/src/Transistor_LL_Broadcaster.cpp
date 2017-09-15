#include "ros/ros.h"

#include "transistor_serial_messages.h"
#include "serial/serial.h"

std::string NODE_NAME = "Transistor Low-Level Broadcaster";

std::string rb_serial_buffer;

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle nh;

    // Initialize serial communication
    serial::Serial rb_serial;
    try {
        rb_serial.setPort(RBSM_SERIAL_PORT);
        rb_serial.setBaudrate(RBSM_SERIAL_BAUD);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        rb_serial.setTimeout(to);
        rb_serial.open();
    }
    catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (rb_serial.isOpen()) {
        ROS_INFO_STREAM("Serial port initialized");
    }
    else {
        return -1;
    }

    while(ros::ok()) {
        if (rb_serial.available()) {
            // Read from the serial port
            ROS_INFO_STREAM("Reading messages from low level");
            
            rb_serial_buffer = rb_serial.read(rb_serial.available());

            // First byte is serial buffer
            switch(rb_serial_buffer[0]) {
                case RBSM_MID_ENC_RESET_CONFIRM:
                    // Confirmation of encoder reset request received
                    break;
                case RBSM_MID_MEGA_STEER_ANGLE:
                    // Steering angle
                    break;
                case RBSM_MID_MEGA_BRAKE_STATE:
                    // Brake state
                    break;
                case DEVICE_ID:
                    break;
                case RBSM_MID_MEGA_TELEOP_BRAKE_COMMAND:
                    break;
                default:
                    break;
            }
        }

        ros::spinOnce();
    }

    return 0;
}
