#include "transistor_serial_messages.h"
#include "serial/serial.h"

#include "low_level/Transistor_LL_Broadcaster.h"

std::string rb_serial_buffer;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Transistor_LL_Broadcaster");
    ros::NodeHandle nh;

    Transistor_LL_Broadcaster broadcaster;

    // Messages
    robobuggy::Brake brake_msg;
    robobuggy::Steering steering_msg;
    robobuggy::Diagnostics diagnostics_msg;
    robobuggy::ENC encoder_msg;

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

    // Constantly read messages from low level
    while(ros::ok()) {
        if (rb_serial.available()) {
            // Read from the serial port
            ROS_INFO_STREAM("Reading messages from low level");
            
            rb_serial_buffer = rb_serial.read(rb_serial.available());
            
            // Validate complete message
            if (rb_serial_buffer[5] != RBSM_FOOTER) {
                // Skip invalid message
                continue;
            }

            // Extract data word from serial message
            uint32_t data = 0;
            data |= (rb_serial_buffer[1] & 0xFF);
            data <<= 8;
            data |= (rb_serial_buffer[2] & 0xFF);
            data <<= 8;
            data |= (rb_serial_buffer[3] & 0xFF);
            data <<= 8;
            data |= (rb_serial_buffer[4] & 0xFF);

            // First byte is serial buffer
            switch((unsigned char)rb_serial_buffer[0]) {
                case RBSM_MID_ENC_RESET_CONFIRM:
                    // Confirmation of encoder reset request received
                    // High level doesn't do anything with this message
                    break;
                case RBSM_MID_MEGA_STEER_ANGLE:
                    // Steering angle
                    steering_msg.steer_angle = (int32_t)data;
                    break;
                case RBSM_MID_MEGA_BRAKE_STATE:
                    // Brake state
                    brake_msg.brake_state = (bool)data;
                    break;
                case DEVICE_ID:
                    diagnostics_msg.device_id = data;
                    break;
                case RBSM_MID_MEGA_TELEOP_BRAKE_COMMAND:
                    brake_msg.brake_cmd_teleop = (bool)data;
                    break;
                case RBSM_MID_MEGA_AUTON_BRAKE_COMMAND:
                    brake_msg.brake_cmd_auton = (bool)data;
                    break;
                case RBSM_MID_MEGA_AUTON_STATE:
                    diagnostics_msg.auton_state = (bool)data;
                    break;
                case RBSM_MID_MEGA_BATTERY_LEVEL:
                    diagnostics_msg.battery_level = data;
                    break;
                case RBSM_MID_MEGA_STEER_FEEDBACK:
                    steering_msg.steer_feedback = (int32_t)data;
                    break;
                case RBSM_MID_ENC_TICKS_RESET:
                    encoder_msg.ticks = data;
                    break;
                case RBSM_MID_ERROR:
                    diagnostics_msg.error = data;
                    break;
                default:
                    break;
            }

            // Add timestamps to messages
            ros::Time timestamp = ros::Time::now();
            brake_msg.header.stamp = timestamp;
            steering_msg.header.stamp = timestamp;
            diagnostics_msg.header.stamp = timestamp;
            encoder_msg.header.stamp = timestamp;

            // Publish messages
            broadcaster.publish_brake_msg(brake_msg);
            broadcaster.publish_steering_msg(steering_msg);
            broadcaster.publish_diagnostics_msg(diagnostics_msg);
            broadcaster.publish_encoder_msg(encoder_msg);
        }
        ros::spinOnce();
    }
}
