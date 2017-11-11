#include "low_level/Transistor_LL_Broadcaster.h"
#include "transistor_serial_messages.h"
#include "serial/serial.h"

std::string rb_serial_buffer;
const std::string Transistor_LL_Broadcaster::NODE_NAME = "Transistor_LL_Broadcaster";
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

void Transistor_LL_Broadcaster::parse_serial_msg(std::string serial_msg) {

    // Verify that we actually read a full message
    if ((serial_msg[5] & 0xFF) != RBSM_FOOTER)
    {
        ROS_ERROR("Error parsing from serial, message doesn't end in footer!");
        // skip it
        return;
    }


    uint32_t data = 0;
    data |= (serial_msg[1] & 0xFF);
    data <<= 8;
    data |= (serial_msg[2] & 0xFF);
    data <<= 8;
    data |= (serial_msg[3] & 0xFF);
    data <<= 8;
    data |= (serial_msg[4] & 0xFF);

    // First byte is serial buffer
    switch((unsigned char)serial_msg[0]) {
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
    publish_brake_msg(brake_msg);
    publish_steering_msg(steering_msg);
    publish_diagnostics_msg(diagnostics_msg);
    publish_encoder_msg(encoder_msg);

}

int Transistor_LL_Broadcaster::handle_serial_messages() {
    // Initialize serial communication
    std::string serial_port;
    int serial_baud;
    if (!nh.getParam(NODE_NAME + "/serial_port", serial_port)) {
        ROS_INFO_STREAM("Serial port parameter not found, using default");
        serial_port = "/dev/ttyACM1"; // Default value
    }
    if (!nh.getParam(NODE_NAME + "/serial_baud", serial_baud)) {
        ROS_INFO_STREAM("Serial baud rate parameter not found, using default");
        serial_baud = 57600; // Default value
    }

    // Initialize serial communication
    serial::Serial rb_serial;
    try {
        rb_serial.setPort(serial_port);
        rb_serial.setBaudrate(serial_baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        rb_serial.setTimeout(to);
        rb_serial.open();
    }
    catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port");
        ROS_ERROR_STREAM(serial_port);
        return -1;
    }

    if (rb_serial.isOpen()) {
        ROS_INFO_STREAM("Serial port initialized");
    }
    else {
        return -1;
    }

    rb_serial.flush();

    while(ros::ok()) {
        if (rb_serial.available()) {
            // Read from the serial port
           
            rb_serial_buffer = rb_serial.read(rb_serial.available());
            
            int msg_len = 6;

            for (int i = 0; i < rb_serial_buffer.length(); i++) {
                if (rb_serial_buffer[i] == RBSM_FOOTER) {
                    // We've found the end of a message from low level
                    if ((i+msg_len) < rb_serial_buffer.length()) {
                        // There is a full message in the buffer, read it
                        // Otherwise, drop it
                        std::string current_msg = rb_serial_buffer.substr(i+1, msg_len);
                        
                        parse_serial_msg(current_msg);
                    }
                }
            }
        }
        ros::spinOnce();
    }
}
