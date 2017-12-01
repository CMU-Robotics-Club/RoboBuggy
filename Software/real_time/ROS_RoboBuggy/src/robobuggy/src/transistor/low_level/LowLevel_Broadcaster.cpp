#include "transistor/low_level/LowLevel_Broadcaster.h"
#include "transistor/transistor_serial_messages.h"
#include "serial/serial.h"

const std::string LowLevel_Broadcaster::NODE_NAME = "LowLevel_Broadcaster";

LowLevel_Broadcaster::LowLevel_Broadcaster()
{
    // Register publishers
    feedback_pub = nh.advertise<robobuggy::Feedback>("Feedback", 1000);
    diagnostics_pub = nh.advertise<robobuggy::Diagnostics>("Diagnostics", 1000);
    encoder_pub = nh.advertise<robobuggy::Encoder>("Encoder", 1000);

    // Register callback for serial command
    command_sub = nh.subscribe<robobuggy::Command>("Command", 1000, &LowLevel_Broadcaster::send_command, this);
}

void LowLevel_Broadcaster::publish_feedback_msg(robobuggy::Feedback msg)
{
    feedback_pub.publish(msg);
}
void LowLevel_Broadcaster::publish_diagnostics_msg(robobuggy::Diagnostics msg)
{
    diagnostics_pub.publish(msg);
}
void LowLevel_Broadcaster::publish_encoder_msg(robobuggy::Encoder msg)
{
    encoder_pub.publish(msg);
}

void LowLevel_Broadcaster::parse_serial_msg(std::string serial_msg)
{
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
        case RBSM_MID_MEGA_STEER_FEEDBACK:
            feedback_msg.steer_angle = (int32_t)data;
            break;
        case RBSM_MID_MEGA_BRAKE_STATE:
            feedback_msg.brake_state = (bool)data;
            break;
        case RBSM_MID_MEGA_TELEOP_BRAKE_COMMAND:
            diagnostics_msg.teleop_brake_cmd_status = (bool)data;
            break;
        case RBSM_MID_MEGA_AUTON_BRAKE_COMMAND:
            diagnostics_msg.auton_brake_cmd_status = (bool)data;
            break;
        case DEVICE_ID:
            diagnostics_msg.device_id = data;
            break;
        case RBSM_MID_MEGA_AUTON_STATE:
            diagnostics_msg.auton_state = (bool)data;
            break;
        case RBSM_MID_MEGA_BATTERY_LEVEL:
            diagnostics_msg.battery_level = data;
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
    feedback_msg.header.stamp = timestamp;
    diagnostics_msg.header.stamp = timestamp;
    encoder_msg.header.stamp = timestamp;

    // Publish messages
    publish_feedback_msg(feedback_msg);
    publish_diagnostics_msg(diagnostics_msg);
    publish_encoder_msg(encoder_msg);

}

void LowLevel_Broadcaster::send_command(const robobuggy::Command::ConstPtr& msg) {
    // Construct and send the steering command
    uint8_t serial_msg[6];
    int data = msg->steer_cmd;

    ROS_INFO("Steering command: %d\n", data);
    serial_msg[0] = RBSM_MID_MEGA_STEER_COMMAND;
    serial_msg[1] = (data >> 24) & 0xFF;
    serial_msg[2] = (data >> 16) & 0xFF;
    serial_msg[3] = (data >> 8) & 0xFF;
    serial_msg[4] = data & 0xFF;
    serial_msg[5] = RBSM_FOOTER;

    if (rb_serial.isOpen()) {
        rb_serial.write(serial_msg, 6);
    }

    // Construct and send the brake command
    data = msg->brake_cmd;

    serial_msg[0] = RBSM_MID_MEGA_AUTON_BRAKE_COMMAND;
    serial_msg[1] = (data >> 24) & 0xFF;
    serial_msg[2] = (data >> 16) & 0xFF;
    serial_msg[3] = (data >> 8) & 0xFF;
    serial_msg[4] = data & 0xFF;
    serial_msg[5] = RBSM_FOOTER;

    if (rb_serial.isOpen()) {
        rb_serial.write(serial_msg, 6);
    }
}

int LowLevel_Broadcaster::handle_serial_messages() {
    // Initialize serial communication
    std::string serial_port;
    int serial_baud;

    if (!nh.getParam(NODE_NAME + "/serial_port", serial_port)) {
        ROS_INFO("Serial port parameter not found, using default");
        serial_port = "/dev/ttyACM1"; // Default value
    }
    if (!nh.getParam(NODE_NAME + "/serial_baud", serial_baud)) {
        ROS_INFO("Serial baud rate parameter not found, using default");
        serial_baud = 57600; // Default value
    }

    // Initialize serial communication
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

    // Clear the serial buffer
    rb_serial.flush();

    while(ros::ok()) {
        if (rb_serial.available()) {
            // Read from the serial port
            ll_serial_buffer = rb_serial.read(rb_serial.available());

            int msg_len = 6;

            for (int i = 0; i < ll_serial_buffer.length(); i++) {
                if (ll_serial_buffer[i] == RBSM_FOOTER) {
                    // We've found the end of a message from low level
                    if ((i+msg_len) < ll_serial_buffer.length()) {
                        // There is a full message in the buffer, read it
                        // Otherwise, drop it
                        std::string current_msg = ll_serial_buffer.substr(i+1, msg_len);

                        parse_serial_msg(current_msg);
                    }
                }
            }
        }
        ros::spinOnce();
    }
}
