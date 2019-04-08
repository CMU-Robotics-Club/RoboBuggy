#include "dynamixel_sdk.h"
#include "ros/ros.h"
#include "robobuggy/Command.h"
#include "robobuggy/Feedback.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <csignal>

static uint16_t desired_angle = 1200;
static dynamixel::PortHandler *port_handler;
static dynamixel::PacketHandler *packet_handler;
static const uint8_t DXL_ID = 0x01;

void check_dxl_result(int id, uint8_t dxl_err, int16_t dxl_comm_res, std::string step, dynamixel::PacketHandler *ph)
{
    if (dxl_err || dxl_comm_res != COMM_SUCCESS)
    {
        std::cerr << "Error performing " << step << " on dxl " << id << ". err=" << ph->getRxPacketError(dxl_err) << " cr=" << ph->getTxRxResult(dxl_comm_res) << std::endl;
    }
}

void controller_cb(const robobuggy::Command::ConstPtr& msg)
{
    double data = msg->steer_cmd_rad;
    uint16_t steer_center = 1200;
    int steer_angle_deg = data*180/M_PI;
    desired_angle = steer_angle_deg * 11 + steer_center;

    ROS_INFO("Set angle to %d", desired_angle);

}

void rc_cb(const robobuggy::Feedback::ConstPtr& msg)
{
    int32_t data = msg->steer_angle;
    uint16_t steer_angle = (uint16_t) data;
    desired_angle = steer_angle;
}

void send_to_dyn()
{
    uint8_t dxl_err;
    int16_t comm_res = packet_handler->write2ByteTxRx(port_handler, DXL_ID, (uint8_t)30, desired_angle, &dxl_err);
    check_dxl_result(DXL_ID, dxl_err, comm_res, "wut", packet_handler);
    ROS_INFO("Sent angle %d", desired_angle);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Dynamixel_Last_ditch");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Subscriber controller_sub = nh.subscribe<robobuggy::Command>("Command", 1000, &controller_cb);
    ros::Subscriber rc_sub = nh.subscribe<robobuggy::Feedback>("Feedback", 10, &rc_cb);

    port_handler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    packet_handler = dynamixel::PacketHandler::getPacketHandler(1.0);


    if (port_handler->openPort())
    {
        ROS_INFO("Opened Dyanmixel Port");
    }
    else
    {
        ROS_ERROR("Couldn't open port");
        return 1;
    }

    if (port_handler->setBaudRate(1000000))
    {
        ROS_INFO("Set Baud Rate");
    }
    else
    {
        ROS_ERROR("Couldn't set baud rate");
        return 1;
    }

    uint8_t dxl_err = 0;
    int16_t dxl_comm_res = 0;

    dxl_comm_res = packet_handler->write1ByteTxRx(port_handler, DXL_ID, 0x18, 1, &dxl_err);
    check_dxl_result(DXL_ID, dxl_err, dxl_comm_res, "wut", packet_handler);

    while (ros::ok())
    {
        ros::spinOnce();
        send_to_dyn();
        loop_rate.sleep();
    }

    dxl_comm_res = packet_handler->write1ByteTxRx(port_handler, DXL_ID, 0x18, 0, &dxl_err);
    check_dxl_result(DXL_ID, dxl_err, dxl_comm_res, "wut", packet_handler);
    port_handler->closePort();

    return 0;
}
