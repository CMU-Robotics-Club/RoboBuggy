#include "transistor/controller/Controller.h"

void Controller::IMU_Callback(const robobuggy::IMU::ConstPtr& msg)
{
    ROS_INFO("Received IMU message with accelerations: (%f, %f, %f)", msg->X_Accel, msg->Y_Accel, msg->Z_Accel);
}

void Controller::GPS_Callback(const robobuggy::GPS::ConstPtr& msg)
{
    ROS_INFO("Received GPS message with coordinates: (%fm, %fm) / (%fm, %fm)", msg->Lat_m, msg->Long_m, msg->Lat_deg, msg->Long_deg);
}

void Controller::Encoder_Callback(const robobuggy::Encoder::ConstPtr& msg)
{
    ROS_INFO("Received Encoder message with ticks: %d", msg->ticks);
}

const std::string Controller::NODE_NAME = "Controller";
Controller::Controller()
{
    imu_sub = nh.subscribe<robobuggy::IMU>("IMU", 1000, IMU_Callback);
    gps_sub = nh.subscribe<robobuggy::GPS>("GPS", 1000, GPS_Callback);
    encoder_sub = nh.subscribe<robobuggy::Encoder>("Encoder", 1000, Encoder_Callback);

    ROS_INFO("Finished setting up subscribers\n");
}
