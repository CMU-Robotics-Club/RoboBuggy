#include "controller/Controller.h"

void Controller::IMU_Callback(const robobuggy::IMU::ConstPtr& msg)
{
    ROS_INFO("Received IMU message with accelerations: (%f, %f, %f)", msg->X_Accel, msg->Y_Accel, msg->Z_Accel);
}

void Controller::GPS_Callback(const robobuggy::GPS::ConstPtr& msg)
{
    ROS_INFO("Received GPS message with coordinates: (%fm, %fm) / (%fm, %fm)", msg->Lat_m, msg->Long_m, msg->Lat_deg, msg->Long_deg);
}

void Controller::ENC_Callback(const robobuggy::ENC::ConstPtr& msg)
{
    ROS_INFO("Received ENC message with coordinates: (%fm, %fm, %fm, %fm)", msg->distance_m, msg->velocity_ms, msg->acceleration_mss, msg->raw_data_word);
}

Controller::Controller()
{
    imu_sub = nh.subscribe<robobuggy::IMU>("IMU", 1000, IMU_Callback);
    gps_sub = nh.subscribe<robobuggy::GPS>("GPS", 1000, GPS_Callback);
    enc_sub = nh.subscribe<robobuggy::ENC>("ENC", 1000, ENC_Callback);

    ROS_INFO("Finished setting up subscribers\n");
}
