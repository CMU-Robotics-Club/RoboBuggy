#include "localizer/Localizer.h"

static void Localizer::ENC_Callback(const robobuggy::ENC:ConstPtr& msg)
{

}

static void Localizer::GPS_Callback(const robobuggy::GPS::ConstPtr& msg)
{

}

static void Localizer::IMU_Callback(const robobuggy::IMU::ConstPtr& msg)
{

}

Localizer::Localizer()
{
    imu_sub = nh.subscribe<robobuggy::IMU>("IMU", 1000, IMU_Callback);
    gps_sub = nh.subscribe<robobuggy::GPS>("GPS", 1000, GPS_Callback);
    enc_sub = nh.subscribe<robobuggy::ENC>("ENC", 1000, ENC_Callback);
}

void Localizer::update_position_estimate()
{

}