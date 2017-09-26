#include "localizer/Localizer.h"

void Localizer::ENC_Callback(const robobuggy::ENC::ConstPtr &msg)
{

}

void Localizer::GPS_Callback(const robobuggy::GPS::ConstPtr &msg)
{

}

void Localizer::IMU_Callback(const robobuggy::IMU::ConstPtr &msg)
{

}

Localizer::Localizer()
{
    imu_sub = nh.subscribe<robobuggy::IMU>("IMU", 1000, IMU_Callback);
    gps_sub = nh.subscribe<robobuggy::GPS>("GPS", 1000, GPS_Callback);
    enc_sub = nh.subscribe<robobuggy::ENC>("ENC", 1000, ENC_Callback);

    R = MatrixXd(5,5);
}

void Localizer::update_position_estimate()
{

}

// TODO create runner file once we've filled the rest of this out
int main() {

}