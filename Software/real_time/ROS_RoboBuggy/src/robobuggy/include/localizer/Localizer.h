#include "ros/ros.h"

#include <stdio.h>
#include <robobuggy/IMU.h>
#include <robobuggy/GPS.h>
#include <robobuggy/ENC.h>

#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;

class Localizer
{
public:
    Localizer();
    static void IMU_Callback(const robobuggy::IMU::ConstPtr& msg);
    static void GPS_Callback(const robobuggy::GPS::ConstPtr& msg);
    static void ENC_Callback(const robobuggy::ENC::ConstPtr& msg);
private:
    double current_gps_lat;
    double current_gps_lon;
    double current_encoder_ticks;
    double prev_encoder_ticks;
    double current_imu_heading;

    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber enc_sub;

    MatrixXd motion_model;
    MatrixXd R;
    MatrixXd P;
    MatrixXd Q_GPS;
    MatrixXd Q_Encoder;
    MatrixXd C_GPS;
    MatrixXd C_Encoder;

    void update_position_estimate();
};