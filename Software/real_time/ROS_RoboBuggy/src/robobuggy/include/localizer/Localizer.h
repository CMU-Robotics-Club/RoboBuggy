#include "ros/ros.h"

#include <stdio.h>
#include <robobuggy/IMU.h>
#include <robobuggy/GPS.h>
#include <robobuggy/ENC.h>
#include <robobuggy/Pose.h>
#include <geodesy/utm.h>
#include <string>
#include <iostream>

#include <eigen3/Eigen/Dense>

using Eigen::Matrix;

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

    Matrix<double, 5, 5> A;
    Matrix<double, 5, 1> x;
    Matrix<double, 5, 5> R;
    Matrix<double, 5, 5> P;
    Matrix<double, 3, 3> Q_GPS;
    Matrix<double, 1, 1> Q_Encoder;
    Matrix<double, 3, 5> C_GPS;
    Matrix<double, 1, 5> C_Encoder;

    void update_position_estimate();
    void update_motion_model();

    void init_R();
    void init_P();
    void init_Q_GPS();
    void init_Q_Encoder();
    void init_C_GPS();
    void init_C_Encoder();
    void init_x();
};