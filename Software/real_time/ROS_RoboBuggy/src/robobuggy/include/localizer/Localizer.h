#include "ros/ros.h"

#include <geodesy/utm.h>
#include <string>
#include <iostream>
#include <math.h>

#include <eigen3/Eigen/Dense>

#include <robobuggy/IMU.h>
#include <robobuggy/GPS.h>
#include <robobuggy/ENC.h>
#include <robobuggy/Pose.h>

using Eigen::Matrix;

class Localizer
{
public:
    Localizer();
    static void IMU_Callback(const robobuggy::IMU::ConstPtr& msg);
    static void GPS_Callback(const robobuggy::GPS::ConstPtr& msg);
    static void ENC_Callback(const robobuggy::ENC::ConstPtr& msg);
private:
    geodesy::UTMPoint current_position_utm;
    double current_encoder_ticks;
    double prev_encoder_ticks;
    double current_heading;
    double current_steering_angle;
    double WHEELBASE_M;

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
    void update_motion_model(double dt);
    double clamp_angle(double theta);

    void init_R();
    void init_P();
    void init_Q_GPS();
    void init_Q_Encoder();
    void init_C_GPS();
    void init_C_Encoder();
    void init_x();
};