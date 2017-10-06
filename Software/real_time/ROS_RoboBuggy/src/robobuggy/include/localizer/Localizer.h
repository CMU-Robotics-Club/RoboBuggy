#include "ros/ros.h"

#include <geodesy/utm.h>
#include <string>
#include <iostream>
#include <cmath>
#include <sys/time.h>

#include <eigen3/Eigen/Dense>

#include <robobuggy/IMU.h>
#include <robobuggy/GPS.h>
#include <robobuggy/ENC.h>
#include <robobuggy/Pose.h>
#include <robobuggy/Steering.h>

using Eigen::Matrix;
using Eigen::MatrixXd;

class Localizer
{
public:
    Localizer();
    void IMU_Callback(const robobuggy::IMU::ConstPtr& msg);
    void GPS_Callback(const robobuggy::GPS::ConstPtr& msg);
    void ENC_Callback(const robobuggy::ENC::ConstPtr& msg);
    void Steering_Callback(const robobuggy::Steering::ConstPtr& msg);
    void update_position_estimate();
private:
    geodesy::UTMPoint prev_position_utm;
    double prev_encoder_ticks;
    double current_steering_angle;
    double WHEELBASE_M;
    long int previous_update_time_ms;
    long int prev_encoder_time;

    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber enc_sub;
    ros::Subscriber steering_sub;

    Matrix<double, 5, 5> A;
    Matrix<double, 5, 1> x;
    Matrix<double, 5, 1> x_hat;
    Matrix<double, 5, 5> R;
    Matrix<double, 5, 5> P;
    Matrix<double, 3, 3> Q_GPS;
    Matrix<double, 1, 1> Q_Encoder;
    Matrix<double, 3, 5> C_GPS;
    Matrix<double, 1, 5> C_Encoder;

    void init_R();
    void init_P();
    void init_Q_GPS();
    void init_Q_Encoder();
    void init_C_GPS();
    void init_C_Encoder();
    void init_x();

    void update_motion_model(double dt);
    double clamp_angle(double theta);
    void propagate();
    long get_current_time_millis();
    void kalman_filter(MatrixXd c, MatrixXd q, MatrixXd z);
};