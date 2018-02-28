#include "ros/ros.h"

#include <geodesy/utm.h>
#include <string>
#include <iostream>
#include <cmath>
#include <sys/time.h>

#include <eigen3/Eigen/Dense>

#include <robobuggy/IMU.h>
#include <robobuggy/GPS.h>
#include <robobuggy/Encoder.h>
#include <robobuggy/Pose.h>
#include <robobuggy/Feedback.h>

using Eigen::Matrix;
using Eigen::MatrixXd;

class Localizer 
{
public:
    Localizer();
    static const std::string NODE_NAME;

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
    Matrix<double, 2, 3> Q_GPS;
    Matrix<double, 1, 1> Q_Encoder;
    Matrix<double, 1, 1> Q_IMU;
    Matrix<double, 2, 5> C_GPS;
    Matrix<double, 1, 5> C_Encoder;
    Matrix<double, 1, 5> C_IMU;

    void init_R();
    void init_P();
    void init_Q_GPS();
    void init_Q_Encoder();
    void init_Q_IMU();
    void init_C_GPS();
    void init_C_Encoder();
    void init_C_IMU();
    void init_x();

    void IMU_Callback(const robobuggy::IMU::ConstPtr& msg);
    void GPS_Callback(const robobuggy::GPS::ConstPtr& msg);
    void Encoder_Callback(const robobuggy::Encoder::ConstPtr& msg);
    void Feedback_Callback(const robobuggy::Feedback::ConstPtr& msg);
 
    void update_motion_model(double dt);
    double clamp_angle(double theta);
    void propagate();
    long get_current_time_millis();
    void kalman_filter(MatrixXd c, MatrixXd q, MatrixXd z);
};
