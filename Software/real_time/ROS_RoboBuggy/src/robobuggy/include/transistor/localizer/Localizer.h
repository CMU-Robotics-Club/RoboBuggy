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
    static const int UPDATE_KALMAN_RATE_HZ = 5;

    void update_position_estimate();

private:
    geodesy::UTMPoint prev_position_utm;
    double prev_encoder_ticks;
    double current_steering_angle;
    double WHEELBASE_M;
    long int previous_update_time_ms;
    long int prev_encoder_time;

    double z_gps_latitude;
    double z_gps_longitude;
    double z_enc_ticks;
    double z_imu_heading;

    const int ROW_X = 0;
    const int ROW_Y = 1;
    const int ROW_VEL = 2;
    const int ROW_HEADING = 3;
    const int ROW_TH_DOT = 4;
    const int x_len = 5;

    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber enc_sub;
    ros::Subscriber steering_sub;

    Matrix<double, 5, 5> A;
    Matrix<double, 5, 1> B;
    Matrix<double, 5, 1> x;
    Matrix<double, 5, 5> R;
    MAtrix<double, 4, 1> z;
    Matrix<double, 5, 5> SIGMA;
    Matrix<double, 4, 4> Q;
    Matrix<double, 4, 5> C;

    void init_R();
    void init_SIGMA();
    void init_Q();
    void init_C();
    void init_x();

    void IMU_Callback(const robobuggy::IMU::ConstPtr& msg);
    void GPS_Callback(const robobuggy::GPS::ConstPtr& msg);
    void Encoder_Callback(const robobuggy::Encoder::ConstPtr& msg);
    void Feedback_Callback(const robobuggy::Feedback::ConstPtr& msg);
 
    void update_motion_model(double dt);
    void update_control_model();
    double clamp_angle(double theta);
    long get_current_time_millis();
    void kalman_filter();
};
