#include "transistor/localizer/Localizer.h"

const std::string Localizer::NODE_NAME = "Localizer";
void Localizer::Encoder_Callback(const robobuggy::Encoder::ConstPtr &msg)
{
    // todo aggregate and then reset once kf is finished

    if (prev_encoder_ticks == -1)
    {
        prev_encoder_ticks = msg->ticks;
    }

    long int current_time = get_current_time_millis();
    long int dt = current_time - prev_encoder_time;

    if (dt < 50)
    {
        return;
    }

    double ticks = msg->ticks;
    double dx = ticks - prev_encoder_ticks;
    // TODO make this clearer
    dx = dx * 0.61 / 7.0 * 0.3048 * 2;
    double body_speed = dx / (dt / 1000.0);

    prev_encoder_time = current_time;
    prev_encoder_ticks = ticks;

    z_enc_ticks_dx += dx;
    z_enc_vel = body_speed;
}

void Localizer::GPS_Callback(const robobuggy::GPS::ConstPtr &msg)
{
    geodesy::UTMPoint p;
    p.northing = msg->northing;
    p.easting = msg->easting;
    p.band = 'T';
    p.zone = 17;
    
    double heading_cartesian = 0.0;

    if (prev_position_utm.northing != 0)
    {
        double dx = p.easting - prev_position_utm.easting;
        double dy = p.northing - prev_position_utm.northing;

        heading_cartesian = atan2(dx, dy);
        if (sqrt(dx * dx + dy * dy) < 0.25)
        {
            heading_cartesian = x(3, 0);
        }
    }
    prev_position_utm = p;
    z_gps_easting = p.easting;
    z_gps_northing = p.northing;

    // TODO until we get the IMU in, this will have to do:
    // z_imu_heading = heading_cartesian;
}


void Localizer::IMU_Callback(const robobuggy::IMU::ConstPtr &msg)
{
    float q0 = msg->A_AngPos;
    float q1 = msg->B_AngPos;
    float q2 = msg->C_AngPos;
    float q3 = msg->D_AngPos;

    float yaw = atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));
    yaw += M_PI_2; // 0 for imu is straight north

    // however, the IMU points to magnetic north
    // we need it to point to true north
    float rad_offset = M_PI * 9.366667 / 180.0;
    yaw += rad_offset;

    z_imu_heading = yaw;
}

void Localizer::Feedback_Callback(const robobuggy::Feedback::ConstPtr &msg)
{
    current_steering_angle = msg->steer_angle;
    current_steering_angle = current_steering_angle / 100;
    current_steering_angle = M_PI / 180 * current_steering_angle;
}

void Localizer::init_R()
{
    R <<
    0.25, 0, 0, 0, 0,
    0, 0.25, 0, 0, 0,
    0, 0, 0.01,0, 0,
    0, 0, 0, 0.001, 0,
    0, 0, 0, 0, 0.001
    ;

    std::stringstream s;
    s << R << std::endl;

    ROS_INFO("Initialized R Matrix to : \n%s", s.str().c_str());
}

void Localizer::init_SIGMA()
{

    SIGMA <<
    1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 0.01, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 0.04
    ;

    std::stringstream s;
    s << SIGMA << std::endl;

    ROS_INFO("Initialized SIGMA Matrix to : \n%s", s.str().c_str());
}

void Localizer::init_C()
{
    // update at a constant rate, this allows C to stay constant
    double t = 1.0 / Localizer::UPDATE_KALMAN_RATE_HZ;

    C <<
    1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, t, 0, 0,
    0, 0, 1, 0, 0
    ;

    std::stringstream s;
    s << C << std::endl;

    ROS_INFO("Initialized C Matrix to : \n%s", s.str().c_str());
}

void Localizer::init_Q() 
{
    Q <<
    9, 0, 0, 0, 0,
    0, 9, 0, 0, 0,
    0, 0, 0.000025, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1
    ;

    std::stringstream s;
    s << Q << std::endl;

    ROS_INFO("Initialized Q matrix to : \n%s", s.str().c_str());
}

void Localizer::init_x()
{
    double init_latitude = 40.442616;
    double init_longitude = -79.943336;

    geographic_msgs::GeoPoint gps_point;
    gps_point.latitude = init_latitude;
    gps_point.longitude = init_longitude;
    geodesy::UTMPoint init_utm(gps_point);

    x <<
      init_utm.easting, 
      init_utm.northing, 
      1,
      0, // TODO initial heading in rad
      0
    ;

    std::stringstream s;
    s << x << std::endl;

    ROS_INFO("Initialized x Matrix to : \n%s", s.str().c_str());
}

Localizer::Localizer()
{
    ROS_INFO("Initializing Localizer\n");

    current_steering_angle = 0;
    WHEELBASE_M = 1.13;

    init_R();
    init_SIGMA();
    init_Q();
    init_C();
    init_x();
    update_motion_model(0);

    // initialize our measurements so that we're stationary
    z_enc_ticks_dx = 0;
    z_enc_vel = 0;
    z_gps_easting = x(ROW_X);
    z_gps_northing = x(ROW_Y);
    z_imu_heading = 0;

    struct timeval tp;
    gettimeofday(&tp, NULL);
    previous_update_time_ms  = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    prev_encoder_time = previous_update_time_ms;
    prev_encoder_ticks = -1;

    // TODO Work IMU into KF
    imu_sub = nh.subscribe<robobuggy::IMU>("IMU", 1000, &Localizer::IMU_Callback, this);
    gps_sub = nh.subscribe<robobuggy::GPS>("GPS", 1000, &Localizer::GPS_Callback, this);
    enc_sub = nh.subscribe<robobuggy::Encoder>("Encoder", 1000, &Localizer::Encoder_Callback, this);
    steering_sub = nh.subscribe<robobuggy::Feedback>("Feedback", 1000, &Localizer::Feedback_Callback, this);
    pose_pub = nh.advertise<robobuggy::Pose>("Pose", 1000);

}

void Localizer::update_position_estimate()
{
    // update the filter
    kalman_filter();

    // convert the UTM understanding to Pose compliant form
    double easting_x = x(ROW_X, 0);
    double northing_y = x(ROW_Y, 0);
    geodesy::UTMPoint utm_point(easting_x, northing_y, 17, 'T');
    geographic_msgs::GeoPoint gps_point = geodesy::toMsg(utm_point);
    double heading = x(ROW_HEADING, 0);

    // create the pose message
    robobuggy::Pose p;
    ros::Time time_now = ros::Time::now();
    p.header.stamp = time_now;
    p.heading_rad = heading;
    p.latitude_deg = gps_point.latitude;
    p.longitude_deg = gps_point.longitude;

    pose_pub.publish(p);

}

void Localizer::update_motion_model(double dt)
{

    A <<
      1, 0, dt * cos(x(ROW_HEADING, 0)), 0, 0,
      0, 1, dt * sin(x(ROW_HEADING, 0)), 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 1, dt,
      0, 0, 0, 0, 1
    ;

}

void Localizer::update_control_model()
{
    B <<
        0,
        0,
        0,
        0,
        1/WHEELBASE_M
    ;
}

double Localizer::clamp_angle(double theta)
{
    while (theta < -M_PI)
    {
        theta += 2*M_PI;
    }
    while (theta > M_PI)
    {
        theta -= 2*M_PI;
    }

    return theta;
}

long Localizer::get_current_time_millis()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

void Localizer::kalman_filter()
{
    // run kalman prediction
    // propagate prediction forwards using motion model
    update_motion_model(1.0 / Localizer::UPDATE_KALMAN_RATE_HZ);
    update_control_model();
    x = A * x + B * tan(current_steering_angle);
    // update covariances using predicted covariance
    SIGMA = A * SIGMA * A.transpose() + R;

    // Get all the measurements together into a single vector
    z <<
        z_gps_easting,
        z_gps_northing,
        z_imu_heading,
        z_enc_ticks_dx,
        z_enc_vel
    ;

    // run kalman correction
    // Calculate Kalman Gain
    Matrix<double, 5, 5> K = SIGMA * C.transpose() * (C*SIGMA*C.transpose() + Q).inverse();
    // compute corrected state
    x = x + K*(z - C*x);
    // compute corrected covariance
    SIGMA = (MatrixXd::Identity(x_len, x_len) - K*C)*SIGMA;

    // normalize x (want to clamp our angles within [-pi, pi])
    double th = x(ROW_HEADING);
    double th_d = x(ROW_TH_DOT);

    double th_norm = clamp_angle(th);
    double th_d_norm = clamp_angle(th_d);

    x(ROW_HEADING) = th_norm;
    x(ROW_TH_DOT) = th_d_norm;

    // reset amalgamated values
    z_enc_ticks_dx = 0;
}
