#include "localizer/Localizer.h"

void Localizer::ENC_Callback(const robobuggy::ENC::ConstPtr &msg)
{

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
    dx = dx * 0.61 / 7.0;
    double body_speed = dx / (dt / 1000);

    prev_encoder_time = current_time;
    prev_encoder_ticks = ticks;

    Matrix<double, 1, 1> z;
    z <<
      body_speed
    ;

    kalman_filter(C_Encoder, Q_Encoder, z);
}

void Localizer::GPS_Callback(const robobuggy::GPS::ConstPtr &msg)
{
    geodesy::UTMPoint p;
    p.northing = msg->Lat_m;
    p.easting = msg->Long_m;
    p.band = 'T';
    p.zone = 17;
    double heading = 0.0;

    if (prev_position_utm.northing != 0)
    {
        double dx = p.easting - prev_position_utm.easting;
        double dy = p.northing - prev_position_utm.northing;

        heading = atan2(dy, dx);
        if (dx * dx + dy * dy < 0.25)
        {
            heading = x(2, 0);
        }
        prev_position_utm = p;
    }

    Matrix<double, 3, 1> z;
    z <<
      p.easting,
      p.northing,
      heading
    ;

    kalman_filter(C_GPS, Q_GPS, z);

}

void Localizer::IMU_Callback(const robobuggy::IMU::ConstPtr &msg)
{

}

void Localizer::Steering_Callback(const robobuggy::Steering::ConstPtr &msg)
{
    current_steering_angle = msg->steer_feedback;
    current_steering_angle = current_steering_angle / 100;
    current_steering_angle = M_PI / 180 * current_steering_angle;
}

void Localizer::init_R()
{
    R <<
      4, 0, 0, 0, 0,
      0, 4, 0, 0, 0,
      0, 0, 0.25, 0, 0,
      0, 0, 0, 0.01, 0,
      0, 0, 0, 0, 0.01
    ;

    std::stringstream s;
    s << R << std::endl;

    ROS_INFO("Initialized R Matrix to : \n%s", s.str().c_str());
}

void Localizer::init_P()
{

    P <<
      25, 0, 0, 0, 0,
      0, 25, 0, 0, 0,
      0, 0, 0.25, 0, 0,
      0, 0, 0, 2.46, 0,
      0, 0, 0, 0, 2.46
    ;

    std::stringstream s;
    s << P << std::endl;

    ROS_INFO("Initialized P Matrix to : \n%s", s.str().c_str());
}

void Localizer::init_Q_GPS()
{
    Q_GPS <<
          1, 0, 0,
          0, 1, 0,
          0, 0, 0.01
    ;

    std::stringstream s;
    s << Q_GPS << std::endl;

    ROS_INFO("Initialized Q_GPS Matrix to : \n%s", s.str().c_str());
}

void Localizer::init_Q_Encoder()
{
    Q_Encoder <<
              0.25
    ;

    std::stringstream s;
    s << Q_Encoder << std::endl;

    ROS_INFO("Initialized Q_Encoder matrix to : \n%s", s.str().c_str());
}

void Localizer::init_C_GPS()
{
    C_GPS <<
          1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 0, 1, 0
    ;

    std::stringstream s;
    s << C_GPS << std::endl;

    ROS_INFO("Initialized C_GPS Matrix to : \n%s", s.str().c_str());
}

void Localizer::init_C_Encoder()
{
    C_Encoder <<
              0, 0, 1, 0, 0
    ;

    std::stringstream s;
    s << C_Encoder << std::endl;

    ROS_INFO("Initialized C_Encoder Matrix to : \n%s", s.str().c_str());
}

void Localizer::init_x()
{
    x <<
      0, // TODO initial lat in UTM
      0, // TODO initial lon in UTM
      0,
      0, // TODO initial heading in rad
      0
    ;

    x_hat <<
          0,
          0,
          0,
          0,
          0
    ;

    std::stringstream s;
    s << x << std::endl;

    ROS_INFO("Initialized x Matrix to : \n%s", s.str().c_str());
}

Localizer::Localizer()
{
    init_R();
    init_P();
    init_Q_GPS();
    init_Q_Encoder();
    init_C_GPS();
    init_C_Encoder();
    init_x();
    update_motion_model(0);

    struct timeval tp;
    gettimeofday(&tp, NULL);
    previous_update_time_ms  = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    prev_encoder_time = previous_update_time_ms;
    prev_encoder_ticks = -1;

    // TODO Work IMU into KF
//    imu_sub = nh.subscribe<robobuggy::IMU>("IMU", 1000, IMU_Callback);
    gps_sub = nh.subscribe<robobuggy::GPS>("GPS", 1000, &Localizer::GPS_Callback, this);
    enc_sub = nh.subscribe<robobuggy::ENC>("Encoder", 1000, &Localizer::ENC_Callback, this);
    steering_sub = nh.subscribe<robobuggy::Steering>("Steering", 1000, &Localizer::Steering_Callback, this);
    pose_pub = nh.advertise<robobuggy::Pose>("Pose", 1000);

}

void Localizer::update_position_estimate()
{
    propagate();

    geodesy::UTMPoint utm_point(x_hat(0, 0), x_hat(1, 0), 17, 'T');
    geographic_msgs::GeoPoint gps_point = geodesy::toMsg(utm_point);
    double heading = x_hat(3, 0);

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
      1, 0, dt * cos(x(3, 0)), 0, 0,
      0, 1, dt * sin(x(3, 0)), 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 1, dt,
      0, 0, tan(current_steering_angle) / WHEELBASE_M, 0, 0
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

void Localizer::propagate()
{
    long int current_time = get_current_time_millis();
    // convert to seconds
    double dt = (current_time - previous_update_time_ms) / 1000.0;

    update_motion_model(dt);
    x_hat = A * x;
}

long Localizer::get_current_time_millis()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

void Localizer::kalman_filter(MatrixXd c, MatrixXd q, MatrixXd z)
{
    long int current_time = get_current_time_millis();
    double dt = (current_time - previous_update_time_ms) / 1000.0;
    previous_update_time_ms = current_time;

    update_motion_model(dt);

    Matrix<double, 5, 1> x_pre = A * x;
    Matrix<double, 5, 5> P_pre = A * P * A.transpose() + R;

    x_pre(2, 0) = clamp_angle(x_pre(2, 0));
    x_pre(4, 0) = clamp_angle(x_pre(4, 0));

    MatrixXd residual = z - c * x_pre;
    MatrixXd K = c * P_pre * c.transpose() + q;
    K = P_pre * c.transpose() * K.inverse();
    x = x_pre + K * residual;
    P = (MatrixXd::Identity(5,5) - (K * c)) * P_pre;

    x(2, 0) = clamp_angle(x(2, 0));
    x(4, 0) = clamp_angle(x(4, 0));
}
