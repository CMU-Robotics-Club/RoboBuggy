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
    update_motion_model();

    imu_sub = nh.subscribe<robobuggy::IMU>("IMU", 1000, IMU_Callback);
    gps_sub = nh.subscribe<robobuggy::GPS>("GPS", 1000, GPS_Callback);
    enc_sub = nh.subscribe<robobuggy::ENC>("ENC", 1000, ENC_Callback);
    pose_pub = nh.advertise<robobuggy::Pose>("Pose", 1000);

}

void Localizer::update_position_estimate()
{

}

void Localizer::update_motion_model()
{

}

// TODO create runner file once we've filled the rest of this out
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Localizer");
    ros::NodeHandle nh;
    Localizer l;

    ros::spin();

    return 0;

}