#include "ros/ros.h"
#include "robobuggy/IMU.h"
#include <stdio.h>

class IMU_Broadcaster
{
public:
    IMU_Broadcaster();
    ros::NodeHandle nh;
    ros::Publisher imu_pub;
    float get_spoofed_x_accel();
    float get_spoofed_y_accel();
    float get_spoofed_z_accel();
    void publish_new_spoofed_message();
}
;