#include "IMU_Broadcaster.h"

float IMU_Broadcaster::get_spoofed_x_accel()
{
    return 1.5;
}
float IMU_Broadcaster::get_spoofed_y_accel()
{
    return 2.5;
}
float IMU_Broadcaster::get_spoofed_z_accel()
{
    return 1.5;
}

IMU_Broadcaster::IMU_Broadcaster()
{
    imu_pub = nh.advertise<robobuggy::IMU>("IMU", 1000);
}

void IMU_Broadcaster::publish_new_spoofed_message()
{
    robobuggy::IMU msg;

    msg.X_Accel = get_spoofed_x_accel();
    msg.Y_Accel = get_spoofed_y_accel();
    msg.Z_Accel = get_spoofed_z_accel();

    imu_pub.publish(msg);
}

