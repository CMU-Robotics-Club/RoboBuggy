#include "imu/IMU_Broadcaster.h"

IMU_Broadcaster::IMU_Broadcaster()
{
    imu_pub = nh.advertise<robobuggy::IMU>("IMU", 1000);
}

void IMU_Broadcaster::init_IMU()
{
    int err;
    err = freespace_init();
    if (err != FREESPACE_SUCCESS) {
        ROS_ERROR_STREAM("libfreespace initialization error %d\n", err);
        return err;
    }

    err = freespace_getDeviceList(&device, 1, &numIds);
    if (numIds == 0) {
        //@TODO: Keep trying to locate IMU
        ROS_INFO_STREAM("Didn't find any devices\n");
    }

    err = freespace_openDevice(device);
    if (err != FREESPACE_SUCCESS) {
        ROS_ERROR_STREAM("Error opening device\n");
        return err;
    }

    err = freespace_flush(device);
    if (err != FREESPACE_SUCCESS) {
        ROS_ERROR_STREAM("Error flushing device\n");
        return err;
    }

    // Configure device for motion outputs
    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    
}

void IMU_Broadcaster::publish_IMU_messages()
{
    robobuggy::IMU msg;

    msg.X_Accel = get_spoofed_x_accel();
    msg.Y_Accel = get_spoofed_y_accel();
    msg.Z_Accel = get_spoofed_z_accel();

    imu_pub.publish(msg);
}

