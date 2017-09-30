#include "imu/IMU_Broadcaster.h"

IMU_Broadcaster::IMU_Broadcaster()
{
    imu_pub = nh.advertise<robobuggy::IMU>("IMU", 1000);
}

int IMU_Broadcaster::init_IMU()
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
    message.dataModeControlV2Request.packetSelect = 8;  // MotionEngine Outout
    message.dataModeControlV2Request.mode = 0;          // Set full motion
    message.dataModeControlV2Request.formatSelect = 0;  // MEOut format 0
    message.dataModeControlV2Request.ff0 = 1;           // Pointer fields
	message.dataModeControlV2Request.ff1 = 1;           // Linear Acceleration
	message.dataModeControlV2Request.ff2 = 1;           // Linear Acce no gravity
    message.dataModeControlV2Request.ff3 = 1;           // Angular velocity fields
	message.dataModeControlV2Request.ff4 = 1;           // Magnetometer
	message.dataModeControlV2Request.ff6 = 1;           // Angular position

    err = freespace_sendMessage(device, &message);
    if( err != FREESPACE_SUCCESS) {
        ROS_ERROR_STREAM("Could not send message\n");
        return err;
    }
}

void IMU_Broadcaster::publish_IMU_message()
{
    robobuggy::IMU msg;
    int err;
    
    err = freespace_readMessage(device, &message, 100);

    if ((err == FREESPACE_ERROR_TIMEOUT) || (err == FREESPACE_ERR_INTERRUPTED)) {
        return;
    }
    if (err != FREESPACE_SUCCESS) {
        //@TODO: This doesn't actually terminate the loop
        ROS_ERROR_STREAM("Error reading message\n");
        return;
    }

    if (message.messageType == FREESPACE_MESSAGE_MOTIONENGINEOUTPUT) {
        struct MultiAxisSensor reading;

        // Read acceleration data
        err = freespace_util_getAcceleration(&message.motionEngineOutput, &reading);
        if (err == 0) {
            // Load our IMU message with the data
            msg.X_Accel = reading.x;
            msg.Y_Accel = reading.y;
            msg.Z_Accel = reading.z;
        }

        // Read acceleration no gravity data
        err = freespace_util_getAccNoGravity(&message.motionEngineOutput, &reading);
        if (err == 0) {
            // Load our IMU message with the data
            msg.X_Accel_noG = reading.x;
            msg.Y_Accel_noG = reading.y;
            msg.Z_Accel_noG = reading.z;
        }

        // Read angular velocity data
        err = freespace_util_getAngularVelocity(&message.motionEngineOutput, &reading);
        if (err == 0) {
            // Load our IMU message with the data
            msg.X_AngVel = reading.x;
            msg.Y_AngVel = reading.y;
            msg.Z_AngVel = reading.z;
        }

        // Read magnetometer data
        err = freespace_util_getMagnetometer(&message.motionEngineOutput, &reading);
        if (err == 0) {
            // Load our IMU message with the data
            msg.X_Mag = reading.x;
            msg.Y_Mag = reading.y;
            msg.Z_Mag = reading.z;
        }

        // Read angular position data
        err = freespace_util_getAngPos(&message.motionEngineOutput, &reading);
        if (err == 0) {
            // Load our IMU message with the data
            msg.X_AngPos = reading.x;
            msg.Y_AngPos = reading.y;
            msg.Z_AngPos = reading.z;
        }
    }

    ros::Time tiestamp = ros::Time::now();
    msg.header.stamp = timestamp;

    imu_pub.publish(msg);
}

