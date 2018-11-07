#include <ros/ros.h>
#include <freespace/freespace.h>
#include <freespace/freespace_util.h>
#include <freespace/freespace_printers.h>

struct freespace_message message;
FreespaceDeviceId device;
int numIds;

int init_imu()
{
    int err;
    err = freespace_init();
    if (err != FREESPACE_SUCCESS) {
        ROS_ERROR_STREAM("libfreespace initialization error\n");
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

    memset(&message, 0, sizeof(message));
    // Configure device for motion outputs
    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    message.dataModeControlV2Request.packetSelect = 8;  // MotionEngine Outout
    message.dataModeControlV2Request.mode = 4;          // Set full motion (without going to sleep)
    message.dataModeControlV2Request.formatSelect = 0;  // MEOut format 0
    message.dataModeControlV2Request.ff0 = 1;           // Pointer fields
	message.dataModeControlV2Request.ff1 = 1;           // Linear Acceleration
	message.dataModeControlV2Request.ff2 = 1;           // Linear Acce no gravity
    message.dataModeControlV2Request.ff3 = 1;           // Angular velocity fields
    message.dataModeControlV2Request.ff4 = 1;           // Magnetometer
    message.dataModeControlV2Request.ff5 = 0;
    message.dataModeControlV2Request.ff6 = 1;           // Angular position

    err = freespace_sendMessage(device, &message);
    if( err != FREESPACE_SUCCESS) {
        ROS_ERROR_STREAM("Could not send message\n");
        return err;
    }

    err = freespace_readMessage(device, &message, 100);
    if (err != FREESPACE_SUCCESS) {
        ROS_ERROR("Failed to read back from IMU!\n");
        return err;
    }

    ROS_INFO("Successfully Initialized IMU\n");

    return 0;
}

// prints out the values stored in the flash memory
// for the angular position smoothing parameters
int get_flash_memory_angpossmooth()
{
    memset(&message, 0, sizeof(message));

    message.messageType = FREESPACE_MESSAGE_FRSEFLASHREADREQUEST;
    message.fRSReadRequest.readOffset = 0;
    message.fRSReadRequest.FRStype = 0x3E2D; 
    message.fRSReadRequest.BlockSize = 0;

    int err = freespace_sendMessage(device, &message);
    if (err != FREESPACE_SUCCESS)
    {
        ROS_ERROR("Failed to send message to the IMU");
    }

    err = freespace_readMessage(device, &message, 1000);
    if ((err == FREESPACE_ERROR_TIMEOUT) || (err == FREESPACE_ERROR_INTERRUPTED))
    {
        return err;
    }

    if (message.messageType == FREESPACE_MESSAGE_FRSREADRESPONSE)
    {
        int status = message.fRSReadResponse.status;
        if (status != 3)
        {
            return status;
        }

        uint32_t *data = message.fRSReadResponse.data;

        // scaling is Q30
        float scaling = data[0] * pow(2, -30);
        // max rotation is Q29
        float max_rotation = data[1] * pow(2, -29);
        // max error is Q29
        float max_error = data[2] * pow(2, -29);

        ROS_INFO("Scaling: %f", scaling);
        ROS_INFO("Max Rot: %f", max_rotation);
        ROS_INFO("Max Err: %f", max_error);

    }

    return 0;
}

void imu_command_callback()
{
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "IMU_util_ctl");
    ros::NodeHandle nh;
    ros::Rate loop_rate(5);

    int err = init_imu();
    if (err)
    {
        return err;
    }


    while (ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}