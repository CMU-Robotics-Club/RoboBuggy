#include "transistor/imu/IMU_Broadcaster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, IMU_Broadcaster::NODE_NAME);
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    IMU_Broadcaster broadcaster;

    int err = broadcaster.init_IMU();

    if (err != 0) {
        ROS_ERROR_STREAM("Failed to initialize IMU");
        ROS_ERROR_STREAM(err);
        return err;
    }

    // Infinitely spin sending dummy IMU messages
    while (ros::ok())
    {
        broadcaster.publish_IMU_message();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
