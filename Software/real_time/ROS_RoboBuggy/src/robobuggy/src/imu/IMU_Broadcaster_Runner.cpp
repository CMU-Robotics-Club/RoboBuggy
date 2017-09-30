#include "imu/IMU_Broadcaster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IMU_Broadcaster_Runner");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    IMU_Broadcaster broadcaster;

    broadcaster.init_IMU();

    // Infinitely spin sending dummy IMU messages
    while (ros::ok())
    {
        broadcaster.publish_IMU_message();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
