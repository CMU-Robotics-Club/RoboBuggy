#include "ros/ros.h"
#include "robobuggy/IMU.h"
#include <stdio.h>

#include <freespace/freespace.h>
#include <freespace/freespace_util.h>
#include <freespace/freespace_printers.h>


class IMU_Broadcaster
{
public:
    IMU_Broadcaster();
    int init_IMU();
    void publish_IMU_messages();
private:
    ros::NodeHandle nh;
    ros::Publisher imu_pub;

    FreespaceDeviceId device;
    int numIds;

    struct freespace_message message;
}
;
