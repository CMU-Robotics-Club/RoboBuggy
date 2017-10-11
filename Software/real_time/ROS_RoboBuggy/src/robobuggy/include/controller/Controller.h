//
// Created by bhai on 9/15/17.
//

#ifndef ROS_ROBOBUGGY_CONTROLLER_H
#define ROS_ROBOBUGGY_CONTROLLER_H

#include <robobuggy/ENC.h>
#include "ros/ros.h"
#include "robobuggy/IMU.h"
#include "robobuggy/GPS.h"
#include "std_msgs/String.h"
#include <string>
class Controller
{
public:
    static void Pose_Callback(const robobuggy::Pose::ConstPtr& msg);
    Controller();
    ros::NodeHandle nh;
    void update_steering_estimate();
private:
    ros::Subscriber pose_sub;
    ros::Publisher steering_pub;

    robobuggy::Pose current_pose_estimate;
    robobuggy::GPS target_waypoint;
    std::vector<robobuggy::GPS> waypoint_list;

    int get_closest_waypoint_index();
    double pure_pursuit_controller();
    bool get_deploy_brake_value();
};

#endif //ROS_ROBOBUGGY_CONTROLLER_H
