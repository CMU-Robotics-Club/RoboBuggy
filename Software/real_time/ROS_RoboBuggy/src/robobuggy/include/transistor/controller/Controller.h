//
// Created by bhai on 9/15/17.
//

#ifndef ROS_ROBOBUGGY_CONTROLLER_H
#define ROS_ROBOBUGGY_CONTROLLER_H

#include "ros/ros.h"
#include <ros/package.h>
#include <robobuggy/Pose.h>
#include <robobuggy/Command.h>
#include <robobuggy/GPS.h>
#include <string>
#include <geodesy/utm.h>
#include <cmath>

class Controller
{
public:
    Controller(std::vector<robobuggy::GPS>& initial_waypoint_list);
    void Pose_Callback(const robobuggy::Pose::ConstPtr& msg);
    void update_steering_estimate();
    ros::NodeHandle nh;
private:
    ros::Subscriber pose_sub;
    ros::Publisher steering_pub;
    ros::Publisher waypoint_pub;

    robobuggy::Pose current_pose_estimate;
    robobuggy::GPS target_waypoint;
    std::vector<robobuggy::GPS> waypoint_list;
    int last_closest_index;
    const static int WAY_POINT_LOOKAHEAD_MAX = 10;

    void fill_waypoint_utms(std::vector<robobuggy::GPS>& waypoint_list);
    int get_closest_waypoint_index();
    double stanley_controller();
    bool get_deploy_brake_value();
    double normalize_angle_rad(double radians);
    double get_distance_from_pose(robobuggy::GPS pose_msg);
    double get_angle_from_pose(robobuggy::GPS waypoint);
    double get_path_angle(robobuggy::GPS w1, robobuggy::GPS w2);
};

#endif //ROS_ROBOBUGGY_CONTROLLER_H
