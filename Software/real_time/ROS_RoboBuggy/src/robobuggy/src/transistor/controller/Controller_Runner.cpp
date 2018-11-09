//
// Created by bhai on 9/15/17.
//

#include "transistor/controller/Controller.h"
#include <json/json.h>
#include <fstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    std::vector<robobuggy::GPS> waypoints;

    std::string package_path = ros::package::getPath("robobuggy");
    std::string config_path = "/config/waypoints.txt";

    std::ifstream waypoint_stream(package_path + config_path);
    std::string waypoint_str;
    while (std::getline(waypoint_stream, waypoint_str))
    {
        Json::Value waypoint_json;

        // convert the line into a string stream so that we can put it easily in the jsoncpp
        std::stringstream waypoint_str_stream(waypoint_str);
        waypoint_str_stream >> waypoint_json;

        // get a new GPS msg
        robobuggy::GPS waypoint_msg;
        waypoint_msg.Lat_deg = waypoint_json["latitude"].asFloat();
        waypoint_msg.Long_deg = waypoint_json["longitude"].asFloat();
        waypoints.push_back(waypoint_msg);

    }

    ROS_INFO("Read in %lu waypoints\n", waypoints.size());

    if (waypoints.size() == 0) {
        return 1;
    }

    Controller controller(waypoints);

    while (ros::ok())
    {
        controller.update_steering_estimate();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
