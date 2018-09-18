#include "transistor/controller/Controller.h"

void Controller::Pose_Callback(const robobuggy::Pose::ConstPtr& msg)
{
    current_pose_estimate.heading_rad = msg->heading_rad;
    current_pose_estimate.latitude_deg = msg->latitude_deg;
    current_pose_estimate.longitude_deg = msg->longitude_deg;
}


Controller::Controller(std::vector<robobuggy::GPS>& initial_waypoint_list)
{
    waypoint_list = std::vector<robobuggy::GPS>(initial_waypoint_list);
    steering_pub = nh.advertise<robobuggy::Command>("Command", 1000);
    pose_sub = nh.subscribe<robobuggy::Pose>("Pose", 1000, &Controller::Pose_Callback, this);
    last_closest_index = 0;
}

void Controller::update_steering_estimate()
{
    if (current_pose_estimate.latitude_deg == 0 && current_pose_estimate.longitude_deg == 0)
    {
        // haven't initialized yet
        ROS_INFO("No pose update yet!\n");
        return;
    }

    robobuggy::Command steering_msg;
    double steering_rad = pure_pursuit_controller();
    steering_msg.steer_cmd_rad = steering_rad;

    steering_pub.publish(steering_msg);
}

double Controller::get_distance_from_pose(robobuggy::Pose pose_msg)
{
    double dx = (current_pose_estimate.longitude_deg - pose_msg.longitude_deg) * 111319.9;
    double dy = (current_pose_estimate.latitude_deg - pose_msg.latitude_deg) * 84723.58765;

    return std::sqrt(dx * dx + dy * dy);
}

int Controller::get_closest_waypoint_index()
{
    //At this point, the brakes will definitely deploy
    double min = std::numeric_limits<double>::max();
    
    int closestIndex = last_closest_index;
    for(int i = last_closest_index; i < (last_closest_index + WAY_POINT_LOOKAHEAD_MAX) && i < waypoint_list.size(); i++) {
        robobuggy::Pose gpsPoseMessage;
        gpsPoseMessage.latitude_deg = waypoint_list.at(i).Lat_deg;
        gpsPoseMessage.longitude_deg = waypoint_list.at(i).Long_deg;

        double d = get_distance_from_pose(gpsPoseMessage);

        if (d < min) {
            min = d;
            closestIndex = i;
        }
    }
    last_closest_index = closestIndex;
    return closestIndex;
}

double Controller::pure_pursuit_controller()
{
    int closest_index = get_closest_waypoint_index();
    // ROS_INFO("closest waypoint = %d\n", closest_index);
    double k = 2.5;
    double velocity = 3; // TODO bake velocity into pose messages
    double lookahead_lower_bound = 5.0;
    double lookahead_upper_bound = 25.0;
    double lookahead = (k*velocity) / 2.0;

    if(lookahead < lookahead_lower_bound) 
    {
        lookahead = lookahead_lower_bound;
    }
    else if(lookahead > lookahead_upper_bound) 
    {
        lookahead = lookahead_upper_bound;
    }

    int lookahead_index = 0;
    for (lookahead_index = closest_index; lookahead_index < waypoint_list.size(); lookahead_index++)
    {
        robobuggy::Pose gpsPoseMessage;
        gpsPoseMessage.latitude_deg = waypoint_list.at(lookahead_index).Lat_deg;
        gpsPoseMessage.longitude_deg = waypoint_list.at(lookahead_index).Long_deg;

        if (get_distance_from_pose(gpsPoseMessage) > lookahead)
        {
            break;
        }
    }

    if (lookahead_index >= waypoint_list.size()) {
        // run out of waypoints nearby, go straight
        ROS_INFO("uh oh, out of waypoints!\n");

        return 0.0;
    }

    robobuggy::GPS target_waypoint = waypoint_list.at(lookahead_index);
    // ROS_INFO("target waypoint = (%f, %f)\n", target_waypoint.Lat_deg, target_waypoint.Long_deg);
    
    geographic_msgs::GeoPoint gps_point;
    gps_point.latitude = target_waypoint.Lat_deg;
    gps_point.longitude = target_waypoint.Long_deg;

    geodesy::UTMPoint utm_waypoint(gps_point);

    gps_point.latitude = current_pose_estimate.latitude_deg;
    gps_point.longitude = current_pose_estimate.longitude_deg;
    geodesy::UTMPoint utm_pose(gps_point);

    double dx = utm_waypoint.easting - utm_pose.easting;
    double dy = utm_waypoint.northing - utm_pose.northing;
    double theta = atan2(dy, dx) - current_pose_estimate.heading_rad;


    double commanded_angle = normalize_angle_rad(atan2(2 * 1.13 * sin(theta), lookahead));
    return commanded_angle;
}

bool Controller::get_deploy_brake_value()
{
    return false;
}

double Controller::normalize_angle_rad(double radians) {
    while (radians < 0.0) 
    {
        radians = radians + 2 * M_PI;
    }

    if (radians > M_PI) 
    {
        radians = radians - 2.0 * M_PI;
    }
    return radians;

}