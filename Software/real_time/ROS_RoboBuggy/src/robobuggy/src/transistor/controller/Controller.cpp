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
    fill_waypoint_utms(waypoint_list);
    steering_pub = nh.advertise<robobuggy::Command>("Command", 1000);
    pose_sub = nh.subscribe<robobuggy::Pose>("Pose", 1000, &Controller::Pose_Callback, this);
    last_closest_index = 12;
}

void Controller::fill_waypoint_utms(std::vector<robobuggy::GPS>& waypoint_list)
{
    for (robobuggy::GPS& waypoint : waypoint_list)
    {
        geographic_msgs::GeoPoint gps_point;
        gps_point.latitude = waypoint.Lat_deg;
        gps_point.longitude = waypoint.Long_deg;
        geodesy::UTMPoint utm_point(gps_point);
        waypoint.easting = utm_point.easting;
        waypoint.northing = utm_point.northing;
    }
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
    double steering_rad = stanley_controller();
    steering_msg.steer_cmd_rad = steering_rad;

    steering_pub.publish(steering_msg);
}

double Controller::get_distance_from_pose(robobuggy::GPS pose_msg)
{
    geographic_msgs::GeoPoint curr_point;
    curr_point.latitude = current_pose_estimate.latitude_deg;
    curr_point.longitude = current_pose_estimate.longitude_deg;
    geodesy::UTMPoint utm_point(curr_point);

    double dx = utm_point.easting - pose_msg.easting;
    double dy = utm_point.northing - pose_msg.northing;

    return std::sqrt(dx * dx + dy * dy);
}

double Controller::get_angle_from_pose(robobuggy::GPS waypoint)
{
    geographic_msgs::GeoPoint curr_point;
    curr_point.latitude = current_pose_estimate.latitude_deg;
    curr_point.longitude = current_pose_estimate.longitude_deg;
    geodesy::UTMPoint utm_point(curr_point);

    double dx = utm_point.easting - waypoint.easting;
    double dy = utm_point.northing - waypoint.northing;

    return atan2(dy, dx);
}

double Controller::get_path_angle(robobuggy::GPS w1, robobuggy::GPS w2)
{
    double dx = w1.easting - w2.easting;
    double dy = w1.northing - w2.northing;

    return atan2(dy, dx);
}

int Controller::get_closest_waypoint_index()
{
    //At this point, the brakes will definitely deploy
    double min = std::numeric_limits<double>::max();
    
    int closestIndex = last_closest_index;
    for(int i = last_closest_index; i < (last_closest_index + WAY_POINT_LOOKAHEAD_MAX) && i < waypoint_list.size(); i++) {
        double d = get_distance_from_pose(waypoint_list.at(i));

        if (d < min) {
            min = d;
            closestIndex = i;
        }
    }
    last_closest_index = closestIndex;
    return closestIndex;
}

double Controller::stanley_controller()
{
    // figure out theta_p, or the angle of the path
    // get the closest waypoint to us, and get the angle to it and the one behind it
    int closest_index = get_closest_waypoint_index();
    int second_closest = closest_index - 1;
    robobuggy::GPS closest_waypoint = waypoint_list.at(closest_index);
    robobuggy::GPS prev_waypoint = waypoint_list.at(second_closest);
    double theta_p = get_path_angle(closest_waypoint, prev_waypoint);
    // subtract it from theta to get our theta error
    double theta_e = normalize_angle_rad(theta_p - current_pose_estimate.heading_rad);

    // calculate cross-track error
    // we have the angle between the buggy and the path
    double buggy_wp_angle = get_angle_from_pose(prev_waypoint);
    // and we have the line segment between the closest waypoint and the buggy
    double pose_wp_dist = get_distance_from_pose(prev_waypoint);
    // so we can use the pythagorean theorem to figure out the distance between buggy and path
    double d_buggy_path_angle = buggy_wp_angle - theta_p;
    double cross_track_err = -1 * pose_wp_dist * sin(d_buggy_path_angle);
    double k = 5; // define gain
    // TEMP assume velocity = 3m/s
    double current_velocity = 3.0;

    // combine it together into the stanley control formula
    // delta = theta_e + atan(k * xtrack_e / velocity)
    double delta = theta_e + atan2(k * cross_track_err, current_velocity);

    ROS_INFO("current heading = %f", current_pose_estimate.heading_rad);
    ROS_INFO("theta_p = %f", theta_p);
    ROS_INFO("theta_e = %f", theta_e);
    ROS_INFO("xtrack err = %f", cross_track_err);
    ROS_INFO("delta = %f\n", delta);

    return normalize_angle_rad(delta);
}

bool Controller::get_deploy_brake_value()
{
    return false;
}

double Controller::normalize_angle_rad(double radians) {
    while (radians < -M_PI) 
    {
        radians = radians + 2 * M_PI;
    }

    while (radians > M_PI) 
    {
        radians = radians - 2 * M_PI;
    }
    return radians;

}