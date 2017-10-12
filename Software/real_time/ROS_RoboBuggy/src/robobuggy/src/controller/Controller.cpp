#include "controller/Controller.h"

void Controller::Pose_Callback(const robobuggy::Pose::ConstPtr& msg)
{
}


Controller::Controller()
{

}

void Controller::update_steering_estimate()
{

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

        double dx = (current_pose_estimate.longitude_deg - gpsPoseMessage.longitude_deg) * 84723.58765;
        double dy = (current_pose_estimate.latitude_deg - gpsPoseMessage.latitude_deg) * 111319.9;

        double d = std::sqrt(dx * dx + dy * dy);

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

}

bool Controller::get_deploy_brake_value()
{

}
