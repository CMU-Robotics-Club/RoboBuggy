#include "transistor/localizer/Localizer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, Localizer::NODE_NAME);
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    Localizer l;

    while(ros::ok())
    {
        l.update_position_estimate();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
