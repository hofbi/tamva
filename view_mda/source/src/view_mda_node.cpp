#include <ros/ros.h>

#include "view_mda.h"

using namespace lmt;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "view_mda");
    ros::NodeHandle pnh("~");

    ViewMDA viewMDA(pnh);
    ros::spin();

    return EXIT_SUCCESS;
}
