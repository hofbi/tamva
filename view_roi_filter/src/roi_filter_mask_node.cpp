#include <ros/ros.h>

#include "roi_filter_mask.h"

using namespace lmt;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "view_roi_filter");

    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle pnh = ros::NodeHandle("~");

    ROIFilter roiFilter(nh, pnh);

    ros::spin();

    return 0;
}
