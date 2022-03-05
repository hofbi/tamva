#ifndef ROI_FILTER_ROI_FILTER_MASK_H
#define ROI_FILTER_ROI_FILTER_MASK_H

#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <opencv2/core/types.hpp>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>

namespace lmt
{
class ROIFilter
{
  public:
    ROIFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void statusCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr& msg);

  private:
    ros::Publisher imgPub_;
    ros::Subscriber imgSub_;
    ros::Subscriber statusSub_;
    double steer_{0.0};
    double centerOffset_{0.0};
    bool blockBasedMode_{false};
};

cv::Point calculateMaskCenter(int width, int height, double offset);
cv::Size calculateMaskSize(int width, int height, double offset, double steer);
}  // namespace lmt

#endif  // ROI_FILTER_ROI_FILTER_MASK_H
