#ifndef VIEW_MDA_VIEW_MDA_H
#define VIEW_MDA_VIEW_MDA_H

#include <dynamic_reconfigure/client.h>
#include <gstreaming/RateControlConfig.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <telecarla_msgs/FrameStatistics.h>
#include <view_adaptation_msgs/ViewPriority.h>

#include "mda.h"

namespace lmt
{
class ViewMDA
{
  public:
    explicit ViewMDA(ros::NodeHandle& pnh) noexcept;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void priorityCallback(const view_adaptation_msgs::ViewPriorityConstPtr& msg);
    void frameStatisticsCallback(const telecarla_msgs::FrameStatisticsConstPtr& msg);

  private:
    void setNewRateControlParameter() noexcept;

  private:
    dynamic_reconfigure::Client<gstreaming::RateControlConfig> mdaClient_;
    ros::Subscriber imgSub_;
    ros::Subscriber prioSub_;
    ros::Subscriber statsSub_;
    model::MDAModel mdaModel_;
    view_adaptation_msgs::ViewPriorityConstPtr viewPriority_{nullptr};
    telecarla_msgs::FrameStatisticsConstPtr frameStatistics_{nullptr};
};
}  // namespace lmt

#endif  // VIEW_MDA_VIEW_MDA_H
