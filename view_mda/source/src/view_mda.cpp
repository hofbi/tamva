#include "view_mda.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

using namespace lmt;

ViewMDA::ViewMDA(ros::NodeHandle& pnh) noexcept
    : mdaClient_(pnh.param("rate_control_topic", std::string("/server/front/rtsp_server"))),
      imgSub_(pnh.subscribe<sensor_msgs::Image>(pnh.param("image_topic", std::string("/carla/ego_vehicle/front/image")),
                                                1,
                                                &ViewMDA::imageCallback,
                                                this)),
      prioSub_(pnh.subscribe<view_adaptation_msgs::ViewPriority>(
          pnh.param("prio_topic", std::string("/carla/ego_vehicle/front/prioritization")),
          1,
          &ViewMDA::priorityCallback,
          this)),
      statsSub_(pnh.subscribe<telecarla_msgs::FrameStatistics>(
          pnh.param("frame_stats_topic", std::string("/server/front/rtsp_server/camera/stats")),
          1,
          &ViewMDA::frameStatisticsCallback,
          this)),
      frameStatistics_([]() {
          auto defaultValue = boost::make_shared<telecarla_msgs::FrameStatistics>();
          defaultValue->packetSize = gstreaming::RateControlConfig::__getDefault__().bitrate;
          defaultValue->frameType = telecarla_msgs::FrameStatistics::IFRAME;
          return defaultValue;
      }()),
      viewPriority_([]() {
          auto defaultValue = boost::make_shared<view_adaptation_msgs::ViewPriority>();
          defaultValue->weight = 0.0;
          defaultValue->bitrate = gstreaming::RateControlConfig::__getDefault__().bitrate;
          return defaultValue;
      }())
{
    ROS_INFO("Starting View MDAModel...");

    ROS_INFO_STREAM("Subscribed to gstreaming dynamic reconfigure server: " << mdaClient_.getName());
    ROS_INFO_STREAM("Subscribed to image topic: " << imgSub_.getTopic());
    ROS_INFO_STREAM("Subscribed to prioritization topic: " << prioSub_.getTopic());
    ROS_INFO_STREAM("Subscribed to frame statistics topic: " << statsSub_.getTopic());
}

void ViewMDA::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
    mdaModel_.update(image->image, frameStatistics_->packetSize);

    if (mdaModel_.isNewCodingMode())
    {
        setNewRateControlParameter();
    }
}

void ViewMDA::setNewRateControlParameter() noexcept
{
    gstreaming::RateControlConfig config;
    config.bitrate = viewPriority_->bitrate;
    config.fps = mdaModel_.getOptimalFps();
    config.spatial_scale = mdaModel_.getOptimalSpatialScaleInPercent();
    mdaClient_.setConfiguration(config);
}

void ViewMDA::priorityCallback(const view_adaptation_msgs::ViewPriorityConstPtr& msg)
{
    const auto lastBitrate = viewPriority_->bitrate;
    viewPriority_ = msg;
    if (lastBitrate != msg->bitrate)
    {
        setNewRateControlParameter();
    }
}

void ViewMDA::frameStatisticsCallback(const telecarla_msgs::FrameStatisticsConstPtr& msg)
{
    frameStatistics_ = msg;
}
