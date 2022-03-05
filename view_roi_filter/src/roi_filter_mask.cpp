
#include "roi_filter_mask.h"

#include <cmath>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

// using namespace lmt;
namespace lmt
{
ROIFilter::ROIFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : centerOffset_(pnh.param("center_offset", 0)), blockBasedMode_(pnh.param("block_based_mode", false))
{
    ROS_INFO("Starting ROI Filter...");
    std::string imgTopic;
    if (!pnh.getParam("image_topic", imgTopic))
    {
        ROS_ERROR("No input image topic provided. Set argument image_topic");
        return;
    }
    imgSub_ = nh.subscribe<sensor_msgs::Image>(imgTopic, 1, &ROIFilter::imageCallback, this);

    std::string statusTopic;
    if (!pnh.getParam("status_topic", statusTopic))
    {
        ROS_ERROR("No input status topic provided. Set argument status_topic");
        return;
    }
    statusSub_ = nh.subscribe<carla_msgs::CarlaEgoVehicleStatus>(statusTopic, 1, &ROIFilter::statusCallback, this);

    imgPub_ = pnh.advertise<sensor_msgs::Image>(pnh.getNamespace() + imgTopic, 1);
}

void ROIFilter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    auto imgPtr = cv_bridge::toCvShare(msg);

    const auto imgWidth = imgPtr->image.cols;
    const auto imgHeight = imgPtr->image.rows;
    constexpr auto endDegree = 360;
    constexpr auto maskColor = 255;
    cv::Mat mask(imgHeight, imgWidth, CV_8UC1, cv::Scalar(0));

    cv::ellipse(mask,
                calculateMaskCenter(imgWidth, imgHeight, centerOffset_),
                calculateMaskSize(imgWidth, imgHeight, centerOffset_, steer_),
                0,
                0,
                endDegree,
                cv::Scalar(maskColor),
                cv::FILLED);

    if (blockBasedMode_)
    {
        constexpr auto blockSize = 4;
        const auto numBlocksX = imgWidth / blockSize;
        const auto numBlocksY = imgHeight / blockSize;
        if (imgWidth % blockSize != 0)
        {
            ROS_ERROR_STREAM("Please type the right value of 'blockSize'. "
                             << blockSize << " is not the divisor of the image width " << imgWidth);
        }
        const cv::Mat blackBlock(blockSize, blockSize, CV_8UC1, cv::Scalar(0));
        const cv::Mat whiteBlock(blockSize, blockSize, CV_8UC1, cv::Scalar(maskColor));

        for (auto x = 0; x < numBlocksX; ++x)
        {
            for (auto y = 0; y < numBlocksY; ++y)
            {
                cv::Mat block =
                    mask(cv::Range(y * blockSize, (y + 1) * blockSize), cv::Range(x * blockSize, (x + 1) * blockSize));
                const auto count = cv::countNonZero(block);
                if (count != 0 && count != blockSize * blockSize)
                {
                    count > blockSize* blockSize / 2 ? whiteBlock.copyTo(block) : blackBlock.copyTo(block);
                }
            }
        }
    }

    auto imgRoiPtr = std::make_unique<cv_bridge::CvImage>(imgPtr->header, imgPtr->encoding);
    imgPtr->image.copyTo(imgRoiPtr->image, mask);
    imgPub_.publish(imgRoiPtr->toImageMsg());
}

void ROIFilter::statusCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr& msg)
{
    steer_ = msg->control.steer;
}

cv::Point calculateMaskCenter(int width, int height, double offset)
{
    return {static_cast<int>((-offset + 1) * width / 2), height};
}

cv::Size calculateMaskSize(int width, int height, double offset, double steer)
{
    const auto widthBias = static_cast<int>((fabs(steer) + fabs(offset)) * width / 2);
    return {width * 3 / 4 + widthBias, height * 3 / 4};
}

}  // namespace lmt
