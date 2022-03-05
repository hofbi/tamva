#include "frame_history.h"

#include <numeric>

#include <opencv2/imgproc.hpp>

namespace lmt::vqm
{
FrameHistory::FrameHistory(uint8_t frameRateMax) noexcept : frameRateMax_(frameRateMax) {}

uint8_t FrameHistory::getFullFrameRate() const noexcept
{
    return frameRateMax_;
}

void FrameHistory::update(const cv::Mat& image, uint32_t encodedFrameSize)
{
    width_ = image.cols;
    height_ = image.rows;
    encodedFrameSizeBuffer_.push_back(encodedFrameSize);

    const auto yChannel = getYChannel(image);
    spatialActivityHistory_.push_back(getSpatialActivity(yChannel));
    temporalActivityHistory_.push_back(getTemporalActivity(yChannel, oldImage_));
    oldImage_ = yChannel;
}

double FrameHistory::getMeanTA() const noexcept
{
    return std::accumulate(temporalActivityHistory_.begin(), temporalActivityHistory_.end(), 0.0) /
           static_cast<double>(temporalActivityHistory_.size());
}

double FrameHistory::getMeanSA() const noexcept
{
    return std::accumulate(spatialActivityHistory_.begin(), spatialActivityHistory_.end(), 0.0) /
           static_cast<double>(spatialActivityHistory_.size());
}

double FrameHistory::getTA() const noexcept
{
    return temporalActivityHistory_.back();
}

double FrameHistory::getSA() const noexcept
{
    return spatialActivityHistory_.back();
}

uint16_t FrameHistory::getFullFrameSizeInPixel() const noexcept
{
    // YCbCr as 3 color channels
    return width_ * height_ * 3;
}

uint32_t FrameHistory::getEncodedVideoBitrate() const noexcept
{
    return std::accumulate(encodedFrameSizeBuffer_.begin(), encodedFrameSizeBuffer_.end(), 0U);
}

double getSpatialActivity(const cv::Mat& image)
{
    cv::Mat gradX;
    cv::Mat gradY;
    cv::Mat grad;

    cv::Sobel(getYChannel(image), gradX, CV_32F, 1, 0);
    cv::Sobel(getYChannel(image), gradY, CV_32F, 0, 1);
    cv::add(gradX.mul(gradX), gradY.mul(gradY), grad);
    cv::sqrt(grad, grad);

    return getStdDev(grad);
}

double getTemporalActivity(const cv::Mat& image, const cv::Mat& oldImage)
{
    if (oldImage.empty())
    {
        return 0.0;
    }

    cv::Mat diff;
    cv::absdiff(getYChannel(image), getYChannel(oldImage), diff);
    return getStdDev(diff);
}

double getStdDev(const cv::Mat& image)
{
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(image, mean, stddev);

    return stddev[0];
}

cv::Mat getYChannel(const cv::Mat& image)
{
    if (image.channels() == 1)
    {
        return image;
    }

    cv::Mat yChannel;
    // CARLA uses bgra8 color format
    cv::cvtColor(image, yChannel, cv::COLOR_BGRA2GRAY);
    return yChannel;
}
}  // namespace lmt::vqm
