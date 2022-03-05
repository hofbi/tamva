#ifndef VIEW_MDA_FRAME_HISTORY_H
#define VIEW_MDA_FRAME_HISTORY_H

#include <boost/circular_buffer.hpp>
#include <opencv2/core/mat.hpp>

namespace lmt
{
namespace vqm
{
class FrameHistory
{
  public:
    explicit FrameHistory(uint8_t frameRateMax) noexcept;

    uint16_t getFullFrameSizeInPixel() const noexcept;
    uint8_t getFullFrameRate() const noexcept;
    uint32_t getEncodedVideoBitrate() const noexcept;

    double getMeanSA() const noexcept;
    double getMeanTA() const noexcept;
    double getSA() const noexcept;
    double getTA() const noexcept;

    void update(const cv::Mat& image, uint32_t encodedFrameSize);

  public:
    static constexpr uint8_t activityMeasureHistorySize{5};  // Defined in the PhD Thesis of Fan Zhang

  private:
    uint16_t width_{640};
    uint16_t height_{480};
    uint8_t frameRateMax_{20};
    boost::circular_buffer<double> spatialActivityHistory_{activityMeasureHistorySize, 0.0};
    boost::circular_buffer<double> temporalActivityHistory_{activityMeasureHistorySize, 0.0};
    boost::circular_buffer<uint32_t> encodedFrameSizeBuffer_{frameRateMax_, 0};
    cv::Mat oldImage_;
};

double getSpatialActivity(const cv::Mat& image);
double getTemporalActivity(const cv::Mat& image, const cv::Mat& oldImage);
double getStdDev(const cv::Mat& image);
cv::Mat getYChannel(const cv::Mat& image);
}  // namespace vqm
}  // namespace lmt

#endif  // VIEW_MDA_FRAME_HISTORY_H
