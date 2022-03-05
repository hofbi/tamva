#ifndef VIEW_MDA_MDA_H
#define VIEW_MDA_MDA_H

#include "optimal_fps_estimator.h"
#include "optimal_spatial_scale_estimator.h"

namespace lmt
{
namespace model
{
class MDAModel
{
  public:
    void update(const cv::Mat& image, uint32_t encodedFrameSize);

    bool isNewCodingMode() const noexcept;

    uint8_t getOptimalFps() const noexcept;
    uint8_t getOptimalSpatialScaleInPercent() const noexcept;

  private:
    void updateInternalState(const cv::Mat& image, uint32_t encodedFrameSize);
    void calculateOptimalProposals();
    void resetCounter();
    void setNewCodingMode();
    bool isBelowMinimumWaitingTime();
    bool isProposalChanging();
    bool isProposalHistroyConstant();

  private:
    static constexpr uint8_t qualityChangeThreshold{5};
    static constexpr uint8_t maxFrameRate{20};  // Max rate of the ros bridge

  private:
    vqm::FrameHistory frameHistory_{maxFrameRate};
    bool newCodingMode_{false};
    uint8_t optimalFps_{frameHistory_.getFullFrameRate()};
    uint8_t optimalSpatialScaleInPercent_{100};
    OptimalFpsEstimator optimalFpsEstimator_{qualityChangeThreshold};
    OptimalSpatialScaleEstimator optimalSpatialScaleEstimator_{qualityChangeThreshold};
    uint32_t minimumWaitingTimeCounter_{0};
};
}  // namespace model
}  // namespace lmt

#endif  // VIEW_MDA_MDA_H
