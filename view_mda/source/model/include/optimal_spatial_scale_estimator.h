#ifndef VIEW_MDA_OPTIMAL_SPATIAL_SCALE_ESTIMATOR_H
#define VIEW_MDA_OPTIMAL_SPATIAL_SCALE_ESTIMATOR_H

#include "frame_history.h"

namespace lmt
{
namespace model
{
class OptimalSpatialScaleEstimator
{
  public:
    explicit OptimalSpatialScaleEstimator(uint8_t bufferSize) noexcept;

    void calculateOptimalSpatialScaleCandidate(const vqm::FrameHistory& frameHistory, uint8_t optimalFps) noexcept;

    bool isProposalHistoryConstant() const noexcept;
    uint8_t getLatestOptimalSpatialScaleProposalInPercent() const noexcept;

  private:
    static constexpr std::array<double, 4> spatialScaleModes{1.0, 0.75, 0.66, 0.5};

  private:
    boost::circular_buffer<uint8_t> optimalSpatialScaleInPercentProposals_;
};
}  // namespace model
}  // namespace lmt

#endif  // VIEW_MDA_OPTIMAL_SPATIAL_SCALE_ESTIMATOR_H
