#ifndef VIEW_MDA_OPTIMAL_FPS_ESTIMATOR_H
#define VIEW_MDA_OPTIMAL_FPS_ESTIMATOR_H

#include "frame_history.h"

namespace lmt
{
namespace model
{
class OptimalFpsEstimator
{
  public:
    explicit OptimalFpsEstimator(uint8_t bufferSize) noexcept;

    void calculateOptimalFpsCandidate(const vqm::FrameHistory& frameHistory) noexcept;

    bool isProposalHistoryConstant() const noexcept;
    uint8_t getLatestOptimalFpsProposal() const noexcept;

  private:
    static constexpr std::array<double, 6> temporalScaleModes{1.0, 0.75, 0.66, 0.5, 0.33, 0.25};

  private:
    boost::circular_buffer<uint8_t> optimalFpsProposals_;
};
}  // namespace model
}  // namespace lmt

#endif  // VIEW_MDA_OPTIMAL_FPS_ESTIMATOR_H
