#include "optimal_fps_estimator.h"

#include "mdvqm.h"

using namespace lmt::model;
using namespace lmt::vqm;

OptimalFpsEstimator::OptimalFpsEstimator(uint8_t bufferSize) noexcept : optimalFpsProposals_{bufferSize, 0} {}

void OptimalFpsEstimator::calculateOptimalFpsCandidate(const FrameHistory& frameHistory) noexcept
{
    const auto tvqs = [&]() {
        std::array<double, temporalScaleModes.size()> tvqArray{};
        std::transform(
            temporalScaleModes.cbegin(), temporalScaleModes.cend(), tvqArray.begin(), [&](double temporalScale) {
                return calculateMDVQM(frameHistory,
                                      static_cast<uint8_t>(frameHistory.getFullFrameRate() * temporalScale));
            });
        return tvqArray;
    }();
    optimalFpsProposals_.push_back(static_cast<uint8_t>(
        frameHistory.getFullFrameRate() *
        temporalScaleModes[std::distance(tvqs.cbegin(), std::max_element(tvqs.cbegin(), tvqs.cend()))]));
}

bool OptimalFpsEstimator::isProposalHistoryConstant() const noexcept
{
    return std::equal(optimalFpsProposals_.begin() + 1, optimalFpsProposals_.end(), optimalFpsProposals_.begin());
}

uint8_t OptimalFpsEstimator::getLatestOptimalFpsProposal() const noexcept
{
    return optimalFpsProposals_.back();
}
