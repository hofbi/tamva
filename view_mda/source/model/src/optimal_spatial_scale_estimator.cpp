#include "optimal_spatial_scale_estimator.h"

#include "mdvqm.h"

using namespace lmt::model;
using namespace lmt::vqm;

OptimalSpatialScaleEstimator::OptimalSpatialScaleEstimator(uint8_t bufferSize) noexcept
    : optimalSpatialScaleInPercentProposals_{bufferSize, 0}
{
}

void OptimalSpatialScaleEstimator::calculateOptimalSpatialScaleCandidate(const FrameHistory& frameHistory,
                                                                         uint8_t optimalFps) noexcept
{
    const auto svqs = [&]() {
        std::array<double, spatialScaleModes.size()> svqArray{};
        std::transform(
            spatialScaleModes.cbegin(), spatialScaleModes.cend(), svqArray.begin(), [&](double spatialScale) {
                const auto scaleFactor = spatialScale * spatialScale;  // width * height
                return calculateMDVQM(frameHistory, optimalFps, scaleFactor);
            });
        return svqArray;
    }();
    optimalSpatialScaleInPercentProposals_.push_back(
        100 * static_cast<uint8_t>(
                  spatialScaleModes[std::distance(svqs.cbegin(), std::max_element(svqs.cbegin(), svqs.cend()))]));
}

bool OptimalSpatialScaleEstimator::isProposalHistoryConstant() const noexcept
{
    return std::equal(optimalSpatialScaleInPercentProposals_.begin() + 1,
                      optimalSpatialScaleInPercentProposals_.end(),
                      optimalSpatialScaleInPercentProposals_.begin());
}

uint8_t OptimalSpatialScaleEstimator::getLatestOptimalSpatialScaleProposalInPercent() const noexcept
{
    return optimalSpatialScaleInPercentProposals_.back();
}
