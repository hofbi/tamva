#include "mda.h"

#include "mdvqm.h"

namespace lmt::model
{
bool MDAModel::isNewCodingMode() const noexcept
{
    return newCodingMode_;
}

void MDAModel::update(const cv::Mat& image, uint32_t encodedFrameSize)
{
    updateInternalState(image, encodedFrameSize);

    if (isBelowMinimumWaitingTime())
    {
        return;
    }

    calculateOptimalProposals();

    if (isProposalHistroyConstant() && isProposalChanging())
    {
        resetCounter();
        setNewCodingMode();
    }
}

void MDAModel::updateInternalState(const cv::Mat& image, uint32_t encodedFrameSize)
{
    frameHistory_.update(image, encodedFrameSize);
    newCodingMode_ = false;
    ++minimumWaitingTimeCounter_;
}

void MDAModel::calculateOptimalProposals()
{
    optimalFpsEstimator_.calculateOptimalFpsCandidate(frameHistory_);
    optimalSpatialScaleEstimator_.calculateOptimalSpatialScaleCandidate(
        frameHistory_, optimalFpsEstimator_.getLatestOptimalFpsProposal());
}

bool MDAModel::isProposalChanging()
{
    return optimalFps_ != optimalFpsEstimator_.getLatestOptimalFpsProposal() ||
           optimalSpatialScaleInPercent_ !=
               optimalSpatialScaleEstimator_.getLatestOptimalSpatialScaleProposalInPercent();
}

bool MDAModel::isProposalHistroyConstant()
{
    return optimalFpsEstimator_.isProposalHistoryConstant() &&
           optimalSpatialScaleEstimator_.isProposalHistoryConstant();
}

void MDAModel::resetCounter()
{
    minimumWaitingTimeCounter_ = 0;
}

void MDAModel::setNewCodingMode()
{
    newCodingMode_ = true;
    optimalFps_ = optimalFpsEstimator_.getLatestOptimalFpsProposal();
    optimalSpatialScaleInPercent_ = optimalSpatialScaleEstimator_.getLatestOptimalSpatialScaleProposalInPercent();
}

bool MDAModel::isBelowMinimumWaitingTime()
{
    // Waiting time according to dissertation Fan Zhang (1 sec => full frame rate steps)
    return minimumWaitingTimeCounter_ < frameHistory_.getFullFrameRate();
}

uint8_t MDAModel::getOptimalFps() const noexcept
{
    return optimalFps_;
}

uint8_t MDAModel::getOptimalSpatialScaleInPercent() const noexcept
{
    return optimalSpatialScaleInPercent_;
}
}  // namespace lmt::model
