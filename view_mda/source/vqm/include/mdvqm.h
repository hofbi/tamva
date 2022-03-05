#ifndef VIEW_MDA_MDVQM_H
#define VIEW_MDA_MDVQM_H

#include <cstdint>

namespace lmt::vqm
{
class FrameHistory;

double calculateMDVQM(const FrameHistory& frameHistory, uint8_t frameRate, double spatialScaleFactor = 1.0) noexcept;
double calculateSNRVQ(double bpp, double sa, double ta) noexcept;
double calculateSpatialCorrectionFactor(double spatialScaleFactor, double bs, double sa) noexcept;
double calculateTemporalCorrectionFactor(double frameRate, double frameRateMax, double bt, double ta) noexcept;
}  // namespace lmt::vqm

#endif  // VIEW_MDA_MDVQM_H
