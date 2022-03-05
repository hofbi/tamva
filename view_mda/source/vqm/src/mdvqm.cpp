#include "mdvqm.h"

#include "frame_history.h"
#include "mdvqm_param.h"
#include "pixel_bitrate.h"

namespace lmt::vqm
{
double calculateMDVQM(const FrameHistory& frameHistory, uint8_t frameRate, double spatialScaleFactor) noexcept
{
    const auto sa = frameHistory.getMeanSA();
    const auto ta = frameHistory.getMeanTA();

    const auto bpp = PixelBitrate(frameHistory)
                         .getScaledBitratePerPixel(spatialScaleFactor,
                                                   static_cast<double>(frameHistory.getFullFrameRate()) / frameRate);

    const auto snrvq = calculateSNRVQ(bpp, sa, ta);
    const auto spatialCorrectionFactor = calculateSpatialCorrectionFactor(spatialScaleFactor, MDVQMParam::bs, sa);
    const auto temporalCorrectionFactor =
        calculateTemporalCorrectionFactor(frameRate, frameHistory.getFullFrameRate(), MDVQMParam::bt, ta);

    return snrvq * std::min(spatialCorrectionFactor, temporalCorrectionFactor);
}

double calculateSNRVQ(double bpp, double sa, double ta) noexcept
{
    const auto m = pow(ta, MDVQMParam::a[0]) * pow(sa, MDVQMParam::a[1]) * MDVQMParam::a[2];
    return 100.0 / (1 + exp(-1 * (m * log(bpp) + MDVQMParam::a[3] * sa + MDVQMParam::a[4] * ta + MDVQMParam::a[5])));
}

double calculateSpatialCorrectionFactor(double spatialScaleFactor, double bs, double sa) noexcept
{
    return std::pow(spatialScaleFactor, bs * sa);
}

double calculateTemporalCorrectionFactor(double frameRate, double frameRateMax, double bt, double ta) noexcept
{
    return (frameRate / frameRateMax) * ((1 + bt * frameRateMax / ta) / (1 + bt * frameRate / ta));
}
}  // namespace lmt::vqm
