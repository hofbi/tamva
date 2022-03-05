#include "pixel_bitrate.h"

#include "frame_history.h"
#include "mdvqm_param.h"

using namespace lmt::vqm;

PixelBitrate::PixelBitrate(const FrameHistory& frameHistory) noexcept
    : bitratePerPixel_(static_cast<double>(frameHistory.getEncodedVideoBitrate()) /
                       (frameHistory.getFullFrameRate() * frameHistory.getFullFrameSizeInPixel())),
      ps_(exp(-1 * MDVQMParam::as * frameHistory.getMeanSA())),
      pt_(exp(-1 * MDVQMParam::at * frameHistory.getMeanTA()))
{
}

double PixelBitrate::getBitratePerPixel() const noexcept
{
    return bitratePerPixel_;
}

double PixelBitrate::getScaledBitratePerPixel(double spatialScaleFactor, double temporalScaleFactor) const noexcept
{
    return getBitratePerPixel() * pow(spatialScaleFactor, ps_) * pow(temporalScaleFactor, pt_);
}
