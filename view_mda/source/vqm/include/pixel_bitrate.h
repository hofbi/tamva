#ifndef VIEW_MDA_PIXEL_BITRATE_H
#define VIEW_MDA_PIXEL_BITRATE_H

namespace lmt::vqm
{
class FrameHistory;

class PixelBitrate
{
  public:
    explicit PixelBitrate(const FrameHistory& frameHistory) noexcept;

    double getBitratePerPixel() const noexcept;
    double getScaledBitratePerPixel(double spatialScaleFactor, double temporalScaleFactor) const noexcept;

  private:
    double bitratePerPixel_{0.0};
    double ps_{0.0};
    double pt_{0.0};
};
}  // namespace lmt::vqm

#endif  // VIEW_MDA_PIXEL_BITRATE_H
