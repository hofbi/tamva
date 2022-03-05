#include "mda.h"

#include <gtest/gtest.h>

using namespace lmt::model;
using namespace lmt::vqm;

cv::Mat createTestImage(int width = 16, int height = 9)
{
    return cv::Mat(width, height, CV_8UC4);
}

TEST(MDATest, constructor_shouldNotBeANewCodingModeAndFullSpatioTemporalResolution)
{
    MDAModel unit;

    EXPECT_FALSE(unit.isNewCodingMode());
    EXPECT_EQ(100, unit.getOptimalSpatialScaleInPercent());
    EXPECT_EQ(20, unit.getOptimalFps());
}

TEST(MDATest, updateFrame_singleUpdate_shouldNotBeNewCodingMode)
{
    MDAModel unit;
    const auto image = createTestImage();

    unit.update(image, 1000);

    EXPECT_FALSE(unit.isNewCodingMode());
}
