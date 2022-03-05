#include "frame_history.h"

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

using namespace lmt::vqm;

constexpr auto akiyo0Path = "vqm/resources/akiyo-0.png";
constexpr auto akiyo1Path = "vqm/resources/akiyo-1.png";
constexpr auto mobile0Path = "vqm/resources/mobile-0.png";
constexpr auto mobile1Path = "vqm/resources/mobile-1.png";

class FrameHistoryTest : public ::testing::Test
{
  protected:
    FrameHistoryTest() { cv::randn(imageWithActivity_, cv::Scalar::all(100), cv::Scalar::all(100)); }

    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const cv::Mat imageNoTA_{cv::Mat(10, 10, CV_8UC4)};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    cv::Mat imageWithActivity_{cv::Mat(10, 10, CV_8UC4)};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const cv::Mat akiyo0_{cv::imread(akiyo0Path)};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const cv::Mat akiyo1_{cv::imread(akiyo1Path)};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const cv::Mat mobile0_{cv::imread(mobile0Path)};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const cv::Mat mobile1_{cv::imread(mobile1Path)};
};

TEST_F(FrameHistoryTest, getYChannel_3ChannelIn_1ChannelOut)
{
    const auto actual = getYChannel(imageWithActivity_);

    EXPECT_EQ(1, actual.channels());
    EXPECT_EQ(CV_8UC1, actual.type());
}

TEST_F(FrameHistoryTest, getYChannel_1ChannelIn_unChanged)
{
    cv::Mat singleChannel(10, 10, CV_8UC1);

    const auto actual = getYChannel(singleChannel);

    EXPECT_EQ(1, actual.channels());
    EXPECT_EQ(CV_8UC1, actual.type());
}

TEST_F(FrameHistoryTest, getTemporalActivity_emptyOldImage_zeroTA)
{
    const auto actualTA = getTemporalActivity(imageNoTA_, {});

    EXPECT_DOUBLE_EQ(0, actualTA);
}

TEST_F(FrameHistoryTest, getTemporalActivity_noTemporalDiff_zeroTA)
{
    const auto actualTA = getTemporalActivity(imageNoTA_, imageNoTA_);

    EXPECT_EQ(0, actualTA);
}

TEST_F(FrameHistoryTest, getTemporalActivity_noTemporalDiffAkiyo_zeroTA)
{
    const auto actualTA = getTemporalActivity(akiyo0_, akiyo0_);

    EXPECT_EQ(0, actualTA);
}

TEST_F(FrameHistoryTest, getTemporalActivity_temporalDiff_nonZeroTA)
{
    const auto actualTA = getTemporalActivity(imageWithActivity_, imageNoTA_);

    EXPECT_NE(0, actualTA);
}

TEST_F(FrameHistoryTest, getTemporalActivity_realImagesAkiyo_almostNoTA)
{
    const auto actualTA = getTemporalActivity(akiyo1_, akiyo0_);

    EXPECT_LT(actualTA, 1);
}

TEST_F(FrameHistoryTest, DISABLED_getTemporalActivity_realImagesMobile_shouldBeRound24TA)
{
    const auto actualTA = getTemporalActivity(mobile1_, mobile0_);

    EXPECT_EQ(24, static_cast<int>(actualTA));
}

TEST_F(FrameHistoryTest, getMeanTemporalActivity_temporalActivityDiffButTooLongAgo_zeroTA)
{
    FrameHistory unit(0);

    unit.update(imageWithActivity_, 0);
    // Update once more because of diff to previous frame
    for (int i = 0; i < FrameHistory::activityMeasureHistorySize + 1; ++i)
    {
        if (i == 0)
        {
            EXPECT_EQ(0, unit.getTA());
            EXPECT_EQ(0, unit.getMeanTA());
        }
        else
        {
            EXPECT_NE(0, unit.getMeanTA());
        }
        unit.update(imageNoTA_, 0);
    }

    EXPECT_EQ(0, unit.getTA());
    EXPECT_EQ(0, unit.getMeanTA());
}

TEST_F(FrameHistoryTest, getSpatialActivity_monoChromeImages_zeroSA)
{
    EXPECT_EQ(0, getSpatialActivity(cv::Mat(10, 10, CV_8UC4, cv::Scalar::all(0))));
    EXPECT_EQ(0, getSpatialActivity(cv::Mat(10, 10, CV_8UC4, cv::Scalar::all(100))));
    EXPECT_EQ(0, getSpatialActivity(cv::Mat(10, 10, CV_8UC4, cv::Scalar::all(255))));
}

TEST_F(FrameHistoryTest, getSpatialActivity_randomImage_nonZeroSA)
{
    const auto actualSA = getSpatialActivity(imageWithActivity_);

    EXPECT_NE(0, actualSA);
}

TEST_F(FrameHistoryTest, DISABLED_getSpatialActivity_realImagesAkiyo_correctSA)
{
    EXPECT_EQ(64.6736, getSpatialActivity(akiyo0_));
    EXPECT_EQ(64.6685, getSpatialActivity(akiyo1_));
}

TEST_F(FrameHistoryTest, DISABLED_getSpatialActivity_realImagesMobile_correctSA)
{
    EXPECT_EQ(166.638, getSpatialActivity(mobile0_));
    EXPECT_EQ(166.878, getSpatialActivity(mobile1_));
}

TEST_F(FrameHistoryTest, getMeanSpatialActivity_spatialActivityDiffButTooLongAgo_zeroSA)
{
    FrameHistory unit(0);
    cv::Mat monochromeImage(10, 10, CV_8UC4, cv::Scalar::all(0));

    unit.update(imageWithActivity_, 0);
    for (int i = 0; i < FrameHistory::activityMeasureHistorySize; ++i)
    {
        EXPECT_NE(0, unit.getMeanSA());
        unit.update(monochromeImage, 0);
    }

    EXPECT_EQ(0, unit.getSA());
    EXPECT_EQ(0, unit.getMeanSA());
}

TEST_F(FrameHistoryTest, getFullFrameSize_with15x10_sizeShouldBe450)
{
    FrameHistory unit(0);
    cv::Mat testImage(15, 10, CV_8UC1);

    unit.update(testImage, 0);

    EXPECT_EQ(450, unit.getFullFrameSizeInPixel());
}

TEST_F(FrameHistoryTest, getEncodedVideoBitrate_withoutUpdates_bitrateShouldBe0)
{
    FrameHistory unit(20);
    cv::Mat testImage(15, 10, CV_8UC1);

    EXPECT_EQ(0, unit.getEncodedVideoBitrate());
}

TEST_F(FrameHistoryTest, getEncodedVideoBitrate_withOneUpdateOf100_bitrateShouldBe100)
{
    FrameHistory unit(20);
    cv::Mat testImage(15, 10, CV_8UC1);

    unit.update(testImage, 100);

    EXPECT_EQ(100, unit.getEncodedVideoBitrate());
}

TEST_F(FrameHistoryTest, getEncodedVideoBitrate_withOneSecondUpdates_bitrateShouldBeSummedUp)
{
    FrameHistory unit(3);  // That we only need to call 3 times the update
    cv::Mat testImage(15, 10, CV_8UC1);

    unit.update(testImage, 100);
    unit.update(testImage, 50);
    unit.update(testImage, 75);

    EXPECT_EQ(225, unit.getEncodedVideoBitrate());
}

TEST_F(FrameHistoryTest, getEncodedVideoBitrate_withMoreThanOneSecondUpdates_bitrateShouldBeLastSecondSum)
{
    FrameHistory unit(3);  // That we only need to call 3 times the update
    cv::Mat testImage(15, 10, CV_8UC1);

    unit.update(testImage, 100);
    unit.update(testImage, 50);
    unit.update(testImage, 75);
    unit.update(testImage, 80);

    EXPECT_EQ(205, unit.getEncodedVideoBitrate());
}
