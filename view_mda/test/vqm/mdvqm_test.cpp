#include "mdvqm.h"

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

#include "frame_history.h"
#include "mdvqm_param.h"

using namespace lmt::vqm;

FrameHistory createFrameHistory(uint32_t encodedFrameSize = 5000)
{
    FrameHistory frameHistory(20);
    cv::Mat imageWithRandomActivity(640, 480, CV_8UC4);
    for (int i = 0; i < frameHistory.getFullFrameRate(); ++i)
    {
        cv::randn(imageWithRandomActivity, cv::Scalar::all(100), cv::Scalar::all(100));
        frameHistory.update(imageWithRandomActivity, encodedFrameSize);
    }
    return frameHistory;
}

TEST(MDVQMTest, getSpatialCorrectionFactor_spatialScaleFactorOne_shouldBeAlmostOne)
{
    const auto actualSCF = calculateSpatialCorrectionFactor(1, MDVQMParam::bs, 1);

    EXPECT_DOUBLE_EQ(1, actualSCF);
}

TEST(MDVQMTest, getSpatialCorrectionFactor_spatialActivityZero_shouldBeAlmostOne)
{
    const auto actualSCF = calculateSpatialCorrectionFactor(0.5, MDVQMParam::bs, 0);

    EXPECT_DOUBLE_EQ(1, actualSCF);
}

TEST(MDVQMTest, getSpatialCorrectionFactor_spatialScaleFactorBetweenZeroAndOne_shouldBeLessThanOne)
{
    const auto actualSCF = calculateSpatialCorrectionFactor(0.5, MDVQMParam::bs, 1);

    EXPECT_LT(actualSCF, 1);
}

TEST(MDVQMTest, getTemporalCorrectionFactor_fullFrameRate_shouldBeAlmostOne)
{
    const auto actualTCF = calculateTemporalCorrectionFactor(20, 20, MDVQMParam::bt, 1);

    EXPECT_DOUBLE_EQ(1, actualTCF);
}

TEST(MDVQMTest, getTemporalCorrectionFactor_temporalActivityAlmostZero_shouldBeAlmostOne)
{
    const auto actualTCF = calculateTemporalCorrectionFactor(20, 20, MDVQMParam::bt, 1e-15);

    EXPECT_DOUBLE_EQ(1, actualTCF);
}

TEST(MDVQMTest, getTemporalCorrectionFactor_lessThanFullFrameRate_shouldBeLessThanOne)
{
    const auto actualTCF = calculateTemporalCorrectionFactor(10, 20, MDVQMParam::bt, 1);

    EXPECT_LE(actualTCF, 1);
}

TEST(MDVQMTest, calculateSNRVQ_spatialActivityAlmostZero_shouldBeAlmost100)
{
    const auto actualSNRVQ = calculateSNRVQ(0.5, 1e-15, 10);

    EXPECT_NEAR(100, actualSNRVQ, 0.2);
}

TEST(MDVQMTest, calculateSNRVQ_temporalActivityAlmostZero_shouldBe0)
{
    const auto actualSNRVQ = calculateSNRVQ(0.5, 10, 1e-15);

    EXPECT_EQ(0, actualSNRVQ);
}

TEST(MDVQMTest, calculateMDVQM_emptyStateAndFullRateAndScale_shouldBeNan)
{
    FrameHistory frameHistory(20);

    const auto actualMDVQM = calculateMDVQM(frameHistory, frameHistory.getFullFrameRate());

    EXPECT_NE(actualMDVQM, actualMDVQM);
}

TEST(MDVQMTest, calculateMDVQM_highBitrateAndFullRateAndFullScale_shouldBeAlmost100)
{
    const auto frameHistory = createFrameHistory(5000);

    const auto actualMDVQM = calculateMDVQM(frameHistory, frameHistory.getFullFrameRate());

    EXPECT_NEAR(100, actualMDVQM, 0.1);
}

TEST(MDVQMTest, calculateMDVQM_highBitrateAndHalfRateAndFullScale_shouldBeLessThan60)
{
    const auto frameHistory = createFrameHistory(5000);

    const auto actualMDVQM = calculateMDVQM(frameHistory, 10);

    EXPECT_LT(actualMDVQM, 60);
}

TEST(MDVQMTest, calculateMDVQM_highBitrateAndFullRateAndHalfScale_shouldBeLessThan80)
{
    const auto frameHistory = createFrameHistory(5000);

    const auto actualMDVQM = calculateMDVQM(frameHistory, frameHistory.getFullFrameRate(), 0.5);

    EXPECT_LT(actualMDVQM, 80);
}

TEST(MDVQMTest, calculateMDVQM_lowBitrateAndFullRateAndFullScale_shouldBeLessThan60)
{
    const auto frameHistory = createFrameHistory(2000);

    const auto actualMDVQM = calculateMDVQM(frameHistory, frameHistory.getFullFrameRate());

    EXPECT_LT(actualMDVQM, 60);
}
