#include "roi_filter_mask.h"

#include <gtest/gtest.h>

using namespace lmt;

constexpr auto imageWidth{800};
constexpr auto imageHeight{400};
constexpr auto positiveOffset{1.0};
constexpr auto negativeOffset{-1.0};
constexpr auto positiveSteer{0.5};
constexpr auto negativeSteer{-0.5};

// tests of mask center
TEST(ROIFilterTest, calculateMaskCenter_middleCamera)
{
    const auto result = calculateMaskCenter(imageWidth, imageHeight, 0.0);

    EXPECT_EQ(cv::Point(400, 400), result);
}

TEST(ROIFilterTest, calculateMaskCenter_rightCamera)
{
    const auto result = calculateMaskCenter(imageWidth, imageHeight, positiveOffset);

    EXPECT_EQ(cv::Point(0, 400), result);
}

TEST(ROIFilterTest, calculateMaskCenter_leftCamera)
{
    const auto result = calculateMaskCenter(imageWidth, imageHeight, negativeOffset);

    EXPECT_EQ(cv::Point(800, 400), result);
}

TEST(ROIFilterTest, calculateMaskCenter_zeroSize)
{
    const auto result = calculateMaskCenter(0, 0, positiveOffset);

    EXPECT_EQ(cv::Point(0, 0), result);
}

// tests of mask size
TEST(ROIFilterTest, calculateMaskSize_middleCamera_zeroSteer)
{
    const auto result = calculateMaskSize(imageWidth, imageHeight, 0.0, 0.0);

    EXPECT_EQ(cv::Size(600, 300), result);
}

TEST(ROIFilterTest, calculateMaskSize_rightCamera_positiveSteer)
{
    const auto result = calculateMaskSize(imageWidth, imageHeight, positiveOffset, positiveSteer);

    EXPECT_EQ(cv::Size(1200, 300), result);
}

TEST(ROIFilterTest, calculateMaskSize_leftCamera_negativeSteer)
{
    const auto result = calculateMaskSize(imageWidth, imageHeight, negativeOffset, negativeSteer);

    EXPECT_EQ(cv::Size(1200, 300), result);
}

TEST(ROIFilterTest, calculateMaskSize_leftCamera_positiveSteer)
{
    const auto result = calculateMaskSize(imageWidth, imageHeight, negativeOffset, positiveSteer);

    EXPECT_EQ(cv::Size(1200, 300), result);
}

TEST(ROIFilterTest, calculateMaskSize_zeroSize)
{
    const auto result = calculateMaskSize(0, 0, negativeOffset, positiveSteer);

    EXPECT_EQ(cv::Size(0, 0), result);
}
