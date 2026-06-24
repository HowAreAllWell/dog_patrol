#include <gtest/gtest.h>

#include <numeric>

#include <opencv2/core.hpp>

#include "vision_demo_host/modules/appearance_feature_service.hpp"

namespace {

cv::Mat MakeColorPatchFrame() {
  cv::Mat frame(64, 48, CV_8UC3, cv::Scalar(24, 48, 96));
  frame(cv::Rect(0, 0, frame.cols, frame.rows / 2)).setTo(cv::Scalar(180, 40, 20));
  frame(cv::Rect(0, frame.rows / 2, frame.cols, frame.rows / 2)).setTo(cv::Scalar(20, 180, 40));
  return frame;
}

}  // namespace

TEST(AppearanceFeatureServiceTest, LightTrackerFeatureReturnsNormalizedMatrix) {
  vision_demo_host::AppearanceFeatureService service(
      vision_demo_host::AppearanceFeatureService::Config{"light", "", 128, 256, 8, 4},
      vision_demo_host::AppearanceFeatureService::Profile::kTracker);

  std::string error;
  ASSERT_TRUE(service.Initialize(&error)) << error;
  EXPECT_FALSE(service.UsesOnnx());

  const cv::Mat feature = service.ExtractTrackerFeature(MakeColorPatchFrame(), cv::Rect2f(4, 4, 32, 48));
  ASSERT_FALSE(feature.empty());
  EXPECT_EQ(feature.rows, 1);
  EXPECT_EQ(feature.cols, 64);
  EXPECT_NEAR(cv::sum(feature)[0], 1.0, 1e-5);
}

TEST(AppearanceFeatureServiceTest, LightIdentityFeatureReturnsHistogramVector) {
  vision_demo_host::AppearanceFeatureService service(
      vision_demo_host::AppearanceFeatureService::Config{"light", "", 128, 256, 8, 4},
      vision_demo_host::AppearanceFeatureService::Profile::kIdentity);

  std::string error;
  ASSERT_TRUE(service.Initialize(&error)) << error;

  const auto feature = service.ExtractIdentityFeature(MakeColorPatchFrame(), cv::Rect2f(4, 4, 32, 48));
  ASSERT_EQ(feature.size(), 24U);
  const float sum = std::accumulate(feature.begin(), feature.end(), 0.0F);
  EXPECT_NEAR(sum, 3.0F, 1e-5F);
}
