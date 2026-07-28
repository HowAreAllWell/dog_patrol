#include "vision_demo_host/modules/camera_ingest.hpp"

#include <gtest/gtest.h>

#include <string>

namespace vision_demo_host {
namespace {

TEST(CameraIngestContractTest, ValidatesExplicitBayerConfiguration) {
  CameraIngest::Config config;
  std::string error;

  EXPECT_TRUE(CameraIngest::ValidateConfig(config, &error));
  EXPECT_TRUE(error.empty());

  config.bayer_interpolation = static_cast<CameraIngest::BayerInterpolation>(99);
  EXPECT_FALSE(CameraIngest::ValidateConfig(config, &error));
  EXPECT_NE(error.find("bayer_interpolation"), std::string::npos);

  config = CameraIngest::Config{};
  config.fps = 0.0;
  EXPECT_FALSE(CameraIngest::ValidateConfig(config, &error));
  EXPECT_NE(error.find("fps"), std::string::npos);
}

TEST(CameraIngestContractTest, ParsesSupportedBayerInterpolationNames) {
  CameraIngest::BayerInterpolation interpolation{};
  std::string error;

  EXPECT_TRUE(CameraIngest::ParseBayerInterpolation("fast", &interpolation, &error));
  EXPECT_EQ(interpolation, CameraIngest::BayerInterpolation::kFast);
  EXPECT_TRUE(
      CameraIngest::ParseBayerInterpolation("optimal_plus", &interpolation, &error));
  EXPECT_EQ(interpolation, CameraIngest::BayerInterpolation::kOptimalPlus);
  EXPECT_FALSE(CameraIngest::ParseBayerInterpolation("nearest", &interpolation, &error));
  EXPECT_NE(error.find("nearest"), std::string::npos);
  EXPECT_FALSE(CameraIngest::ParseBayerInterpolation("fast", nullptr, &error));
  EXPECT_NE(error.find("null"), std::string::npos);
}

TEST(CameraIngestContractTest, NamesKnownAndUnknownMvsPixelFormats) {
  EXPECT_EQ(CameraIngest::PixelTypeName(0x01080009U), "BayerRG8");
  EXPECT_EQ(CameraIngest::PixelTypeName(0x02180015U), "BGR8_Packed");
  EXPECT_EQ(CameraIngest::PixelTypeName(0xDEADBEEFU), "Unknown(0xdeadbeef)");
}

TEST(CameraIngestContractTest, CountsNonContiguousCameraFramesWithoutTreatingWrapAsDrop) {
  CameraIngest::FrameContinuity continuity;

  EXPECT_EQ(continuity.Observe(41U), 0U);
  EXPECT_EQ(continuity.Observe(42U), 0U);
  EXPECT_EQ(continuity.Observe(45U), 2U);
  EXPECT_EQ(continuity.DroppedFrames(), 2U);
  EXPECT_EQ(continuity.NonContiguousFrames(), 1U);

  CameraIngest::FrameContinuity wrapping;
  EXPECT_EQ(wrapping.Observe(0xFFFFFFFFU), 0U);
  EXPECT_EQ(wrapping.Observe(0U), 0U);
  EXPECT_EQ(wrapping.DroppedFrames(), 0U);
}

TEST(CameraIngestContractTest, ReportsStageLatencyPercentiles) {
  CameraIngest::StageTiming timing;
  for (int value = 1; value <= 100; ++value) {
    timing.ObserveMilliseconds(static_cast<double>(value));
  }

  const auto summary = timing.Summary();
  EXPECT_EQ(summary.samples, 100U);
  EXPECT_DOUBLE_EQ(summary.p50_ms, 50.0);
  EXPECT_DOUBLE_EQ(summary.p95_ms, 95.0);
  EXPECT_DOUBLE_EQ(summary.p99_ms, 99.0);
}

}  // namespace
}  // namespace vision_demo_host
