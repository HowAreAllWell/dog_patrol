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

TEST(CameraIngestContractTest, DefaultsToBalancedBayerWithoutSmoothing) {
  const CameraIngest::Config config;

  EXPECT_EQ(config.bayer_interpolation, CameraIngest::BayerInterpolation::kBalanced);
  EXPECT_FALSE(config.bayer_smoothing);
}

TEST(CameraIngestContractTest, ParsesEverySupportedBayerInterpolationOverride) {
  CameraIngest::BayerInterpolation interpolation{};
  std::string error;

  EXPECT_TRUE(CameraIngest::ParseBayerInterpolation("fast", &interpolation, &error));
  EXPECT_EQ(interpolation, CameraIngest::BayerInterpolation::kFast);
  EXPECT_TRUE(CameraIngest::ParseBayerInterpolation("balanced", &interpolation, &error));
  EXPECT_EQ(interpolation, CameraIngest::BayerInterpolation::kBalanced);
  EXPECT_TRUE(CameraIngest::ParseBayerInterpolation("optimal", &interpolation, &error));
  EXPECT_EQ(interpolation, CameraIngest::BayerInterpolation::kOptimal);
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

TEST(CameraIngestContractTest, ProjectsSdkIndependentSourceFrameMetadata) {
  CameraIngest::SourceFrameMetadata metadata;
  metadata.source_timestamp_ns = 1'785'216'415'805'346'555ULL;
  metadata.sdk_host_timestamp = 1'785'216'415'795LL;
  metadata.camera_frame_number = 32U;
  metadata.device_timestamp_ticks = 1'367'135'394'241ULL;
  metadata.source_pixel_type = 0x0108000AU;
  metadata.width = 1280;
  metadata.height = 1024;
  metadata.source_payload_bytes = 1'310'720U;
  metadata.camera_lost_packets = 3U;

  CameraIngest::AcquiredFrame frame;
  frame.bgr8 = cv::Mat(2, 3, CV_8UC3);
  CameraIngest::ApplySourceFrameMetadata(metadata, &frame);

  EXPECT_EQ(frame.source_timestamp_ns, metadata.source_timestamp_ns);
  EXPECT_EQ(frame.sdk_host_timestamp, metadata.sdk_host_timestamp);
  EXPECT_TRUE(frame.camera_frame_number_available);
  EXPECT_EQ(frame.camera_frame_number, 32U);
  EXPECT_EQ(frame.device_timestamp_ticks, metadata.device_timestamp_ticks);
  EXPECT_EQ(frame.source_pixel_type, 0x0108000AU);
  EXPECT_EQ(frame.source_pixel_type_name, "BayerGB8");
  EXPECT_EQ(frame.width, 1280);
  EXPECT_EQ(frame.height, 1024);
  EXPECT_EQ(frame.source_payload_bytes, 1'310'720U);
  EXPECT_EQ(frame.camera_lost_packets, 3U);
  EXPECT_EQ(frame.bgr8.type(), CV_8UC3);
  EXPECT_EQ(frame.bgr8.size(), cv::Size(3, 2));
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
