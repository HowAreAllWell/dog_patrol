#include <gtest/gtest.h>

#include <limits>

#include "vision_demo_host/modules/preprocess_infer.hpp"

namespace vision_demo_host {

TEST(PreprocessInferTimingTest, ReportsNearestRankStagePercentiles) {
  PreprocessInfer::StageTiming timing;
  for (int value = 1; value <= 100; ++value) {
    timing.ObserveMilliseconds(static_cast<double>(value));
  }

  const auto summary = timing.Summary();
  EXPECT_EQ(summary.samples, 100U);
  EXPECT_DOUBLE_EQ(summary.p50_ms, 50.0);
  EXPECT_DOUBLE_EQ(summary.p95_ms, 95.0);
  EXPECT_DOUBLE_EQ(summary.p99_ms, 99.0);
}

TEST(PreprocessInferTimingTest, IgnoresInvalidMeasurementsAndCanReset) {
  PreprocessInfer::StageTiming timing;
  timing.ObserveMilliseconds(-1.0);
  timing.ObserveMilliseconds(std::numeric_limits<double>::quiet_NaN());
  timing.ObserveMilliseconds(7.0);
  EXPECT_EQ(timing.Summary().samples, 1U);

  timing.Clear();
  const auto summary = timing.Summary();
  EXPECT_EQ(summary.samples, 0U);
  EXPECT_DOUBLE_EQ(summary.p50_ms, 0.0);
}

}  // namespace vision_demo_host
