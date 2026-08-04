#include <gtest/gtest.h>

#include "dog_patrol_perception_tracking/modules/perception_readiness.hpp"

namespace {

using dog_patrol_perception_tracking::DetectionTrackingReadiness;
using dog_patrol_perception_tracking::PerceptionReadiness;

TEST(DetectionTrackingReadinessTest, ReportsInitializationProgressFromRealRuntimeState) {
  DetectionTrackingReadiness readiness;
  auto contribution = readiness.Contribution();
  EXPECT_EQ(contribution.capability, "detection_tracking");
  EXPECT_EQ(contribution.readiness, PerceptionReadiness::kNotReady);
  EXPECT_EQ(contribution.detail, "detector initialization pending");

  readiness.ReportRuntimeStatus({true, false, {}});
  contribution = readiness.Contribution();
  EXPECT_EQ(contribution.readiness, PerceptionReadiness::kNotReady);
  EXPECT_EQ(contribution.detail, "tracker initialization pending");

  readiness.ReportRuntimeStatus({true, true, {}});
  contribution = readiness.Contribution();
  EXPECT_EQ(contribution.readiness, PerceptionReadiness::kReady);
  EXPECT_EQ(contribution.detail, "detector and tracker runtime ready");
}

TEST(DetectionTrackingReadinessTest, RuntimeFailureOverridesInitializationFlags) {
  DetectionTrackingReadiness readiness;
  readiness.ReportRuntimeStatus({true, true, "tracker no longer accepts frames"});
  const auto contribution = readiness.Contribution();
  EXPECT_EQ(contribution.readiness, PerceptionReadiness::kFailure);
  EXPECT_EQ(contribution.detail, "tracker no longer accepts frames");
}

}  // namespace
