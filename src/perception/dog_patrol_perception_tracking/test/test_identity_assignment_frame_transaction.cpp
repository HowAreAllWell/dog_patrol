#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "identity_assignment_engine_adapter.hpp"
#include "identity_assignment_frame_transaction.hpp"

namespace {

dog_patrol_perception_tracking::Track MakePersonTrack(const int raw_id, const cv::Rect2f &bbox,
                                        const std::vector<float> &feature = {1.0F, 0.0F, 0.0F}) {
  dog_patrol_perception_tracking::Track track;
  track.id = raw_id;
  track.class_id = dog_patrol_perception_tracking::ClassId::kPerson;
  track.confidence = 0.9F;
  track.bbox = bbox;
  track.is_confirmed = true;
  track.appearance_feature = feature;
  track.association.stage = "stage1_confirmed_high";
  track.association.passed_final_cost_gate = true;
  return track;
}
dog_patrol_perception_tracking::PrimaryTargetResult IdlePrimary() {
  dog_patrol_perception_tracking::PrimaryTargetResult primary;
  primary.state = dog_patrol_perception_tracking::PrimaryState::kIdle;
  return primary;
}

const dog_patrol_perception_tracking::IdentityAssignmentEngineAdapter::ScoreDebugRow *FindDebugRow(
    const std::vector<dog_patrol_perception_tracking::IdentityAssignmentEngineAdapter::ScoreDebugRow> &rows,
    const int raw_id, const std::string &stage) {
  const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto &row) {
    return row.raw_track_id == raw_id && row.stage == stage;
  });
  return it == rows.end() ? nullptr : &(*it);
}

}  // namespace

TEST(IdentityAssignmentFrameTransactionTest, KeepsContinuityDebugAndAgesAtFrameEnd) {
  dog_patrol_perception_tracking::IdentityAssignmentEngineAdapter::Config config;
  config.raw_continuity_max_cost = 0.10F;
  config.active_assign_max_cost = 0.90F;
  config.max_missing_frames = 1;
  dog_patrol_perception_tracking::IdentityAssignmentEngineAdapter::RuntimeState runtime_state;
  dog_patrol_perception_tracking::IdentityAssignmentEngineAdapter adapter(config, &runtime_state);

  ASSERT_EQ(adapter.Update({MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50))}, IdlePrimary()).at(7), 1);
  const auto second = adapter.Update({MakePersonTrack(7, cv::Rect2f(500, 0, 50, 50))}, IdlePrimary());

  ASSERT_EQ(second.at(7), 1);
  const auto *continuity_row = FindDebugRow(adapter.LastScoreDebugRows(), 7, "raw_continuity");
  ASSERT_NE(continuity_row, nullptr);
  EXPECT_TRUE(continuity_row->continuity_used);
  EXPECT_FALSE(continuity_row->accepted);
  EXPECT_EQ(continuity_row->reject_reason, "raw_continuity_max_cost_reject");

  adapter.Update({}, IdlePrimary());
  ASSERT_EQ(adapter.IdentitySnapshots().size(), 1U);
  EXPECT_EQ(adapter.IdentitySnapshots().front().missing_frames, 1);
}

TEST(IdentityAssignmentFrameTransactionTest, BackfillsActiveSelectionIntoDebugStream) {
  dog_patrol_perception_tracking::IdentityAssignmentEngineAdapter::Config config;
  config.min_assignment_margin = 0.0F;
  config.active_assign_max_cost = 0.90F;
  dog_patrol_perception_tracking::IdentityAssignmentEngineAdapter::RuntimeState runtime_state;
  dog_patrol_perception_tracking::IdentityAssignmentEngineAdapter adapter(config, &runtime_state);

  ASSERT_EQ(adapter.Update({MakePersonTrack(7, cv::Rect2f(0, 0, 100, 300))}, IdlePrimary()).at(7), 1);
  const auto second = adapter.Update({MakePersonTrack(8, cv::Rect2f(0, 0, 100, 300))}, IdlePrimary());

  ASSERT_EQ(second.at(8), 1);
  const auto *assignment_row = FindDebugRow(adapter.LastScoreDebugRows(), 8, "assign_candidate");
  ASSERT_NE(assignment_row, nullptr);
  EXPECT_TRUE(assignment_row->selected);
  EXPECT_TRUE(assignment_row->accepted);
}
