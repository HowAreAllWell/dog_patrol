#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "identity_assignment_engine_adapter.hpp"
#include "identity_assignment_frame_transaction.hpp"

namespace {

vision_demo_host::Track MakePersonTrack(const int raw_id, const cv::Rect2f &bbox,
                                        const std::vector<float> &feature = {1.0F, 0.0F, 0.0F}) {
  vision_demo_host::Track track;
  track.id = raw_id;
  track.class_id = vision_demo_host::ClassId::kPerson;
  track.confidence = 0.9F;
  track.bbox = bbox;
  track.is_confirmed = true;
  track.appearance_feature = feature;
  track.association.stage = "stage1_confirmed_high";
  track.association.passed_final_cost_gate = true;
  return track;
}
vision_demo_host::PrimaryTargetResult IdlePrimary() {
  vision_demo_host::PrimaryTargetResult primary;
  primary.state = vision_demo_host::PrimaryState::kIdle;
  return primary;
}

const vision_demo_host::IdentityAssignmentEngineAdapter::ScoreDebugRow *FindDebugRow(
    const std::vector<vision_demo_host::IdentityAssignmentEngineAdapter::ScoreDebugRow> &rows,
    const int raw_id, const std::string &stage) {
  const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto &row) {
    return row.raw_track_id == raw_id && row.stage == stage;
  });
  return it == rows.end() ? nullptr : &(*it);
}

}  // namespace

TEST(IdentityAssignmentFrameTransactionTest, KeepsContinuityDebugAndAgesAtFrameEnd) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config config;
  config.raw_continuity_max_cost = 0.10F;
  config.active_assign_max_cost = 0.90F;
  config.max_missing_frames = 1;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter adapter(config, &runtime_state);

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
  vision_demo_host::IdentityAssignmentEngineAdapter::Config config;
  config.min_assignment_margin = 0.0F;
  config.active_assign_max_cost = 0.90F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter adapter(config, &runtime_state);

  ASSERT_EQ(adapter.Update({MakePersonTrack(7, cv::Rect2f(0, 0, 100, 300))}, IdlePrimary()).at(7), 1);
  const auto second = adapter.Update({MakePersonTrack(8, cv::Rect2f(0, 0, 100, 300))}, IdlePrimary());

  ASSERT_EQ(second.at(8), 1);
  const auto *assignment_row = FindDebugRow(adapter.LastScoreDebugRows(), 8, "assign_candidate");
  ASSERT_NE(assignment_row, nullptr);
  EXPECT_TRUE(assignment_row->selected);
  EXPECT_TRUE(assignment_row->accepted);
}
