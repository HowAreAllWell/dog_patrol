#pragma once

#include <functional>
#include <string>
#include <vector>

#include "birth_candidate_decision.hpp"
#include "birth_candidate_store.hpp"

namespace dog_patrol_perception_tracking {

class BirthManager {
 public:
  struct DebugRow {
    int track_idx{-1};
    int raw_track_id{-1};
    int semantic_id{-1};
    float final_score{0.0F};
    bool selected{false};
    std::string stage;
    float margin{0.0F};
    bool accepted{false};
    std::string reject_reason;
  };

  struct Input {
    int frame_index{-1};
    int track_idx{-1};
    int raw_track_id{-1};
    bool hold_for_ambiguous_recovery{false};
    bool duplicate_split{false};
    std::string hide_reason;
    bool small_person_requires_stability{false};
  };

  struct Result {
    BirthCandidateDecision::Decision decision;
    int stable_observation_count{0};
    bool has_debug_row{false};
    DebugRow debug_row;
    bool allocated_semantic_id{false};
    int semantic_id{-1};
  };

  using AllocateSemanticIdFn = std::function<int()>;

  explicit BirthManager(BirthCandidateDecision::Config config = {});

  void Reset();
  void Erase(int raw_track_id);

  Result Evaluate(const Input &input, const AllocateSemanticIdFn &allocate_semantic_id);

  template <typename DebugRowT>
  bool ApplyPhase5AcceptedAllocation(const int raw_track_id,
                                     std::vector<DebugRowT> *debug_rows,
                                     const AllocateSemanticIdFn &allocate_semantic_id,
                                     DebugRow *accepted_row = nullptr,
                                     int *semantic_id = nullptr) {
    if (raw_track_id <= 0 || debug_rows == nullptr) {
      return false;
    }

    for (auto &row : *debug_rows) {
      if (row.raw_track_id != raw_track_id ||
          row.stage != "phase5_birth_candidate" ||
          !row.selected ||
          row.accepted ||
          row.reject_reason != "phase5_birth_manager_pending") {
        continue;
      }

      const int new_semantic_id = allocate_semantic_id();
      row.semantic_id = new_semantic_id;
      row.stage = "phase5_new_semantic";
      row.reject_reason.clear();
      row.selected = true;
      row.accepted = true;
      row.final_score = 0.0F;
      row.margin = 1.0F;
      Erase(raw_track_id);

      if (accepted_row != nullptr) {
        accepted_row->track_idx = row.track_idx;
        accepted_row->raw_track_id = row.raw_track_id;
        accepted_row->semantic_id = row.semantic_id;
        accepted_row->final_score = row.final_score;
        accepted_row->selected = row.selected;
        accepted_row->stage = row.stage;
        accepted_row->margin = row.margin;
        accepted_row->accepted = row.accepted;
        accepted_row->reject_reason = row.reject_reason;
      }
      if (semantic_id != nullptr) {
        *semantic_id = new_semantic_id;
      }
      return true;
    }

    return false;
  }

 private:
  static DebugRow MakeDebugRow(const BirthCandidateDecision::Decision &decision, int semantic_id);

  BirthCandidateDecision::Config config_;
  BirthCandidateStore pending_candidates_;
};

}  // namespace dog_patrol_perception_tracking
