#include "occlusion_mode_state.hpp"

#include <algorithm>

namespace vision_demo_host {

OcclusionModeState::State OcclusionModeState::Advance(const Config &config, const State &previous, const Input &input) {
  State next = previous;

  const bool merge_evidence =
      !config.merged_requires_overlap || input.has_overlap || previous.prev_had_overlap;
  if (input.visible_person_count == 1 && merge_evidence &&
      (previous.prev_visible_person_count >= 2 || previous.mode == LegacyIdentityMode::kMerged ||
       previous.mode == LegacyIdentityMode::kSplitRecovery)) {
    next.mode = LegacyIdentityMode::kMerged;
    next.merged_frames += 1;
    next.split_stable_count = 0;
  } else if (previous.mode == LegacyIdentityMode::kMerged && input.visible_person_count >= 2 &&
             previous.merged_frames >= std::max(1, config.merge_hold_frames)) {
    next.mode = LegacyIdentityMode::kSplitRecovery;
    next.split_stable_count = 0;
  } else if (previous.mode == LegacyIdentityMode::kSplitRecovery) {
    if (input.visible_person_count >= 2 && !input.has_overlap) {
      next.split_stable_count += 1;
      if (next.split_stable_count >= std::max(1, config.split_stable_frames)) {
        next.mode = LegacyIdentityMode::kNormalResumed;
      }
    } else {
      next.split_stable_count = 0;
    }
  } else if (input.visible_person_count >= 2 && !input.has_overlap) {
    next.mode = LegacyIdentityMode::kNormal;
    next.merged_frames = 0;
    next.split_stable_count = 0;
  }

  if (next.mode == LegacyIdentityMode::kNormalResumed) {
    next.mode = LegacyIdentityMode::kNormal;
    next.merged_frames = 0;
    next.split_stable_count = 0;
  }

  next.feature_update_frozen =
      (next.mode == LegacyIdentityMode::kMerged || next.mode == LegacyIdentityMode::kSplitRecovery ||
       input.has_overlap);
  next.prev_visible_person_count = input.visible_person_count;
  next.prev_had_overlap = input.has_overlap;
  return next;
}

}  // namespace vision_demo_host
