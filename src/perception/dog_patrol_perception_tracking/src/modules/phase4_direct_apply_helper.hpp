#pragma once

#include <algorithm>
#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "vision_demo_host/modules/feature_update_policy.hpp"
#include "legacy_identity_mode.hpp"
#include "legacy_identity_record.hpp"
#include "vision_demo_host/types.hpp"

namespace vision_demo_host {

class Phase4DirectApplyHelper {
 public:
  struct Action {
    int raw_track_id{-1};
    int semantic_id{-1};
    std::string stage;
    bool force_geometry_update{false};
  };

  template <typename ScoreDebugRowT>
  struct Input {
    const std::vector<Track> *tracks{nullptr};
    const cv::Mat *frame{nullptr};
    int frame_index{-1};
    LegacyIdentityMode mode{LegacyIdentityMode::kNormal};
    std::vector<Action> actions;
    std::vector<ScoreDebugRowT> *debug_rows{nullptr};
    std::function<bool(int)> contains_semantic_id;
    std::function<const LegacyIdentityRecord *(int)> find_identity;
    std::function<std::vector<float>(const cv::Mat &, const Track &)> extract_feature;
    std::function<void(const Track &, const LegacyIdentityRecord &, const std::vector<float> &,
                       float *, float *, float *, float *)>
        compute_costs;
    std::function<FeatureUpdatePolicy::Decision(const Track &, int, const LegacyIdentityRecord *,
                                                float, float, bool)>
        evaluate_update_policy;
    std::function<void(const Track &, int, const std::vector<float> &,
                       const FeatureUpdatePolicy::Decision &, float, float)>
        upsert_identity;
    std::function<void(int, int)> bind_raw_semantic;
    std::function<void(int)> erase_birth_candidate;
  };

  template <typename ScoreDebugRowT>
  static bool Apply(const Input<ScoreDebugRowT> &input) {
    if (input.tracks == nullptr || input.debug_rows == nullptr || input.actions.empty() ||
        !input.contains_semantic_id || !input.find_identity || !input.extract_feature ||
        !input.compute_costs || !input.evaluate_update_policy || !input.upsert_identity ||
        !input.bind_raw_semantic || !input.erase_birth_candidate) {
      return false;
    }

    std::unordered_set<int> raw_track_ids;
    std::unordered_set<int> semantic_ids;
    std::vector<int> track_indices;
    track_indices.reserve(input.actions.size());
    for (const auto &action : input.actions) {
      if (action.raw_track_id <= 0 || action.semantic_id <= 0 || action.stage.empty() ||
          !raw_track_ids.insert(action.raw_track_id).second ||
          !semantic_ids.insert(action.semantic_id).second) {
        return false;
      }

      const int track_idx = FindVisiblePersonTrack(*input.tracks, action.raw_track_id);
      if (track_idx < 0 || !input.contains_semantic_id(action.semantic_id)) {
        return false;
      }
      track_indices.push_back(track_idx);
    }

    const cv::Mat empty_frame;
    const cv::Mat *feature_frame =
        (input.frame != nullptr && !input.frame->empty()) ? input.frame : &empty_frame;

    std::vector<std::vector<float>> features;
    features.reserve(input.actions.size());
    for (const int track_idx : track_indices) {
      features.push_back(input.extract_feature(*feature_frame,
                                               (*input.tracks)[static_cast<std::size_t>(track_idx)]));
    }

    for (std::size_t i = 0; i < input.actions.size(); ++i) {
      const auto &action = input.actions[i];
      const int track_idx = track_indices[i];
      const Track &track = (*input.tracks)[static_cast<std::size_t>(track_idx)];
      const auto *identity = input.find_identity(action.semantic_id);

      float app = 0.0F;
      float geo = 0.0F;
      float tim = 0.0F;
      float final = 0.0F;
      if (identity != nullptr) {
        input.compute_costs(track, *identity, features[i], &app, &geo, &tim, &final);
      }
      const float margin = std::max(0.0F, 1.0F - std::clamp(final, 0.0F, 1.0F));
      const auto update_policy =
          input.evaluate_update_policy(track, track_idx, identity, final, margin,
                                       action.force_geometry_update);

      ScoreDebugRowT row;
      row.frame_idx = input.frame_index;
      row.mode = input.mode;
      row.track_idx = track_idx;
      row.raw_track_id = track.id;
      row.semantic_id = action.semantic_id;
      row.app_cost = app;
      row.geo_cost = geo;
      row.time_cost = tim;
      row.final_score = final;
      row.margin = margin;
      row.selected = true;
      row.accepted = true;
      row.stage = action.stage;
      row.feature_update_allowed = update_policy.feature_update_allowed;
      row.geometry_update_allowed = update_policy.geometry_update_allowed;
      row.feature_update_reason = update_policy.feature_update_reason;
      row.geometry_update_reason = update_policy.geometry_update_reason;
      input.debug_rows->push_back(row);

      input.upsert_identity(track, action.semantic_id, features[i], update_policy, final, margin);
      input.bind_raw_semantic(track.id, action.semantic_id);
      input.erase_birth_candidate(track.id);
    }
    return true;
  }

 private:
  static int FindVisiblePersonTrack(const std::vector<Track> &tracks, int raw_track_id) {
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      if (tracks[i].class_id == ClassId::kPerson && tracks[i].id == raw_track_id) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }
};

}  // namespace vision_demo_host
