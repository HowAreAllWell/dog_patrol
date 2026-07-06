#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "assignment_application_plan.hpp"
#include "identity_runtime_record.hpp"
#include "identity_runtime_store.hpp"
#include "raw_semantic_binding_store.hpp"
#include "vision_demo_host/modules/feature_update_policy.hpp"
#include "vision_demo_host/types.hpp"

namespace vision_demo_host {

template <typename DebugRowT>
class AssignmentApplicationExecutor {
 public:
  struct Config {
    int frame_idx{-1};
    int occlusion_protect_frames{0};
    bool protect_unseen_active_people{false};
  };

  using EvaluateUpdatePolicyFn = std::function<FeatureUpdatePolicy::Decision(
      const Track &, int, int, const IdentityRuntimeRecord *, float, float, bool)>;
  using UpsertIdentityFn = std::function<void(const Track &, int, const std::vector<float> &,
                                              const FeatureUpdatePolicy::Decision &, float, float)>;

  static void Execute(const AssignmentApplicationPlan::Result &plan,
                      const std::vector<int> &planned_track_indices,
                      const std::vector<Track> &tracks,
                      const std::unordered_map<int, std::size_t> &feature_index_by_track_idx,
                      const std::vector<std::vector<float>> &person_features,
                      std::vector<DebugRowT> *debug_rows,
                      IdentityRuntimeStore *identity_store,
                      RawSemanticBindingStore *raw_semantic_bindings,
                      const Config &config,
                      const EvaluateUpdatePolicyFn &evaluate_update_policy,
                      const UpsertIdentityFn &upsert_identity) {
    std::unordered_map<int, AssignmentApplicationPlan::Application> application_by_track_idx;
    application_by_track_idx.reserve(plan.applications.size());
    for (const auto &application : plan.applications) {
      application_by_track_idx[application.track_idx] = application;
    }

    for (const int planned_track_idx : planned_track_indices) {
      const auto application_it = application_by_track_idx.find(planned_track_idx);
      if (application_it == application_by_track_idx.end()) {
        continue;
      }

      const auto &application = application_it->second;
      if (application.track_idx < 0 ||
          application.track_idx >= static_cast<int>(tracks.size())) {
        continue;
      }

      const auto feature_it = feature_index_by_track_idx.find(application.track_idx);
      if (feature_it == feature_index_by_track_idx.end() ||
          feature_it->second >= person_features.size()) {
        continue;
      }

      const Track &track = tracks[static_cast<std::size_t>(application.track_idx)];
      const auto *identity =
          identity_store == nullptr ? nullptr : identity_store->Find(application.semantic_id);
      const bool side_recovery_geometry_update =
          application.accepted_stage == "merged_side_recovery";
      const auto update_policy =
          evaluate_update_policy(track, application.track_idx, application.semantic_id, identity,
                                 application.assignment_cost, application.assignment_margin,
                                 side_recovery_geometry_update);

      if (debug_rows != nullptr) {
        for (auto &row : *debug_rows) {
          if (row.frame_idx == config.frame_idx &&
              row.track_idx == application.track_idx &&
              row.semantic_id == application.semantic_id &&
              row.accepted) {
            row.feature_update_allowed = update_policy.feature_update_allowed;
            row.geometry_update_allowed = update_policy.geometry_update_allowed;
            row.feature_update_reason = update_policy.feature_update_reason;
            row.geometry_update_reason = update_policy.geometry_update_reason;
          }
        }
      }

      upsert_identity(track, application.semantic_id, person_features[feature_it->second],
                      update_policy, application.assignment_cost, application.assignment_margin);
    }

    if (config.protect_unseen_active_people && identity_store != nullptr) {
      identity_store->ProtectUnseenActivePeople(config.occlusion_protect_frames);
    }

    if (debug_rows != nullptr) {
      for (auto &row : *debug_rows) {
        if (row.feature_update_reason.empty()) {
          row.feature_update_reason =
              row.accepted
                  ? (row.feature_update_allowed ? "allowed_update"
                                                : "unreliable_low_quality_observation")
                  : "update_blocked_by_rejected_assignment";
        }
        if (row.geometry_update_reason.empty()) {
          row.geometry_update_reason =
              row.accepted
                  ? (row.geometry_update_allowed ? "allowed_update"
                                                 : "unreliable_low_quality_observation")
                  : "update_blocked_by_rejected_assignment";
        }
      }
    }

    if (raw_semantic_bindings != nullptr) {
      raw_semantic_bindings->ReplaceFromPlannedEntries(plan.next_raw_to_semantic_entries);
    }
  }
};

}  // namespace vision_demo_host
