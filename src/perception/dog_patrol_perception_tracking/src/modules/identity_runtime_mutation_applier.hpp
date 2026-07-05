#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "legacy_identity_matcher.hpp"
#include "phase4_direct_apply_helper.hpp"

namespace vision_demo_host {

class IdentityRuntimeMutationApplier {
 public:
  IdentityRuntimeMutationApplier(LegacyIdentityMatcher::Config config,
                                 LegacyIdentityMatcher::RuntimeState *runtime_state,
                                 AppearanceFeatureService *appearance_features);

  bool ApplyPhase4MergedSplitHandoff(const std::vector<Track> &tracks,
                                     int continuity_raw_track_id,
                                     int continuity_semantic_id,
                                     int candidate_raw_track_id,
                                     int candidate_semantic_id,
                                     const cv::Mat *frame = nullptr);
  bool ApplyPhase4MergedSideRecovery(const std::vector<Track> &tracks,
                                     int carrier_raw_track_id,
                                     int carrier_semantic_id,
                                     int candidate_raw_track_id,
                                     int candidate_semantic_id,
                                     const cv::Mat *frame = nullptr);
  bool ApplyPhase4MergedSingleBlobHandoff(const std::vector<Track> &tracks,
                                          int carrier_raw_track_id,
                                          int carrier_semantic_id,
                                          int candidate_semantic_id,
                                          const cv::Mat *frame = nullptr);
  bool ApplyPhase4PairwiseAssignment(const std::vector<Track> &tracks,
                                     int first_raw_track_id,
                                     int first_semantic_id,
                                     int second_raw_track_id,
                                     int second_semantic_id,
                                     const cv::Mat *frame = nullptr);
  bool ApplyPhase5BirthAllocation(const std::vector<Track> &tracks,
                                  int raw_track_id,
                                  const cv::Mat *frame = nullptr);

 private:
  using RuntimeState = LegacyIdentityMatcher::RuntimeState;
  using ScoreDebugRow = LegacyIdentityMatcher::ScoreDebugRow;

  std::vector<float> ExtractFeature(const cv::Mat &frame, const Track &track) const;
  ReliableGeometryCost::State ReliableGeometryState(const LegacyIdentityRecord &identity) const;
  bool TrackOverlapsAny(const Track &track, const std::vector<Track> &tracks, int self_idx) const;
  bool IsReliableObservation(const Track &track, bool allow_feat_update,
                             float assignment_cost, float assignment_margin) const;
  FeatureUpdatePolicy::Decision EvaluateUpdatePolicy(const Track &track,
                                                     const std::vector<Track> &tracks,
                                                     int self_idx,
                                                     const LegacyIdentityRecord *identity,
                                                     bool accepted,
                                                     float assignment_cost,
                                                     float assignment_margin,
                                                     bool force_geometry_update = false) const;
  void UpdateIdentityObservation(LegacyIdentityRecord *identity, const Track &track,
                                 float assignment_cost, float assignment_margin) const;
  void UpsertIdentity(const Track &track, int semantic_id, const std::vector<float> &feature,
                      const FeatureUpdatePolicy::Decision &update_policy,
                      float assignment_cost, float assignment_margin);
  void ComputeCosts(const Track &track, const LegacyIdentityRecord &identity,
                    const std::vector<float> &feature, float *app, float *geo,
                    float *tim, float *final) const;
  int AllocateNewSemanticId();
  bool ApplyPhase4DirectActions(const std::vector<Track> &tracks,
                                const std::vector<Phase4DirectApplyHelper::Action> &actions,
                                const cv::Mat *frame);

  LegacyIdentityMatcher::Config config_;
  RuntimeState *runtime_state_{nullptr};
  AppearanceFeatureService *appearance_features_{nullptr};
};

}  // namespace vision_demo_host
