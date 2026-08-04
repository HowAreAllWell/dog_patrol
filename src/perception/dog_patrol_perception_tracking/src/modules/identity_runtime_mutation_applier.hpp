#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "identity_assignment_engine_adapter.hpp"
#include "phase4_direct_apply_helper.hpp"

namespace dog_patrol_perception_tracking {

class IdentityRuntimeMutationApplier {
 public:
  IdentityRuntimeMutationApplier(IdentityAssignmentEngineAdapter::Config config,
                                 IdentityAssignmentEngineAdapter::RuntimeState *runtime_state,
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
  using RuntimeState = IdentityAssignmentEngineAdapter::RuntimeState;
  using ScoreDebugRow = IdentityAssignmentEngineAdapter::ScoreDebugRow;

  std::vector<float> ExtractFeature(const cv::Mat &frame, const Track &track) const;
  ReliableGeometryCost::State ReliableGeometryState(const IdentityRuntimeRecord &identity) const;
  bool TrackOverlapsAny(const Track &track, const std::vector<Track> &tracks, int self_idx) const;
  bool IsReliableObservation(const Track &track, bool allow_feat_update,
                             float assignment_cost, float assignment_margin) const;
  FeatureUpdatePolicy::Decision EvaluateUpdatePolicy(const Track &track,
                                                     const std::vector<Track> &tracks,
                                                     int self_idx,
                                                     const IdentityRuntimeRecord *identity,
                                                     bool accepted,
                                                     float assignment_cost,
                                                     float assignment_margin,
                                                     bool force_geometry_update = false) const;
  void UpdateIdentityObservation(IdentityRuntimeRecord *identity, const Track &track,
                                 float assignment_cost, float assignment_margin) const;
  void UpsertIdentity(const Track &track, int semantic_id, const std::vector<float> &feature,
                      const FeatureUpdatePolicy::Decision &update_policy,
                      float assignment_cost, float assignment_margin);
  void ComputeCosts(const Track &track, const IdentityRuntimeRecord &identity,
                    const std::vector<float> &feature, float *app, float *geo,
                    float *tim, float *final) const;
  int AllocateNewSemanticId();
  bool ApplyPhase4DirectActions(const std::vector<Track> &tracks,
                                const std::vector<Phase4DirectApplyHelper::Action> &actions,
                                const cv::Mat *frame);

  IdentityAssignmentEngineAdapter::Config config_;
  RuntimeState *runtime_state_{nullptr};
  AppearanceFeatureService *appearance_features_{nullptr};
};

}  // namespace dog_patrol_perception_tracking
