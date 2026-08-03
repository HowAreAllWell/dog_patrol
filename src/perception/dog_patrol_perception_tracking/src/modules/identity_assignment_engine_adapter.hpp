#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "vision_demo_host/modules/appearance_feature_service.hpp"
#include "vision_demo_host/modules/feature_geometry_update_state.hpp"
#include "vision_demo_host/modules/feature_update_policy.hpp"
#include "vision_demo_host/modules/reliable_geometry_cost.hpp"
#include "vision_demo_host/types.hpp"
#include "assignment_candidate_builder.hpp"
#include "birth_manager.hpp"
#include "birth_candidate_store.hpp"
#include "identity_runtime_record.hpp"
#include "identity_lifecycle_mode.hpp"
#include "identity_runtime_snapshot.hpp"
#include "identity_runtime_store.hpp"
#include "occlusion_mode_state.hpp"
#include "raw_semantic_binding_store.hpp"
#include "semantic_id_allocator.hpp"

namespace vision_demo_host {

class IdentityAssignmentFrameTransaction;

// Internal assignment/update adapter. IdentityManager owns the public identity
// boundary and runtime state; this class adapts tracks into the current
// assignment, recovery, and debug-update engine.
class IdentityAssignmentEngineAdapter {
 public:
  struct ScoreDebugRow {
    int frame_idx{-1};
    IdentityLifecycleMode mode{IdentityLifecycleMode::kNormal};
    int track_idx{-1};
    int raw_track_id{-1};
    int semantic_id{-1};
    float app_cost{0.0F};
    float geo_cost{0.0F};
    float time_cost{0.0F};
    float final_score{0.0F};
    bool selected{false};
    std::string stage;
    float margin{0.0F};
    bool accepted{false};
    std::string reject_reason;
    bool continuity_used{false};
    bool feature_update_allowed{false};
    bool geometry_update_allowed{false};
    std::string feature_update_reason;
    std::string geometry_update_reason;
  };

  using IdentitySnapshot = IdentityRuntimeSnapshot;

  struct PairwiseAssignmentDebugRow {
    int frame_idx{-1};
    IdentityLifecycleMode mode{IdentityLifecycleMode::kNormal};
    std::string selected_pairs;
    std::string alternate_pairs;
    float selected_final_cost{0.0F};
    float alternate_final_cost{0.0F};
    float selected_app_cost{0.0F};
    float alternate_app_cost{0.0F};
    float margin{0.0F};
    bool appearance_override{false};
  };

  struct Config {
    int max_missing_frames{180};
    int feat_bank_size{30};
    float recover_sim_thresh_strict{0.85F};
    float recover_sim_thresh_relaxed{0.75F};
    int recover_relaxed_max_missing_frames{180};
    int occlusion_protect_frames{30};
    float missing_assign_min_area_ratio{0.40F};
    float missing_assign_max_area_ratio{4.00F};
    float missing_assign_max_center_dist_norm{2.50F};
    float missing_assign_max_app_cost{0.50F};
    float overlap_iou_freeze{0.10F};
    int split_stable_frames{3};
    int merge_hold_frames{2};
    float app_w{0.70F};
    float geo_w{0.20F};
    float time_w{0.10F};
    float active_assign_max_cost{0.55F};
    float recovery_max_cost{0.45F};
    float raw_continuity_max_cost{0.55F};
    float min_assignment_margin{0.08F};
    int stable_frames_before_feature_update{3};
    bool merged_requires_overlap{true};
    bool auto_apply_phase5_birth_allocations{true};
    bool reid_enable{true};
    std::string reid_backend{"light"};
    std::string reid_model_path{};
    int reid_input_width{128};
    int reid_input_height{256};
  };

  struct RuntimeState {
    SemanticIdAllocator semantic_id_allocator;
    bool primary_initialized{false};
    int current_primary_semantic_id{-1};
    int frame_index{0};
    OcclusionModeState::State occlusion_mode{};
    std::vector<ScoreDebugRow> last_score_debug_rows;
    std::vector<PairwiseAssignmentDebugRow> last_pairwise_assignment_debug_rows;
    IdentityRuntimeStore identity_store;
    RawSemanticBindingStore raw_semantic_bindings;
    BirthManager birth_manager;
  };

  IdentityAssignmentEngineAdapter(Config config, RuntimeState *runtime_state);
  bool Initialize(std::string *error);

  static void ResetRuntimeState(RuntimeState *runtime_state);
  void ResetAdapter();
  void Reset();
  const std::unordered_map<int, int> &Update(const std::vector<Track> &tracks, const PrimaryTargetResult &primary,
                                             const cv::Mat *frame = nullptr);

  int SemanticIdForRawTrack(int raw_track_id) const;
  int CurrentPrimarySemanticId() const { return runtime_state_->current_primary_semantic_id; }
  IdentityLifecycleMode CurrentMode() const { return runtime_state_->occlusion_mode.mode; }
  bool IsFeatureUpdateFrozen() const { return runtime_state_->occlusion_mode.feature_update_frozen; }
  const std::vector<ScoreDebugRow> &LastScoreDebugRows() const { return runtime_state_->last_score_debug_rows; }
  const std::vector<PairwiseAssignmentDebugRow> &LastPairwiseAssignmentDebugRows() const {
    return runtime_state_->last_pairwise_assignment_debug_rows;
  }
  std::vector<IdentitySnapshot> IdentitySnapshots() const;

 private:
  friend class IdentityAssignmentFrameTransaction;

  struct Assignment {
    int track_idx{-1};
    int semantic_id{-1};
    float score{-1.0F};
    float cost{1.0F};
    float margin{0.0F};
    bool accepted{false};
    std::string reject_reason;
    bool pairwise_appearance_override{false};
  };

  std::vector<float> ExtractFeature(const cv::Mat &frame, const Track &track) const;
  ReliableGeometryCost::State ReliableGeometryState(const IdentityRuntimeRecord &identity) const;
  bool PassesMissingIdentityGate(const Track &track, const IdentityRuntimeRecord &identity, float app_cost, float geo_cost) const;
  bool PassesMissingAppearanceGate(const IdentityRuntimeRecord &identity, float app_cost, float geo_cost) const;
  void ComputeCosts(const Track &track, const IdentityRuntimeRecord &identity, const std::vector<float> &feature, float *app,
                    float *geo, float *tim, float *final) const;
  float AssignmentScore(const Track &track, const IdentityRuntimeRecord &identity, const std::vector<float> &feature) const;
  float ActiveAssignmentMaxCost(const IdentityRuntimeRecord &identity, const AssociationEvidence &association) const;

  void UpsertIdentity(const Track &track, int semantic_id, const std::vector<float> &feature,
                      const FeatureUpdatePolicy::Decision &update_policy, float assignment_cost,
                      float assignment_margin);
  bool IsReliableObservation(const Track &track, bool allow_feat_update, float assignment_cost, float assignment_margin) const;
  FeatureUpdatePolicy::Decision EvaluateUpdatePolicy(const Track &track, const std::vector<Track> &tracks,
                                                     int self_idx, const IdentityRuntimeRecord *identity, bool accepted,
                                                     float assignment_cost, float assignment_margin,
                                                     bool force_geometry_update = false) const;
  void UpdateIdentityObservation(IdentityRuntimeRecord *identity, const Track &track, float assignment_cost, float assignment_margin) const;
  std::vector<Assignment> SolveAssignments(const AssignmentCandidateBuilder::ActiveBuildResult &build_result,
                                           const std::vector<Track> &tracks);
  bool TrackOverlapsAny(const Track &track, const std::vector<Track> &tracks, int self_idx) const;
  bool LooksLikeMergedSideReappearance(const Track &candidate, const IdentityRuntimeRecord &identity,
                                       const std::vector<Track> &tracks, int candidate_idx,
                                       const std::unordered_map<int, int> &track_idx_to_sid,
                                       float app_cost) const;
  int AllocateNewSemanticId();
  int SelectBestSemanticForMerged(const std::vector<int> &candidate_semantic_ids, const Track &track,
                                  const std::vector<float> &feature) const;
  float RecoverThresholdForSemantic(const IdentityRuntimeRecord &identity) const;
  bool CanRecoverInactiveIdentity(const IdentityRuntimeRecord &identity) const;

  Config config_;
  RuntimeState *runtime_state_{nullptr};
  AppearanceFeatureService appearance_features_;
  bool initialized_{false};
};

}  // namespace vision_demo_host
