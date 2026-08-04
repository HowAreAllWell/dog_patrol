#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

class IdentityManager {
 public:
  enum class Mode {
    kNormal,
    kMerged,
    kSplitRecovery,
    kNormalResumed,
  };

  struct ScoreDebugRow {
    int frame_idx{-1};
    Mode mode{Mode::kNormal};
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

  struct Phase3ShadowDebugRow {
    int frame_idx{-1};
    int event_idx{-1};
    std::string event_type;
    int group_id{-1};
    std::string semantic_ids;
    int carrier_semantic_id{-1};
    int carrier_raw_track_id{-1};
    int candidate_raw_track_id{-1};
    int candidate_semantic_id{-1};
    cv::Rect2f candidate_bbox;
    float candidate_confidence{0.0F};
    std::string reason;
    int related_raw_track_id{-1};
    std::string hypothesis_status;
    int candidate_stable_frames{0};
    int group_age_frames{0};
    int group_last_update_frame{-1};
    float decision_app_cost{0.0F};
    float decision_geo_cost{0.0F};
    float decision_time_cost{0.0F};
    float decision_final_score{0.0F};
    float decision_margin{0.0F};
    bool decision_selected{false};
    bool decision_accepted{false};
    std::string pairwise_selected_pairs;
    std::string pairwise_alternate_pairs;
    float pairwise_selected_final_cost{0.0F};
    float pairwise_alternate_final_cost{0.0F};
    float pairwise_selected_app_cost{0.0F};
    float pairwise_alternate_app_cost{0.0F};
    float pairwise_margin{0.0F};
    bool pairwise_appearance_override{false};
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
    bool reid_enable{true};
    std::string reid_backend{"light"};
    std::string reid_model_path{};
    int reid_input_width{128};
    int reid_input_height{256};
  };

  IdentityManager();
  explicit IdentityManager(Config config);

  bool Initialize(std::string *error);
  void Reset();

  IdentityManagerResult Update(const std::vector<TrackletObservation> &observations,
                               const PrimaryTargetResult &primary,
                               const cv::Mat *frame = nullptr);
  IdentityManagerResult Update(const std::vector<TrackletObservation> &observations,
                               const std::vector<TrackletHypothesis> &shadow_hypotheses,
                               const PrimaryTargetResult &primary,
                               const cv::Mat *frame = nullptr);

  Mode CurrentMode() const;
  bool IsFeatureUpdateFrozen() const;
  const std::vector<ScoreDebugRow> &LastScoreDebugRows() const;
  const std::vector<Phase3ShadowDebugRow> &LastPhase3ShadowDebugRows() const;

 private:
  static std::vector<Track> TracksFromObservations(const std::vector<TrackletObservation> &observations);
  static IdentityAssignmentEvidence AssignmentEvidenceFromDebug(const ScoreDebugRow &row);

  class Impl;
  std::shared_ptr<Impl> impl_;
  std::unordered_map<int, int> raw_to_semantic_id_;
};

std::string IdentityModeToString(IdentityManager::Mode mode);
std::vector<TrackletObservation> TrackletObservationsFromTracks(const std::vector<Track> &tracks);

}  // namespace dog_patrol_perception_tracking
