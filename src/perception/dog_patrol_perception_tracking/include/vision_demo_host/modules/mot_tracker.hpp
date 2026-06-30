#pragma once

#include <array>
#include <fstream>
#include <opencv2/video/tracking.hpp>
#include <string>
#include <vector>

#include "vision_demo_host/modules/appearance_feature_service.hpp"
#include "vision_demo_host/types.hpp"

namespace vision_demo_host {

class MotTracker {
 public:
  struct Config {
    std::string tracker_yaml_path;
    std::string core_mode{"new_core"};  // old_minimal | new_core
    bool gmc_enabled{true};
    bool reid_enabled{true};
    float track_high_thresh{0.5F};
    float track_low_thresh{0.1F};
    float new_track_thresh{0.7F};
    float match_thresh{0.8F};
    int track_buffer{30};
    std::string gmc_method{"sparseOptFlow"};
    int gmc_downscale{4};
    bool with_reid{true};
    std::string reid_backend{"light"};
    std::string reid_model_path{};
    int reid_input_width{128};
    int reid_input_height{256};
    int confirm_hits{3};
    float stage1_iou_min{0.2F};
    float stage2_iou_min{0.15F};
    float unconfirmed_iou_min{0.3F};
    float stage1_max_cost{0.70F};
    float stage2_max_cost{0.65F};
    float lost_recovery_max_cost{0.60F};
    float unconfirmed_max_cost{0.60F};
    bool use_low_score_appearance_gate{true};
    float duplicate_lost_iou{0.50F};
    float duplicate_lost_center_dist_norm{1.0F};
    float motion_gate_thresh{9.4877F};  // Chi-square(4 dof, 0.95)
    float assoc_iou_weight{0.55F};
    float assoc_motion_weight{0.25F};
    float assoc_app_weight{0.20F};
    float appearance_gate{0.45F};
    float appearance_alpha{0.9F};  // EMA momentum
    int appearance_h_bins{16};
    int appearance_s_bins{8};
    int occlusion_enter_min_frames{3};
    int occlusion_base_frames{24};
    int occlusion_extend_step{5};
    int occlusion_max_frames{45};
    int occlusion_release_clear_frames{4};
    float occlusion_shrink_ratio{0.45F};
    float occlusion_suspect_area_ratio{0.60F};
    float occlusion_neighbor_center_dist_norm{1.20F};
    float occlusion_overlap_iou_min{0.02F};
    // Diagnostic-only knobs (no default behavior change).
    bool diag_assoc_enable{false};
    std::string diag_assoc_dir;
    int diag_frame_start{0};
    int diag_frame_end{-1};
    bool diag_stage1_use_appearance{true};
  };

  explicit MotTracker(Config config);

  bool Initialize(std::string *error);
  std::vector<Track> Update(const std::vector<Detection> &detections, const cv::Mat &frame);
  const Config &EffectiveConfig() const { return config_; }
  const std::vector<TrackletHypothesis> &LastTrackletHypotheses() const { return last_tracklet_hypotheses_; }

 private:
  enum class TrackLifeState {
    kTracked,
    kLost,
    kRemoved,
  };

  struct TrackState {
    int id{-1};
    ClassId class_id{ClassId::kUnknown};
    float score{0.0F};
    cv::Rect2f bbox;
    cv::Rect2f predicted_bbox;
    cv::KalmanFilter kf;
    int age{0};
    int hits{0};
    int time_since_update{0};
    bool is_confirmed{false};
    TrackLifeState life_state{TrackLifeState::kTracked};
    cv::Mat appearance_feat;  // 1 x D, CV_32F, L2-normalized.
    bool has_appearance{false};
    float stable_area{0.0F};
    int occlusion_candidate_streak{0};
    int occlusion_clear_streak{0};
    int occlusion_protect_remaining{0};
    bool occlusion_suspect{false};
    AssociationEvidence last_association;
    bool just_recovered{false};
    bool low_score_update{false};
  };

  bool ParseTrackerConfig(std::string *error);
  void InitializeKalman(TrackState *track, const cv::Rect2f &bbox) const;
  cv::Rect2f PredictTrack(TrackState *track) const;
  void UpdateTrack(TrackState *track, const Detection &det);
  void UpdateTrackNewCore(TrackState *track, const Detection &det, const cv::Mat &appearance_feat);
  cv::Mat ExtractAppearanceFeature(const cv::Mat &frame, const cv::Rect2f &bbox) const;
  float AppearanceDistance(const TrackState &track, const cv::Mat &det_feat) const;
  float MahalanobisDistance(const TrackState &track, const Detection &det) const;
  struct AssocTerms {
    float iou{0.0F};
    float motion_dist{1e6F};
    float gate_dist{1e6F};
    float assoc_motion_dist{1e6F};
    float motion_term_norm{1.0F};
    bool motion_ok{false};
    bool motion_gate_pass{true};
    bool app_enabled{false};
    bool app_available{false};
    float app_dist{0.0F};
    bool app_gate_pass{true};
    float motion_gate_effective_thresh{0.0F};
    bool iou_guard_pass{false};
    float measurement_cx{0.0F};
    float measurement_cy{0.0F};
    float measurement_a{0.0F};
    float measurement_h{0.0F};
    float residual_cx{0.0F};
    float residual_cy{0.0F};
    float residual_a{0.0F};
    float residual_h{0.0F};
    std::array<float, 16> innovation_cov_s{};
    std::array<float, 32> kalman_gain_k{};
    std::array<float, 8> error_cov_pre_diag{};
    std::array<float, 8> error_cov_post_diag{};
    std::array<float, 8> process_noise_q_diag{};
    std::array<float, 4> measurement_noise_r_diag{};
    float fused_cost{1e6F};
    bool eligible{false};
    std::string reject_reason;
  };
  struct TrackDiagSnapshot {
    bool valid{false};
    int track_id{-1};
    ClassId class_id{ClassId::kUnknown};
    TrackLifeState life_state{TrackLifeState::kTracked};
    cv::Rect2f pre_gmc_pred_bbox;
    cv::Rect2f pre_gmc_bbox;
    cv::Rect2f post_gmc_pred_bbox;
    cv::Rect2f post_gmc_bbox;
    cv::Mat pre_state_post;          // 8x1
    cv::Mat post_predict_state_pre;  // 8x1
    cv::Mat post_predict_state_post; // 8x1
    cv::Mat post_gmc_state_pre;      // 8x1
    cv::Mat post_gmc_state_post;     // 8x1
    cv::Mat pre_error_cov_post;      // 8x8
    cv::Mat post_predict_error_cov_pre;   // 8x8
    cv::Mat post_predict_error_cov_post;  // 8x8
    cv::Mat post_gmc_error_cov_pre;       // 8x8
    cv::Mat post_gmc_error_cov_post;      // 8x8
  };
  AssocTerms ComputeAssociationTerms(const TrackState &track, const Detection &det, const cv::Mat &det_feat,
                                     bool enable_appearance, float iou_min) const;
  float AssociationCost(const TrackState &track, const Detection &det, const cv::Mat &det_feat,
                        bool enable_appearance, float iou_min, float *out_iou) const;
  void MaybeOpenDiagFiles();
  bool DiagFrameEnabled() const;
  void DiagWriteTracks(const std::vector<Detection> &det_high, const std::vector<Detection> &det_low);
  void DiagWriteDetections(const std::vector<Detection> &det_high, const std::vector<Detection> &det_low) const;
  void DiagWriteGmc() const;
  void DiagWritePairs(const std::string &stage_name, int track_idx, int det_local_idx, int det_src_idx,
                      const AssocTerms &terms, bool selected) const;
  AssociationEvidence EvidenceFromTerms(const AssocTerms &terms, const std::string &stage_name,
                                        bool low_score_detection, bool recovered_from_lost) const;
  void SetAcceptedAssociation(TrackState *track, const AssocTerms &terms, const std::string &stage_name,
                              bool low_score_detection, bool recovered_from_lost) const;
  std::vector<std::pair<int, int>> MatchByHungarian(const std::vector<int> &track_indices,
                                                    const std::vector<Detection> &detections,
                                                    const std::vector<cv::Mat> &det_feats,
                                                    bool enable_appearance,
                                                    float iou_min,
                                                    float stage_max_cost,
                                                    const std::string &stage_name,
                                                    const std::vector<int> *det_src_indices) const;
  std::vector<std::pair<int, int>> GreedyMatch(const std::vector<int> &track_indices,
                                               const std::vector<Detection> &detections,
                                               float min_iou) const;
  void ApplyGmc(const cv::Mat &frame, std::vector<TrackState> *tracks, bool *out_gmc_ok, cv::Mat *out_warp);
  std::vector<Track> UpdateOldMinimal(const std::vector<Detection> &detections, const cv::Mat &frame);
  std::vector<Track> UpdateNewCore(const std::vector<Detection> &detections, const cv::Mat &frame);
  float ComputeIoU(const cv::Rect2f &a, const cv::Rect2f &b) const;
  float CenterDistanceNorm(const cv::Rect2f &a, const cv::Rect2f &b) const;
  bool ShouldSuppressNewTrack(const Detection &det) const;
  bool IsDuplicateOutputTrack(const TrackState &candidate, const TrackState &other) const;
  bool UsingTrueReid() const;
  void NormalizeAppearanceFeature(cv::Mat *feature) const;
  bool IsOcclusionCandidate(int track_idx) const;
  void UpdateOcclusionProtection();
  void MirrorTrackedHypotheses(const std::vector<Track> &tracks);

  Config config_;
  int next_track_id_{1};
  int frame_id_{0};
  std::vector<TrackState> tracks_;
  AppearanceFeatureService appearance_features_;
  cv::Mat prev_gray_;
  mutable std::ofstream diag_tracks_csv_;
  mutable std::ofstream diag_dets_csv_;
  mutable std::ofstream diag_gmc_csv_;
  mutable std::ofstream diag_pairs_csv_;
  bool diag_files_opened_{false};
  bool diag_gmc_ok_{false};
  cv::Mat diag_gmc_warp_;
  std::vector<TrackDiagSnapshot> diag_snapshots_;
  std::vector<TrackletHypothesis> last_tracklet_hypotheses_;
};

}  // namespace vision_demo_host
