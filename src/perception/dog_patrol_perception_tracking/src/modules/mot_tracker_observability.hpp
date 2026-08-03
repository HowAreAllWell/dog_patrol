#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "vision_demo_host/types.hpp"

namespace vision_demo_host {

struct MotTrackerObservabilityConfig {
  bool enabled{false};
  std::filesystem::path output_dir;
  int frame_start{0};
  int frame_end{-1};
};

struct MotTrackerTrackObservation {
  int track_idx{-1};
  int track_id{-1};
  ClassId class_id{ClassId::kUnknown};
  int state_code{0};
  bool is_confirmed{false};
  int hits{0};
  int age{0};
  int time_since_update{0};
  bool has_appearance{false};
  cv::Rect2f predicted_bbox;
  cv::Rect2f bbox;
  cv::Rect2f pre_gmc_pred_bbox;
  cv::Rect2f pre_gmc_bbox;
  cv::Rect2f post_gmc_pred_bbox;
  cv::Rect2f post_gmc_bbox;
  cv::Mat pre_state_post;
  cv::Mat post_predict_state_pre;
  cv::Mat post_predict_state_post;
  cv::Mat post_gmc_state_pre;
  cv::Mat post_gmc_state_post;
  cv::Mat pre_error_cov_post;
  cv::Mat post_predict_error_cov_pre;
  cv::Mat post_predict_error_cov_post;
  cv::Mat post_gmc_error_cov_pre;
  cv::Mat post_gmc_error_cov_post;
};

struct MotTrackerDetectionObservation {
  std::string level;
  int det_local_idx{-1};
  int det_src_idx{-1};
  ClassId class_id{ClassId::kUnknown};
  float score{0.0F};
  cv::Rect2f bbox;
};

struct MotTrackerGmcObservation {
  bool ok{false};
  cv::Mat warp;
};

struct MotTrackerAssociationTermObservation {
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

struct MotTrackerPairObservation {
  std::string stage_name;
  int track_idx{-1};
  int track_id{-1};
  int track_state_code{0};
  int det_local_idx{-1};
  int det_src_idx{-1};
  MotTrackerAssociationTermObservation terms;
  bool selected{false};
  cv::Rect2f pre_gmc_pred_bbox;
  cv::Rect2f post_gmc_pred_bbox;
  cv::Mat pre_state_post;
  cv::Mat post_predict_state_pre;
  cv::Mat post_gmc_state_pre;
};

class MotTrackerObservabilityWriter {
 public:
  virtual ~MotTrackerObservabilityWriter() = default;

  virtual void BeginFrame(int frame_id) = 0;
  virtual void WriteTracks(int frame_id, const std::vector<MotTrackerTrackObservation> &tracks) = 0;
  virtual void WriteDetections(int frame_id, const std::vector<MotTrackerDetectionObservation> &detections) = 0;
  virtual void WriteGmc(int frame_id, const MotTrackerGmcObservation &gmc) = 0;
  virtual void WritePair(int frame_id, const MotTrackerPairObservation &pair) = 0;
};

class MotTrackerObservability {
 public:
  MotTrackerObservability(MotTrackerObservabilityConfig config,
                          std::unique_ptr<MotTrackerObservabilityWriter> writer);

  static std::unique_ptr<MotTrackerObservability> CreateDisabled();
  static std::unique_ptr<MotTrackerObservability> CreateCsv(MotTrackerObservabilityConfig config);

  bool EnabledForFrame(int frame_id) const;
  void BeginFrame(int frame_id);
  void WriteTracks(int frame_id, const std::vector<MotTrackerTrackObservation> &tracks);
  void WriteDetections(int frame_id, const std::vector<MotTrackerDetectionObservation> &detections);
  void WriteGmc(int frame_id, const MotTrackerGmcObservation &gmc);
  void WritePair(int frame_id, const MotTrackerPairObservation &pair);

 private:
  MotTrackerObservabilityConfig config_;
  std::unique_ptr<MotTrackerObservabilityWriter> writer_;
};

}  // namespace vision_demo_host
