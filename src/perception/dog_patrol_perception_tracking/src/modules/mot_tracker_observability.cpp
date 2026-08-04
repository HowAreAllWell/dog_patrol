#include "mot_tracker_observability.hpp"

#include <filesystem>
#include <fstream>
#include <utility>

namespace dog_patrol_perception_tracking {
namespace {

float StateVal(const cv::Mat &state, const int idx) {
  if (state.empty() || state.rows <= idx || state.cols <= 0) {
    return 0.0F;
  }
  return state.at<float>(idx, 0);
}

float MatVal(const cv::Mat &m, const int r, const int c) {
  if (m.empty() || m.rows <= r || m.cols <= c) {
    return 0.0F;
  }
  return m.at<float>(r, c);
}

class CsvMotTrackerObservabilityWriter final : public MotTrackerObservabilityWriter {
 public:
  explicit CsvMotTrackerObservabilityWriter(std::filesystem::path output_dir) : output_dir_(std::move(output_dir)) {}

  void BeginFrame(int) override { EnsureOpen(); }

  void WriteTracks(const int frame_id, const std::vector<MotTrackerTrackObservation> &tracks) override {
    EnsureOpen();
    if (!tracks_csv_.is_open()) {
      return;
    }
    for (const auto &track : tracks) {
      tracks_csv_ << frame_id << "," << track.track_idx << "," << track.track_id << ","
                  << static_cast<int>(track.class_id) << "," << track.state_code << ","
                  << (track.is_confirmed ? 1 : 0) << "," << track.hits << "," << track.age << ","
                  << track.time_since_update << "," << (track.has_appearance ? 1 : 0) << ","
                  << track.predicted_bbox.x << "," << track.predicted_bbox.y << ","
                  << track.predicted_bbox.width << "," << track.predicted_bbox.height << ","
                  << track.bbox.x << "," << track.bbox.y << "," << track.bbox.width << ","
                  << track.bbox.height << ","
                  << track.pre_gmc_pred_bbox.x << "," << track.pre_gmc_pred_bbox.y << ","
                  << track.pre_gmc_pred_bbox.width << "," << track.pre_gmc_pred_bbox.height << ","
                  << track.pre_gmc_bbox.x << "," << track.pre_gmc_bbox.y << ","
                  << track.pre_gmc_bbox.width << "," << track.pre_gmc_bbox.height << ","
                  << track.post_gmc_pred_bbox.x << "," << track.post_gmc_pred_bbox.y << ","
                  << track.post_gmc_pred_bbox.width << "," << track.post_gmc_pred_bbox.height << ","
                  << track.post_gmc_bbox.x << "," << track.post_gmc_bbox.y << ","
                  << track.post_gmc_bbox.width << "," << track.post_gmc_bbox.height;
      for (int k = 0; k < 8; ++k) {
        tracks_csv_ << "," << StateVal(track.pre_state_post, k);
      }
      for (int k = 0; k < 8; ++k) {
        tracks_csv_ << "," << StateVal(track.post_predict_state_pre, k);
      }
      for (int k = 0; k < 8; ++k) {
        tracks_csv_ << "," << StateVal(track.post_predict_state_post, k);
      }
      for (int k = 0; k < 8; ++k) {
        tracks_csv_ << "," << StateVal(track.post_gmc_state_pre, k);
      }
      for (int k = 0; k < 8; ++k) {
        tracks_csv_ << "," << StateVal(track.post_gmc_state_post, k);
      }
      for (int k = 0; k < 8; ++k) {
        tracks_csv_ << "," << MatVal(track.pre_error_cov_post, k, k);
      }
      for (int k = 0; k < 8; ++k) {
        tracks_csv_ << "," << MatVal(track.post_predict_error_cov_pre, k, k);
      }
      for (int k = 0; k < 8; ++k) {
        tracks_csv_ << "," << MatVal(track.post_predict_error_cov_post, k, k);
      }
      for (int k = 0; k < 8; ++k) {
        tracks_csv_ << "," << MatVal(track.post_gmc_error_cov_pre, k, k);
      }
      for (int k = 0; k < 8; ++k) {
        tracks_csv_ << "," << MatVal(track.post_gmc_error_cov_post, k, k);
      }
      tracks_csv_ << "\n";
    }
  }

  void WriteDetections(const int frame_id,
                       const std::vector<MotTrackerDetectionObservation> &detections) override {
    EnsureOpen();
    if (!detections_csv_.is_open()) {
      return;
    }
    for (const auto &detection : detections) {
      detections_csv_ << frame_id << "," << detection.level << "," << detection.det_local_idx << ","
                      << detection.det_src_idx << "," << static_cast<int>(detection.class_id) << ","
                      << detection.score << "," << detection.bbox.x << "," << detection.bbox.y << ","
                      << detection.bbox.width << "," << detection.bbox.height << "\n";
    }
  }

  void WriteGmc(const int frame_id, const MotTrackerGmcObservation &gmc) override {
    EnsureOpen();
    if (!gmc_csv_.is_open()) {
      return;
    }
    float w00 = 1.0F;
    float w01 = 0.0F;
    float w02 = 0.0F;
    float w10 = 0.0F;
    float w11 = 1.0F;
    float w12 = 0.0F;
    if (!gmc.warp.empty() && gmc.warp.rows == 2 && gmc.warp.cols == 3) {
      w00 = gmc.warp.at<float>(0, 0);
      w01 = gmc.warp.at<float>(0, 1);
      w02 = gmc.warp.at<float>(0, 2);
      w10 = gmc.warp.at<float>(1, 0);
      w11 = gmc.warp.at<float>(1, 1);
      w12 = gmc.warp.at<float>(1, 2);
    }
    gmc_csv_ << frame_id << "," << (gmc.ok ? 1 : 0) << ","
             << w00 << "," << w01 << "," << w02 << ","
             << w10 << "," << w11 << "," << w12 << "\n";
  }

  void WritePair(const int frame_id, const MotTrackerPairObservation &pair) override {
    EnsureOpen();
    if (!pairs_csv_.is_open()) {
      return;
    }
    const auto &terms = pair.terms;
    pairs_csv_ << frame_id << "," << pair.stage_name << "," << pair.track_idx << "," << pair.track_id << ","
               << pair.track_state_code << "," << pair.det_local_idx << "," << pair.det_src_idx << ","
               << terms.iou << "," << terms.motion_dist << "," << terms.gate_dist << ","
               << terms.assoc_motion_dist << "," << terms.motion_term_norm << ","
               << (terms.motion_ok ? 1 : 0) << "," << terms.motion_gate_effective_thresh << ","
               << (terms.iou_guard_pass ? 1 : 0) << "," << (terms.motion_gate_pass ? 1 : 0) << ","
               << (terms.app_enabled ? 1 : 0) << "," << (terms.app_available ? 1 : 0) << ","
               << terms.app_dist << "," << (terms.app_gate_pass ? 1 : 0) << ","
               << terms.measurement_cx << "," << terms.measurement_cy << ","
               << terms.measurement_a << "," << terms.measurement_h << ","
               << terms.residual_cx << "," << terms.residual_cy << ","
               << terms.residual_a << "," << terms.residual_h;
    for (int i = 0; i < 16; ++i) {
      pairs_csv_ << "," << terms.innovation_cov_s[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < 32; ++i) {
      pairs_csv_ << "," << terms.kalman_gain_k[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < 8; ++i) {
      pairs_csv_ << "," << terms.error_cov_pre_diag[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < 8; ++i) {
      pairs_csv_ << "," << terms.error_cov_post_diag[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < 8; ++i) {
      pairs_csv_ << "," << terms.process_noise_q_diag[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < 4; ++i) {
      pairs_csv_ << "," << terms.measurement_noise_r_diag[static_cast<std::size_t>(i)];
    }
    pairs_csv_ << ","
               << terms.fused_cost << "," << (terms.eligible ? 1 : 0) << "," << (pair.selected ? 1 : 0) << ","
               << terms.reject_reason << ","
               << pair.pre_gmc_pred_bbox.x << "," << pair.pre_gmc_pred_bbox.y << ","
               << pair.pre_gmc_pred_bbox.width << "," << pair.pre_gmc_pred_bbox.height << ","
               << pair.post_gmc_pred_bbox.x << "," << pair.post_gmc_pred_bbox.y << ","
               << pair.post_gmc_pred_bbox.width << "," << pair.post_gmc_pred_bbox.height;
    for (int k = 0; k < 8; ++k) {
      pairs_csv_ << "," << StateVal(pair.pre_state_post, k);
    }
    for (int k = 0; k < 8; ++k) {
      pairs_csv_ << "," << StateVal(pair.post_predict_state_pre, k);
    }
    for (int k = 0; k < 8; ++k) {
      pairs_csv_ << "," << StateVal(pair.post_gmc_state_pre, k);
    }
    pairs_csv_ << "\n";
  }

 private:
  void EnsureOpen() {
    if (opened_ || output_dir_.empty()) {
      return;
    }
    std::filesystem::create_directories(output_dir_);
    tracks_csv_.open(output_dir_ / "tracks.csv");
    detections_csv_.open(output_dir_ / "detections.csv");
    gmc_csv_.open(output_dir_ / "gmc.csv");
    pairs_csv_.open(output_dir_ / "pairs.csv");

    if (tracks_csv_.is_open()) {
      tracks_csv_
          << "frame,track_idx,track_id,class_id,state_code,is_confirmed,hits,age,time_since_update,has_appearance,"
             "pred_x,pred_y,pred_w,pred_h,last_x,last_y,last_w,last_h,"
             "pre_gmc_pred_x,pre_gmc_pred_y,pre_gmc_pred_w,pre_gmc_pred_h,"
             "pre_gmc_last_x,pre_gmc_last_y,pre_gmc_last_w,pre_gmc_last_h,"
             "post_gmc_pred_x,post_gmc_pred_y,post_gmc_pred_w,post_gmc_pred_h,"
             "post_gmc_last_x,post_gmc_last_y,post_gmc_last_w,post_gmc_last_h,"
             "pre_state_post_0,pre_state_post_1,pre_state_post_2,pre_state_post_3,pre_state_post_4,pre_state_post_5,pre_state_post_6,pre_state_post_7,"
             "post_predict_state_pre_0,post_predict_state_pre_1,post_predict_state_pre_2,post_predict_state_pre_3,post_predict_state_pre_4,post_predict_state_pre_5,post_predict_state_pre_6,post_predict_state_pre_7,"
             "post_predict_state_post_0,post_predict_state_post_1,post_predict_state_post_2,post_predict_state_post_3,post_predict_state_post_4,post_predict_state_post_5,post_predict_state_post_6,post_predict_state_post_7,"
             "post_gmc_state_pre_0,post_gmc_state_pre_1,post_gmc_state_pre_2,post_gmc_state_pre_3,post_gmc_state_pre_4,post_gmc_state_pre_5,post_gmc_state_pre_6,post_gmc_state_pre_7,"
             "post_gmc_state_post_0,post_gmc_state_post_1,post_gmc_state_post_2,post_gmc_state_post_3,post_gmc_state_post_4,post_gmc_state_post_5,post_gmc_state_post_6,post_gmc_state_post_7,"
             "pre_error_cov_post_d0,pre_error_cov_post_d1,pre_error_cov_post_d2,pre_error_cov_post_d3,pre_error_cov_post_d4,pre_error_cov_post_d5,pre_error_cov_post_d6,pre_error_cov_post_d7,"
             "post_predict_error_cov_pre_d0,post_predict_error_cov_pre_d1,post_predict_error_cov_pre_d2,post_predict_error_cov_pre_d3,post_predict_error_cov_pre_d4,post_predict_error_cov_pre_d5,post_predict_error_cov_pre_d6,post_predict_error_cov_pre_d7,"
             "post_predict_error_cov_post_d0,post_predict_error_cov_post_d1,post_predict_error_cov_post_d2,post_predict_error_cov_post_d3,post_predict_error_cov_post_d4,post_predict_error_cov_post_d5,post_predict_error_cov_post_d6,post_predict_error_cov_post_d7,"
             "post_gmc_error_cov_pre_d0,post_gmc_error_cov_pre_d1,post_gmc_error_cov_pre_d2,post_gmc_error_cov_pre_d3,post_gmc_error_cov_pre_d4,post_gmc_error_cov_pre_d5,post_gmc_error_cov_pre_d6,post_gmc_error_cov_pre_d7,"
             "post_gmc_error_cov_post_d0,post_gmc_error_cov_post_d1,post_gmc_error_cov_post_d2,post_gmc_error_cov_post_d3,post_gmc_error_cov_post_d4,post_gmc_error_cov_post_d5,post_gmc_error_cov_post_d6,post_gmc_error_cov_post_d7\n";
    }
    if (gmc_csv_.is_open()) {
      gmc_csv_ << "frame,gmc_ok,warp00,warp01,warp02,warp10,warp11,warp12\n";
    }
    if (detections_csv_.is_open()) {
      detections_csv_ << "frame,level,det_local_idx,det_src_idx,class,score,x,y,w,h\n";
    }
    if (pairs_csv_.is_open()) {
      pairs_csv_
          << "frame,stage,track_idx,track_id,track_state_code,det_local_idx,det_src_idx,"
             "iou,motion_dist,gate_dist,assoc_motion_dist,motion_term_norm,motion_ok,motion_gate_thresh_effective,motion_iou_guard_pass,motion_gate_pass,"
             "app_enabled,app_available,app_dist,app_gate_pass,"
             "measurement_cx,measurement_cy,measurement_a,measurement_h,"
             "residual_cx,residual_cy,residual_a,residual_h,"
             "innovation_cov_s_00,innovation_cov_s_01,innovation_cov_s_02,innovation_cov_s_03,innovation_cov_s_10,innovation_cov_s_11,innovation_cov_s_12,innovation_cov_s_13,innovation_cov_s_20,innovation_cov_s_21,innovation_cov_s_22,innovation_cov_s_23,innovation_cov_s_30,innovation_cov_s_31,innovation_cov_s_32,innovation_cov_s_33,"
             "kalman_gain_k_00,kalman_gain_k_01,kalman_gain_k_02,kalman_gain_k_03,kalman_gain_k_10,kalman_gain_k_11,kalman_gain_k_12,kalman_gain_k_13,kalman_gain_k_20,kalman_gain_k_21,kalman_gain_k_22,kalman_gain_k_23,kalman_gain_k_30,kalman_gain_k_31,kalman_gain_k_32,kalman_gain_k_33,kalman_gain_k_40,kalman_gain_k_41,kalman_gain_k_42,kalman_gain_k_43,kalman_gain_k_50,kalman_gain_k_51,kalman_gain_k_52,kalman_gain_k_53,kalman_gain_k_60,kalman_gain_k_61,kalman_gain_k_62,kalman_gain_k_63,kalman_gain_k_70,kalman_gain_k_71,kalman_gain_k_72,kalman_gain_k_73,"
             "error_cov_pre_d0,error_cov_pre_d1,error_cov_pre_d2,error_cov_pre_d3,error_cov_pre_d4,error_cov_pre_d5,error_cov_pre_d6,error_cov_pre_d7,"
             "error_cov_post_d0,error_cov_post_d1,error_cov_post_d2,error_cov_post_d3,error_cov_post_d4,error_cov_post_d5,error_cov_post_d6,error_cov_post_d7,"
             "process_noise_q_d0,process_noise_q_d1,process_noise_q_d2,process_noise_q_d3,process_noise_q_d4,process_noise_q_d5,process_noise_q_d6,process_noise_q_d7,"
             "measurement_noise_r_d0,measurement_noise_r_d1,measurement_noise_r_d2,measurement_noise_r_d3,"
             "fused_cost,eligible,selected,reject_reason,"
             "pre_gmc_pred_x,pre_gmc_pred_y,pre_gmc_pred_w,pre_gmc_pred_h,"
             "post_gmc_pred_x,post_gmc_pred_y,post_gmc_pred_w,post_gmc_pred_h,"
             "pre_state_post_0,pre_state_post_1,pre_state_post_2,pre_state_post_3,pre_state_post_4,pre_state_post_5,pre_state_post_6,pre_state_post_7,"
             "post_predict_state_pre_0,post_predict_state_pre_1,post_predict_state_pre_2,post_predict_state_pre_3,post_predict_state_pre_4,post_predict_state_pre_5,post_predict_state_pre_6,post_predict_state_pre_7,"
             "post_gmc_state_pre_0,post_gmc_state_pre_1,post_gmc_state_pre_2,post_gmc_state_pre_3,post_gmc_state_pre_4,post_gmc_state_pre_5,post_gmc_state_pre_6,post_gmc_state_pre_7\n";
    }
    opened_ = true;
  }

  std::filesystem::path output_dir_;
  bool opened_{false};
  std::ofstream tracks_csv_;
  std::ofstream detections_csv_;
  std::ofstream gmc_csv_;
  std::ofstream pairs_csv_;
};

}  // namespace

MotTrackerObservability::MotTrackerObservability(MotTrackerObservabilityConfig config,
                                                 std::unique_ptr<MotTrackerObservabilityWriter> writer)
    : config_(std::move(config)), writer_(std::move(writer)) {}

std::unique_ptr<MotTrackerObservability> MotTrackerObservability::CreateDisabled() {
  return std::make_unique<MotTrackerObservability>(MotTrackerObservabilityConfig{}, nullptr);
}

std::unique_ptr<MotTrackerObservability> MotTrackerObservability::CreateCsv(MotTrackerObservabilityConfig config) {
  if (!config.enabled || config.output_dir.empty()) {
    return CreateDisabled();
  }
  return std::make_unique<MotTrackerObservability>(
      config, std::make_unique<CsvMotTrackerObservabilityWriter>(config.output_dir));
}

bool MotTrackerObservability::EnabledForFrame(const int frame_id) const {
  if (!config_.enabled || writer_ == nullptr) {
    return false;
  }
  if (frame_id < config_.frame_start) {
    return false;
  }
  if (config_.frame_end >= 0 && frame_id > config_.frame_end) {
    return false;
  }
  return true;
}

void MotTrackerObservability::BeginFrame(const int frame_id) {
  if (config_.enabled && writer_ != nullptr) {
    writer_->BeginFrame(frame_id);
  }
}

void MotTrackerObservability::WriteTracks(const int frame_id,
                                          const std::vector<MotTrackerTrackObservation> &tracks) {
  if (EnabledForFrame(frame_id)) {
    writer_->WriteTracks(frame_id, tracks);
  }
}

void MotTrackerObservability::WriteDetections(
    const int frame_id, const std::vector<MotTrackerDetectionObservation> &detections) {
  if (EnabledForFrame(frame_id)) {
    writer_->WriteDetections(frame_id, detections);
  }
}

void MotTrackerObservability::WriteGmc(const int frame_id, const MotTrackerGmcObservation &gmc) {
  if (EnabledForFrame(frame_id)) {
    writer_->WriteGmc(frame_id, gmc);
  }
}

void MotTrackerObservability::WritePair(const int frame_id, const MotTrackerPairObservation &pair) {
  if (EnabledForFrame(frame_id)) {
    writer_->WritePair(frame_id, pair);
  }
}

}  // namespace dog_patrol_perception_tracking
