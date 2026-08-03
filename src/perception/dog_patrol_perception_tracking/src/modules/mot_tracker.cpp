#include "vision_demo_host/modules/mot_tracker.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "vision_demo_host/modules/association_utils.hpp"

#include "mot_tracker_observability.hpp"

namespace vision_demo_host {
namespace {

constexpr float kLargeCost = 1e6F;
constexpr float kDuplicateTrackedSpawnIou = 0.10F;
constexpr float kDuplicateTrackedCenterDistNorm = 0.25F;
constexpr float kDuplicateTrackedLowOverlapIou = 0.20F;
constexpr float kDuplicateTrackedLowOverlapCenterDistNorm = 0.30F;
constexpr float kDuplicateOutputIou = 0.60F;
constexpr float kDuplicateOutputCenterDistNorm = 0.15F;
constexpr float kDuplicateOutputMaxAreaRatio = 1.05F;
constexpr int kLostCenterDuplicateSuppressFrames = 6;

std::string Trim(const std::string &s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

bool ParseBool(std::string v) {
  v = Trim(v);
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v == "true" || v == "1" || v == "yes" || v == "on";
}

float ParseFloat(const std::string &v, const float fallback) {
  try {
    return std::stof(Trim(v));
  } catch (...) {
    return fallback;
  }
}

int ParseInt(const std::string &v, const int fallback) {
  try {
    return std::stoi(Trim(v));
  } catch (...) {
    return fallback;
  }
}

cv::Rect2f ClampRect(const cv::Rect2f &r) {
  return cv::Rect2f(std::max(0.0F, r.x), std::max(0.0F, r.y), std::max(0.0F, r.width), std::max(0.0F, r.height));
}

std::string ToLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

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

std::vector<float> FeatureMatToVector(const cv::Mat &feature) {
  if (feature.empty() || feature.rows != 1 || feature.type() != CV_32F) {
    return {};
  }
  std::vector<float> out(static_cast<std::size_t>(feature.cols));
  for (int i = 0; i < feature.cols; ++i) {
    out[static_cast<std::size_t>(i)] = feature.at<float>(0, i);
  }
  return out;
}

// Hungarian algorithm (Kuhn-Munkres) for rectangular minimum-cost assignment.
// Returns row->col mapping, -1 for unmatched rows.
std::vector<int> HungarianAssign(const std::vector<std::vector<float>> &cost) {
  const int rows = static_cast<int>(cost.size());
  if (rows == 0) {
    return {};
  }
  const int cols = static_cast<int>(cost[0].size());
  if (cols == 0) {
    return std::vector<int>(rows, -1);
  }

  bool transposed = false;
  std::vector<std::vector<float>> a = cost;
  int n = rows;
  int m = cols;
  if (n > m) {
    transposed = true;
    std::vector<std::vector<float>> t(cols, std::vector<float>(rows, 0.0F));
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        t[j][i] = cost[i][j];
      }
    }
    a.swap(t);
    n = static_cast<int>(a.size());
    m = static_cast<int>(a[0].size());
  }

  std::vector<float> u(n + 1, 0.0F), v(m + 1, 0.0F);
  std::vector<int> p(m + 1, 0), way(m + 1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<float> minv(m + 1, std::numeric_limits<float>::infinity());
    std::vector<char> used(m + 1, false);

    do {
      used[j0] = true;
      const int i0 = p[j0];
      float delta = std::numeric_limits<float>::infinity();
      int j1 = 0;
      for (int j = 1; j <= m; ++j) {
        if (used[j]) {
          continue;
        }
        const float cur = a[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= m; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> assign_r(rows, -1);
  if (!transposed) {
    for (int j = 1; j <= m; ++j) {
      if (p[j] > 0 && p[j] <= rows && (j - 1) < cols) {
        assign_r[p[j] - 1] = j - 1;
      }
    }
  } else {
    // p[j] indexes transposed rows (original cols). j indexes original rows.
    for (int j = 1; j <= m; ++j) {
      const int orig_row = j - 1;
      const int orig_col = p[j] - 1;
      if (orig_row >= 0 && orig_row < rows && orig_col >= 0 && orig_col < cols) {
        assign_r[orig_row] = orig_col;
      }
    }
  }

  return assign_r;
}

}  // namespace

MotTracker::MotTracker(Config config) : config_(std::move(config)), appearance_features_() {}

MotTracker::~MotTracker() = default;

MotTracker::MotTracker(MotTracker &&other) noexcept = default;

MotTracker &MotTracker::operator=(MotTracker &&other) noexcept = default;

bool MotTracker::ParseTrackerConfig(std::string *error) {
  std::ifstream ifs(config_.tracker_yaml_path);
  if (!ifs.good()) {
    if (error != nullptr) {
      *error = "Failed to open tracker config: " + config_.tracker_yaml_path;
    }
    return false;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }

    const auto pos = line.find(':');
    if (pos == std::string::npos) {
      continue;
    }

    std::string key = Trim(line.substr(0, pos));
    std::string value = Trim(line.substr(pos + 1));
    if (!value.empty() && (value.front() == '"' || value.front() == '\'')) {
      value.erase(value.begin());
    }
    if (!value.empty() && (value.back() == '"' || value.back() == '\'')) {
      value.pop_back();
    }

  if (key == "core_mode") {
      config_.core_mode = ToLower(value);
    } else if (key == "track_high_thresh") {
      config_.track_high_thresh = ParseFloat(value, config_.track_high_thresh);
    } else if (key == "track_low_thresh") {
      config_.track_low_thresh = ParseFloat(value, config_.track_low_thresh);
    } else if (key == "new_track_thresh") {
      config_.new_track_thresh = ParseFloat(value, config_.new_track_thresh);
    } else if (key == "match_thresh") {
      config_.match_thresh = ParseFloat(value, config_.match_thresh);
    } else if (key == "track_buffer") {
      config_.track_buffer = ParseInt(value, config_.track_buffer);
    } else if (key == "gmc_method") {
      config_.gmc_method = value;
    } else if (key == "gmc_downscale") {
      config_.gmc_downscale = std::max(1, ParseInt(value, config_.gmc_downscale));
    } else if (key == "with_reid") {
      config_.with_reid = ParseBool(value);
    } else if (key == "reid_backend") {
      config_.reid_backend = value;
    } else if (key == "reid_model_path") {
      config_.reid_model_path = value;
    } else if (key == "reid_input_width") {
      config_.reid_input_width = std::max(16, ParseInt(value, config_.reid_input_width));
    } else if (key == "reid_input_height") {
      config_.reid_input_height = std::max(16, ParseInt(value, config_.reid_input_height));
    } else if (key == "confirm_hits") {
      config_.confirm_hits = std::max(1, ParseInt(value, config_.confirm_hits));
    } else if (key == "stage1_iou_min") {
      config_.stage1_iou_min = ParseFloat(value, config_.stage1_iou_min);
    } else if (key == "stage2_iou_min") {
      config_.stage2_iou_min = ParseFloat(value, config_.stage2_iou_min);
    } else if (key == "unconfirmed_iou_min") {
      config_.unconfirmed_iou_min = ParseFloat(value, config_.unconfirmed_iou_min);
    } else if (key == "stage1_max_cost") {
      config_.stage1_max_cost = ParseFloat(value, config_.stage1_max_cost);
    } else if (key == "stage2_max_cost") {
      config_.stage2_max_cost = ParseFloat(value, config_.stage2_max_cost);
    } else if (key == "lost_recovery_max_cost") {
      config_.lost_recovery_max_cost = ParseFloat(value, config_.lost_recovery_max_cost);
    } else if (key == "unconfirmed_max_cost") {
      config_.unconfirmed_max_cost = ParseFloat(value, config_.unconfirmed_max_cost);
    } else if (key == "use_low_score_appearance_gate") {
      config_.use_low_score_appearance_gate = ParseBool(value);
    } else if (key == "duplicate_lost_iou") {
      config_.duplicate_lost_iou = ParseFloat(value, config_.duplicate_lost_iou);
    } else if (key == "duplicate_lost_center_dist_norm") {
      config_.duplicate_lost_center_dist_norm = ParseFloat(value, config_.duplicate_lost_center_dist_norm);
    } else if (key == "motion_gate_thresh") {
      config_.motion_gate_thresh = ParseFloat(value, config_.motion_gate_thresh);
    } else if (key == "assoc_iou_weight") {
      config_.assoc_iou_weight = ParseFloat(value, config_.assoc_iou_weight);
    } else if (key == "assoc_motion_weight") {
      config_.assoc_motion_weight = ParseFloat(value, config_.assoc_motion_weight);
    } else if (key == "assoc_app_weight") {
      config_.assoc_app_weight = ParseFloat(value, config_.assoc_app_weight);
    } else if (key == "appearance_gate") {
      config_.appearance_gate = ParseFloat(value, config_.appearance_gate);
    } else if (key == "appearance_alpha") {
      config_.appearance_alpha = ParseFloat(value, config_.appearance_alpha);
    } else if (key == "appearance_h_bins") {
      config_.appearance_h_bins = std::max(4, ParseInt(value, config_.appearance_h_bins));
    } else if (key == "appearance_s_bins") {
      config_.appearance_s_bins = std::max(4, ParseInt(value, config_.appearance_s_bins));
    } else if (key == "occlusion_enter_min_frames") {
      config_.occlusion_enter_min_frames = std::max(1, ParseInt(value, config_.occlusion_enter_min_frames));
    } else if (key == "occlusion_base_frames") {
      config_.occlusion_base_frames = std::max(1, ParseInt(value, config_.occlusion_base_frames));
    } else if (key == "occlusion_extend_step") {
      config_.occlusion_extend_step = std::max(1, ParseInt(value, config_.occlusion_extend_step));
    } else if (key == "occlusion_max_frames") {
      config_.occlusion_max_frames = std::max(1, ParseInt(value, config_.occlusion_max_frames));
    } else if (key == "occlusion_release_clear_frames") {
      config_.occlusion_release_clear_frames = std::max(1, ParseInt(value, config_.occlusion_release_clear_frames));
    } else if (key == "occlusion_shrink_ratio") {
      config_.occlusion_shrink_ratio = ParseFloat(value, config_.occlusion_shrink_ratio);
    } else if (key == "occlusion_suspect_area_ratio") {
      config_.occlusion_suspect_area_ratio = ParseFloat(value, config_.occlusion_suspect_area_ratio);
    } else if (key == "occlusion_neighbor_center_dist_norm") {
      config_.occlusion_neighbor_center_dist_norm = ParseFloat(value, config_.occlusion_neighbor_center_dist_norm);
    } else if (key == "occlusion_overlap_iou_min") {
      config_.occlusion_overlap_iou_min = ParseFloat(value, config_.occlusion_overlap_iou_min);
    } else if (key == "diag_assoc_enable") {
      config_.diag_assoc_enable = ParseBool(value);
    } else if (key == "diag_assoc_dir") {
      config_.diag_assoc_dir = value;
    } else if (key == "diag_frame_start") {
      config_.diag_frame_start = ParseInt(value, config_.diag_frame_start);
    } else if (key == "diag_frame_end") {
      config_.diag_frame_end = ParseInt(value, config_.diag_frame_end);
    } else if (key == "diag_stage1_use_appearance") {
      config_.diag_stage1_use_appearance = ParseBool(value);
    } else if (key == "use_gmc") {
      (void)ParseBool(value);
    } else if (key == "use_reid") {
      (void)ParseBool(value);
    }
  }

  if (config_.track_low_thresh > config_.track_high_thresh) {
    std::swap(config_.track_low_thresh, config_.track_high_thresh);
  }

  if (config_.stage1_iou_min <= 0.0F) {
    config_.stage1_iou_min = std::max(0.01F, 1.0F - config_.match_thresh);
  }
  if (config_.stage2_iou_min <= 0.0F) {
    config_.stage2_iou_min = std::max(0.01F, std::min(config_.stage1_iou_min, 0.15F));
  }
  if (config_.unconfirmed_iou_min <= 0.0F) {
    config_.unconfirmed_iou_min = std::max(0.01F, config_.stage1_iou_min);
  }

  config_.assoc_iou_weight = std::max(0.0F, config_.assoc_iou_weight);
  config_.assoc_motion_weight = std::max(0.0F, config_.assoc_motion_weight);
  config_.assoc_app_weight = std::max(0.0F, config_.assoc_app_weight);
  if ((config_.assoc_iou_weight + config_.assoc_motion_weight + config_.assoc_app_weight) <= 1e-6F) {
    config_.assoc_iou_weight = 1.0F;
    config_.assoc_motion_weight = 0.0F;
    config_.assoc_app_weight = 0.0F;
  }

  config_.stage1_max_cost = std::clamp(config_.stage1_max_cost, 0.0F, 1.0F);
  config_.stage2_max_cost = std::clamp(config_.stage2_max_cost, 0.0F, 1.0F);
  config_.lost_recovery_max_cost = std::clamp(config_.lost_recovery_max_cost, 0.0F, 1.0F);
  config_.unconfirmed_max_cost = std::clamp(config_.unconfirmed_max_cost, 0.0F, 1.0F);
  config_.duplicate_lost_iou = std::clamp(config_.duplicate_lost_iou, 0.0F, 1.0F);
  config_.duplicate_lost_center_dist_norm = std::max(0.0F, config_.duplicate_lost_center_dist_norm);

  config_.appearance_alpha = std::clamp(config_.appearance_alpha, 0.0F, 0.999F);
  config_.appearance_gate = std::clamp(config_.appearance_gate, 0.0F, 1.5F);
  config_.motion_gate_thresh = std::max(1.0F, config_.motion_gate_thresh);
  config_.occlusion_shrink_ratio = std::clamp(config_.occlusion_shrink_ratio, 0.05F, 0.95F);
  config_.occlusion_suspect_area_ratio =
      std::clamp(config_.occlusion_suspect_area_ratio, config_.occlusion_shrink_ratio, 1.0F);
  config_.occlusion_neighbor_center_dist_norm =
      std::max(0.1F, config_.occlusion_neighbor_center_dist_norm);
  config_.occlusion_overlap_iou_min = std::clamp(config_.occlusion_overlap_iou_min, 0.0F, 1.0F);
  config_.occlusion_base_frames = std::min(config_.occlusion_base_frames, config_.occlusion_max_frames);

  return true;
}

bool MotTracker::Initialize(std::string *error) {
  if (config_.tracker_yaml_path.empty()) {
    if (error != nullptr) {
      *error = "tracker_yaml_path is empty.";
    }
    return false;
  }

  if (!std::filesystem::exists(config_.tracker_yaml_path)) {
    if (error != nullptr) {
      *error = "Tracker config not found: " + config_.tracker_yaml_path;
    }
    return false;
  }

  if (!ParseTrackerConfig(error)) {
    return false;
  }
  observability_ = MotTrackerObservability::CreateCsv(
      MotTrackerObservabilityConfig{config_.diag_assoc_enable, config_.diag_assoc_dir, config_.diag_frame_start,
                                    config_.diag_frame_end});

  if (!config_.with_reid || !config_.reid_enabled) {
    std::cerr << "[mot_tracker] reid is mandatory; override with_reid/reid_enabled to true" << std::endl;
  }
  config_.with_reid = true;
  config_.reid_enabled = true;

  appearance_features_ = AppearanceFeatureService(
      AppearanceFeatureService::Config{config_.reid_backend, config_.reid_model_path, config_.reid_input_width,
                                       config_.reid_input_height, config_.appearance_h_bins,
                                       config_.appearance_s_bins},
      AppearanceFeatureService::Profile::kTracker);
  if (!appearance_features_.Initialize(error)) {
    return false;
  }

  std::cout << "[mot_tracker] backend=tracker_core_v1"
            << " core_mode=" << config_.core_mode
            << " gmc_enabled=" << (config_.gmc_enabled ? "true" : "false")
            << " gmc_method=" << config_.gmc_method
            << " gmc_downscale=" << config_.gmc_downscale
            << " with_reid=" << (config_.with_reid ? "true" : "false")
            << " track_high=" << config_.track_high_thresh
            << " track_low=" << config_.track_low_thresh
            << " new_track=" << config_.new_track_thresh
            << " match_thresh=" << config_.match_thresh
            << " track_buffer=" << config_.track_buffer
            << " confirm_hits=" << config_.confirm_hits
            << " stage1_iou_min=" << config_.stage1_iou_min
            << " stage2_iou_min=" << config_.stage2_iou_min
            << " stage1_max_cost=" << config_.stage1_max_cost
            << " stage2_max_cost=" << config_.stage2_max_cost
            << " lost_recovery_max_cost=" << config_.lost_recovery_max_cost
            << " unconfirmed_max_cost=" << config_.unconfirmed_max_cost
            << " use_low_score_appearance_gate=" << (config_.use_low_score_appearance_gate ? "true" : "false")
            << " duplicate_lost_iou=" << config_.duplicate_lost_iou
            << " duplicate_lost_center_dist_norm=" << config_.duplicate_lost_center_dist_norm
            << " motion_gate=" << config_.motion_gate_thresh
            << " app_gate=" << config_.appearance_gate
            << " reid_backend=" << config_.reid_backend
            << " occ_base=" << config_.occlusion_base_frames
            << " occ_max=" << config_.occlusion_max_frames
            << std::endl;

  return true;
}

void MotTracker::InitializeKalman(TrackState *track, const cv::Rect2f &bbox) const {
  track->kf = cv::KalmanFilter(8, 4, 0, CV_32F);

  track->kf.transitionMatrix = (cv::Mat_<float>(8, 8) <<
      1, 0, 0, 0, 1, 0, 0, 0,
      0, 1, 0, 0, 0, 1, 0, 0,
      0, 0, 1, 0, 0, 0, 1, 0,
      0, 0, 0, 1, 0, 0, 0, 1,
      0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 1);

  track->kf.measurementMatrix = cv::Mat::zeros(4, 8, CV_32F);
  track->kf.measurementMatrix.at<float>(0, 0) = 1.0F;
  track->kf.measurementMatrix.at<float>(1, 1) = 1.0F;
  track->kf.measurementMatrix.at<float>(2, 2) = 1.0F;
  track->kf.measurementMatrix.at<float>(3, 3) = 1.0F;

  track->kf.processNoiseCov = cv::Mat::eye(8, 8, CV_32F);
  track->kf.processNoiseCov.at<float>(0, 0) = 1e-2F;
  track->kf.processNoiseCov.at<float>(1, 1) = 1e-2F;
  track->kf.processNoiseCov.at<float>(2, 2) = 1e-3F;
  track->kf.processNoiseCov.at<float>(3, 3) = 1e-2F;
  track->kf.processNoiseCov.at<float>(4, 4) = 2e-1F;
  track->kf.processNoiseCov.at<float>(5, 5) = 2e-1F;
  track->kf.processNoiseCov.at<float>(6, 6) = 1e-2F;
  track->kf.processNoiseCov.at<float>(7, 7) = 2e-1F;

  track->kf.measurementNoiseCov = cv::Mat::eye(4, 4, CV_32F);
  track->kf.measurementNoiseCov.at<float>(0, 0) = 1e-1F;
  track->kf.measurementNoiseCov.at<float>(1, 1) = 1e-1F;
  track->kf.measurementNoiseCov.at<float>(2, 2) = 5e-2F;
  track->kf.measurementNoiseCov.at<float>(3, 3) = 1e-1F;

  cv::setIdentity(track->kf.errorCovPost, cv::Scalar::all(1.0));

  const float cx = bbox.x + bbox.width * 0.5F;
  const float cy = bbox.y + bbox.height * 0.5F;
  const float a = bbox.width / std::max(1.0F, bbox.height);
  const float h = std::max(1.0F, bbox.height);

  track->kf.statePost = cv::Mat::zeros(8, 1, CV_32F);
  track->kf.statePost.at<float>(0) = cx;
  track->kf.statePost.at<float>(1) = cy;
  track->kf.statePost.at<float>(2) = a;
  track->kf.statePost.at<float>(3) = h;

  track->predicted_bbox = ClampRect(bbox);
}

cv::Rect2f MotTracker::PredictTrack(TrackState *track) const {
  const cv::Mat pred = track->kf.predict();
  const float cx = pred.at<float>(0);
  const float cy = pred.at<float>(1);
  const float a = std::max(0.05F, pred.at<float>(2));
  const float h = std::max(1.0F, pred.at<float>(3));
  const float w = std::max(1.0F, a * h);
  track->predicted_bbox = ClampRect(cv::Rect2f(cx - w * 0.5F, cy - h * 0.5F, w, h));
  return track->predicted_bbox;
}

void MotTracker::UpdateTrack(TrackState *track, const Detection &det) {
  const float cx = det.bbox.x + det.bbox.width * 0.5F;
  const float cy = det.bbox.y + det.bbox.height * 0.5F;
  const float a = det.bbox.width / std::max(1.0F, det.bbox.height);
  const float h = det.bbox.height;

  cv::Mat measurement = cv::Mat::zeros(4, 1, CV_32F);
  measurement.at<float>(0) = cx;
  measurement.at<float>(1) = cy;
  measurement.at<float>(2) = a;
  measurement.at<float>(3) = h;

  track->kf.correct(measurement);
  track->bbox = ClampRect(det.bbox);
  track->predicted_bbox = track->bbox;
  track->score = det.confidence;
  track->class_id = det.class_id;
  track->hits += 1;
  track->age += 1;
  track->time_since_update = 0;
  if (track->hits >= config_.confirm_hits) {
    track->is_confirmed = true;
  }
}

void MotTracker::UpdateTrackNewCore(TrackState *track, const Detection &det, const cv::Mat &appearance_feat) {
  const float cx = det.bbox.x + det.bbox.width * 0.5F;
  const float cy = det.bbox.y + det.bbox.height * 0.5F;
  const float a = det.bbox.width / std::max(1.0F, det.bbox.height);
  const float h = std::max(1.0F, det.bbox.height);

  cv::Mat measurement = cv::Mat::zeros(4, 1, CV_32F);
  measurement.at<float>(0) = cx;
  measurement.at<float>(1) = cy;
  measurement.at<float>(2) = a;
  measurement.at<float>(3) = h;

  track->kf.correct(measurement);

  const cv::Mat state = track->kf.statePost;
  const float out_cx = state.at<float>(0);
  const float out_cy = state.at<float>(1);
  const float out_a = std::max(0.05F, state.at<float>(2));
  const float out_h = std::max(1.0F, state.at<float>(3));
  const float out_w = std::max(1.0F, out_a * out_h);
  track->bbox = ClampRect(cv::Rect2f(out_cx - out_w * 0.5F, out_cy - out_h * 0.5F, out_w, out_h));
  track->predicted_bbox = track->bbox;

  track->score = det.confidence;
  track->class_id = det.class_id;
  track->hits += 1;
  track->age += 1;
  track->time_since_update = 0;
  track->life_state = TrackLifeState::kTracked;
  track->is_confirmed = (track->hits >= config_.confirm_hits);

  if (!appearance_feat.empty()) {
    if (!track->has_appearance || track->appearance_feat.empty()) {
      track->appearance_feat = appearance_feat.clone();
      track->has_appearance = true;
    } else {
      track->appearance_feat = config_.appearance_alpha * track->appearance_feat +
                               (1.0F - config_.appearance_alpha) * appearance_feat;
      NormalizeAppearanceFeature(&track->appearance_feat);
    }
  }
}

cv::Mat MotTracker::ExtractAppearanceFeature(const cv::Mat &frame, const cv::Rect2f &bbox) const {
  return appearance_features_.ExtractTrackerFeature(frame, bbox);
}

float MotTracker::AppearanceDistance(const TrackState &track, const cv::Mat &det_feat) const {
  if (!track.has_appearance || track.appearance_feat.empty() || det_feat.empty()) {
    return 0.0F;
  }
  if (UsingTrueReid()) {
    const float dot = static_cast<float>(track.appearance_feat.dot(det_feat));
    return std::clamp(1.0F - dot, 0.0F, 1.0F);
  }
  const float l1 = static_cast<float>(cv::norm(track.appearance_feat - det_feat, cv::NORM_L1));
  return std::clamp(0.5F * l1, 0.0F, 1.0F);
}

bool MotTracker::UsingTrueReid() const {
  return appearance_features_.UsesOnnx();
}

void MotTracker::NormalizeAppearanceFeature(cv::Mat *feature) const {
  if (feature == nullptr || feature->empty()) {
    return;
  }
  if (UsingTrueReid()) {
    const float norm = static_cast<float>(cv::norm(*feature, cv::NORM_L2));
    if (norm > 1e-6F) {
      *feature /= norm;
    }
    return;
  }
  const float sum_val = static_cast<float>(cv::sum(*feature)[0]);
  if (sum_val > 1e-6F) {
    *feature /= sum_val;
  }
}

float MotTracker::MahalanobisDistance(const TrackState &track, const Detection &det) const {
  if (track.kf.statePre.empty() || track.kf.errorCovPre.empty()) {
    return kLargeCost;
  }

  cv::Mat z = cv::Mat::zeros(4, 1, CV_32F);
  z.at<float>(0) = det.bbox.x + det.bbox.width * 0.5F;
  z.at<float>(1) = det.bbox.y + det.bbox.height * 0.5F;
  z.at<float>(2) = det.bbox.width / std::max(1.0F, det.bbox.height);
  z.at<float>(3) = std::max(1.0F, det.bbox.height);

  const cv::Mat H = track.kf.measurementMatrix;
  const cv::Mat R = track.kf.measurementNoiseCov;
  const cv::Mat x = track.kf.statePre;
  const cv::Mat P = track.kf.errorCovPre;

  cv::Mat residual = z - H * x;
  cv::Mat S = H * P * H.t() + R;

  cv::Mat S_inv;
  if (!cv::invert(S, S_inv, cv::DECOMP_SVD)) {
    return kLargeCost;
  }

  cv::Mat d2 = residual.t() * S_inv * residual;
  const float dist = d2.at<float>(0, 0);
  if (!std::isfinite(dist)) {
    return kLargeCost;
  }
  return dist;
}

float MotTracker::CenterDistanceNorm(const cv::Rect2f &a, const cv::Rect2f &b) const {
  return association::CenterDistanceNormByMaxArea(a, b);
}

float MotTracker::AssociationCost(const TrackState &track, const Detection &det, const cv::Mat &det_feat,
                                  const bool enable_appearance, const float iou_min, float *out_iou) const {
  const AssocTerms terms = ComputeAssociationTerms(track, det, det_feat, enable_appearance, iou_min);
  if (out_iou != nullptr) {
    *out_iou = terms.iou;
  }
  return terms.fused_cost;
}

AssociationEvidence MotTracker::EvidenceFromTerms(const AssocTerms &terms, const std::string &stage_name,
                                                  const bool low_score_detection,
                                                  const bool recovered_from_lost) const {
  AssociationEvidence evidence;
  evidence.stage = stage_name;
  evidence.fused_cost = terms.eligible ? terms.fused_cost : 1.0F;
  evidence.iou = terms.iou;
  evidence.motion_dist = terms.motion_dist;
  evidence.motion_term = terms.motion_term_norm;
  evidence.app_dist = terms.app_available ? terms.app_dist : 1.0F;
  evidence.appearance_used = terms.app_enabled && terms.app_available;
  evidence.low_score_detection = low_score_detection;
  evidence.recovered_from_lost = recovered_from_lost;
  evidence.passed_final_cost_gate = terms.eligible;
  evidence.reject_reason = terms.reject_reason;
  return evidence;
}

void MotTracker::SetAcceptedAssociation(TrackState *track, const AssocTerms &terms, const std::string &stage_name,
                                        const bool low_score_detection,
                                        const bool recovered_from_lost) const {
  if (track == nullptr) {
    return;
  }
  track->last_association = EvidenceFromTerms(terms, stage_name, low_score_detection, recovered_from_lost);
  track->last_association.passed_final_cost_gate = true;
  track->last_association.reject_reason.clear();
  track->low_score_update = low_score_detection;
  track->just_recovered = recovered_from_lost;
}

MotTracker::AssocTerms MotTracker::ComputeAssociationTerms(const TrackState &track, const Detection &det,
                                                           const cv::Mat &det_feat, const bool enable_appearance,
                                                           const float iou_min) const {
  AssocTerms out;
  out.measurement_cx = det.bbox.x + det.bbox.width * 0.5F;
  out.measurement_cy = det.bbox.y + det.bbox.height * 0.5F;
  out.measurement_a = det.bbox.width / std::max(1.0F, det.bbox.height);
  out.measurement_h = std::max(1.0F, det.bbox.height);

  if (!track.kf.statePre.empty()) {
    out.residual_cx = out.measurement_cx - StateVal(track.kf.statePre, 0);
    out.residual_cy = out.measurement_cy - StateVal(track.kf.statePre, 1);
    out.residual_a = out.measurement_a - StateVal(track.kf.statePre, 2);
    out.residual_h = out.measurement_h - StateVal(track.kf.statePre, 3);
  }
  for (int i = 0; i < 8; ++i) {
    out.error_cov_pre_diag[static_cast<std::size_t>(i)] = MatVal(track.kf.errorCovPre, i, i);
    out.error_cov_post_diag[static_cast<std::size_t>(i)] = MatVal(track.kf.errorCovPost, i, i);
    out.process_noise_q_diag[static_cast<std::size_t>(i)] = MatVal(track.kf.processNoiseCov, i, i);
  }
  for (int i = 0; i < 4; ++i) {
    out.measurement_noise_r_diag[static_cast<std::size_t>(i)] = MatVal(track.kf.measurementNoiseCov, i, i);
  }

  if (track.class_id != det.class_id) {
    out.reject_reason = "class_mismatch";
    return out;
  }

  const float iou = ComputeIoU(track.predicted_bbox, det.bbox);
  out.iou = iou;
  if (iou < iou_min) {
    out.reject_reason = "iou_below_min";
    return out;
  }

  float motion_dist = kLargeCost;
  cv::Mat z = cv::Mat::zeros(4, 1, CV_32F);
  z.at<float>(0) = out.measurement_cx;
  z.at<float>(1) = out.measurement_cy;
  z.at<float>(2) = out.measurement_a;
  z.at<float>(3) = out.measurement_h;
  const cv::Mat H = track.kf.measurementMatrix;
  const cv::Mat R = track.kf.measurementNoiseCov;
  const cv::Mat x = track.kf.statePre;
  const cv::Mat P = track.kf.errorCovPre;
  if (!H.empty() && !R.empty() && !x.empty() && !P.empty()) {
    cv::Mat residual = z - H * x;
    cv::Mat S = H * P * H.t() + R;
    cv::Mat S_inv;
    if (cv::invert(S, S_inv, cv::DECOMP_SVD)) {
      cv::Mat d2 = residual.t() * S_inv * residual;
      const float dist = d2.at<float>(0, 0);
      if (std::isfinite(dist)) {
        motion_dist = dist;
      }
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          out.innovation_cov_s[static_cast<std::size_t>(r * 4 + c)] = MatVal(S, r, c);
        }
      }
      cv::Mat K = P * H.t() * S_inv;  // 8x4
      for (int r = 0; r < 8; ++r) {
        for (int c = 0; c < 4; ++c) {
          out.kalman_gain_k[static_cast<std::size_t>(r * 4 + c)] = MatVal(K, r, c);
        }
      }
    }
  }
  out.motion_dist = motion_dist;
  out.gate_dist = motion_dist;
  out.assoc_motion_dist = motion_dist;
  const bool motion_ok = std::isfinite(motion_dist) && motion_dist < kLargeCost * 0.5F;
  out.motion_ok = motion_ok;
  out.motion_gate_effective_thresh = config_.motion_gate_thresh * 8.0F;
  out.iou_guard_pass = (iou >= 0.6F);
  out.motion_gate_pass = true;
  if (motion_ok && motion_dist > out.motion_gate_effective_thresh && !out.iou_guard_pass) {
    out.motion_gate_pass = false;
    out.reject_reason = "motion_gate_reject";
    return out;
  }

  float app_weight = 0.0F;
  float app_term = 0.0F;
  out.app_enabled = enable_appearance;
  out.app_available = (track.has_appearance && !det_feat.empty());
  if (enable_appearance && track.has_appearance && !det_feat.empty()) {
    const float app_dist = AppearanceDistance(track, det_feat);
    out.app_dist = app_dist;
    if (app_dist > config_.appearance_gate) {
      out.app_gate_pass = false;
      out.reject_reason = "app_gate_reject";
      return out;
    }
    app_weight = config_.assoc_app_weight;
    app_term = std::clamp(app_dist, 0.0F, 1.0F);
  }

  const float iou_weight = config_.assoc_iou_weight;
  const float motion_weight = config_.assoc_motion_weight;
  const float total_w = std::max(1e-6F, iou_weight + motion_weight + app_weight);

  const float iou_term = 1.0F - iou;
  const float motion_term = motion_ok
                                ? std::clamp(motion_dist / (config_.motion_gate_thresh * 4.0F), 0.0F, 1.0F)
                                : 1.0F;
  out.motion_term_norm = motion_term;
  const float cost = (iou_weight * iou_term + motion_weight * motion_term + app_weight * app_term) / total_w;
  out.fused_cost = cost;
  out.eligible = (cost < kLargeCost * 0.5F);
  return out;
}

bool MotTracker::IsOcclusionCandidate(const int track_idx) const {
  if (track_idx < 0 || track_idx >= static_cast<int>(tracks_.size())) {
    return false;
  }
  const auto &track = tracks_[static_cast<std::size_t>(track_idx)];
  if (track.life_state != TrackLifeState::kTracked || track.time_since_update > 0 ||
      track.class_id != ClassId::kPerson) {
    return false;
  }
  const float current_area = std::max(1.0F, track.bbox.area());
  const float stable_area = std::max(current_area, track.stable_area);
  if (stable_area <= 1.0F) {
    return false;
  }
  const float area_ratio = current_area / stable_area;
  if (area_ratio > config_.occlusion_shrink_ratio) {
    return false;
  }

  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (static_cast<int>(i) == track_idx) {
      continue;
    }
    const auto &other = tracks_[i];
    if (other.life_state != TrackLifeState::kTracked || other.time_since_update > 0 ||
        other.class_id != ClassId::kPerson) {
      continue;
    }
    const float iou = ComputeIoU(track.bbox, other.bbox);
    const float center_norm = CenterDistanceNorm(track.bbox, other.bbox);
    if (iou >= config_.occlusion_overlap_iou_min || center_norm <= config_.occlusion_neighbor_center_dist_norm) {
      return true;
    }
  }
  return false;
}

void MotTracker::UpdateOcclusionProtection() {
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    auto &track = tracks_[i];
    if (track.class_id != ClassId::kPerson || track.life_state != TrackLifeState::kTracked ||
        track.time_since_update > 0) {
      if (track.occlusion_protect_remaining > 0) {
        track.occlusion_protect_remaining -= 1;
      }
      continue;
    }

    const float current_area = std::max(1.0F, track.bbox.area());
    if (track.stable_area <= 0.0F) {
      track.stable_area = current_area;
    }

    const bool candidate = IsOcclusionCandidate(static_cast<int>(i));
    if (candidate) {
      track.occlusion_candidate_streak =
          std::min(config_.occlusion_enter_min_frames, track.occlusion_candidate_streak + 1);
      track.occlusion_clear_streak = 0;
      if (track.occlusion_candidate_streak >= config_.occlusion_enter_min_frames) {
        track.occlusion_suspect =
            (current_area <= std::max(1.0F, track.stable_area * config_.occlusion_suspect_area_ratio));
        if (track.occlusion_suspect) {
          if (track.occlusion_protect_remaining <= 0) {
            track.occlusion_protect_remaining = config_.occlusion_base_frames;
          } else {
            track.occlusion_protect_remaining =
                std::min(config_.occlusion_max_frames,
                         track.occlusion_protect_remaining + config_.occlusion_extend_step);
          }
        }
      }
      continue;
    }

    track.occlusion_candidate_streak = std::max(0, track.occlusion_candidate_streak - 1);
    if (track.occlusion_suspect) {
      track.occlusion_clear_streak += 1;
      if (track.occlusion_protect_remaining > 0) {
        track.occlusion_protect_remaining -= 1;
      }
      if (track.occlusion_clear_streak >= config_.occlusion_release_clear_frames ||
          track.occlusion_protect_remaining <= 0) {
        track.occlusion_suspect = false;
        track.occlusion_clear_streak = 0;
        track.occlusion_protect_remaining = 0;
        track.stable_area = std::max(track.stable_area, current_area);
      }
      continue;
    }

    track.occlusion_clear_streak = 0;
    track.stable_area = std::max(current_area, 0.9F * track.stable_area + 0.1F * current_area);
  }
}

void MotTracker::MirrorTrackedHypotheses(const std::vector<Track> &tracks) {
  last_tracklet_hypotheses_.clear();
  last_tracklet_hypotheses_.reserve(tracks.size() + pending_suppressed_new_track_hypotheses_.size() +
                                    pending_duplicate_output_hypotheses_.size());
  for (const auto &track : tracks) {
    TrackletHypothesis hypothesis;
    hypothesis.raw_track_id = track.id;
    hypothesis.class_id = track.class_id;
    hypothesis.confidence = track.confidence;
    hypothesis.bbox = track.bbox;
    hypothesis.status = TrackletHypothesisStatus::kTracked;
    hypothesis.candidate_reason = "final_track_output";
    hypothesis.related_raw_track_id = std::nullopt;
    hypothesis.association = track.association;
    last_tracklet_hypotheses_.push_back(hypothesis);
  }
  last_tracklet_hypotheses_.insert(last_tracklet_hypotheses_.end(), pending_suppressed_new_track_hypotheses_.begin(),
                                   pending_suppressed_new_track_hypotheses_.end());
  last_tracklet_hypotheses_.insert(last_tracklet_hypotheses_.end(), pending_duplicate_output_hypotheses_.begin(),
                                   pending_duplicate_output_hypotheses_.end());
  pending_suppressed_new_track_hypotheses_.clear();
  pending_duplicate_output_hypotheses_.clear();
}

void MotTracker::AppendSuppressedNewTrackHypothesis(const Detection &det,
                                                    const NewTrackSuppression &suppression) {
  TrackletHypothesis hypothesis;
  hypothesis.raw_track_id = -1;
  hypothesis.class_id = det.class_id;
  hypothesis.confidence = det.confidence;
  hypothesis.bbox = ClampRect(det.bbox);
  hypothesis.status = TrackletHypothesisStatus::kSuppressedDuplicateCandidate;
  hypothesis.candidate_reason = suppression.reason;
  if (suppression.related_raw_track_id > 0) {
    hypothesis.related_raw_track_id = suppression.related_raw_track_id;
  }
  hypothesis.association.stage = "new_track_suppressed";
  hypothesis.association.reject_reason = suppression.reason;
  pending_suppressed_new_track_hypotheses_.push_back(hypothesis);
}

void MotTracker::AppendDuplicateOutputHypothesis(const TrackState &track,
                                                 const DuplicateOutputSuppression &suppression) {
  TrackletHypothesis hypothesis;
  hypothesis.raw_track_id = track.id;
  hypothesis.class_id = track.class_id;
  hypothesis.confidence = track.score;
  hypothesis.bbox = ClampRect(track.bbox);
  hypothesis.status = TrackletHypothesisStatus::kSuppressedDuplicateCandidate;
  hypothesis.candidate_reason = suppression.reason;
  if (suppression.related_raw_track_id > 0) {
    hypothesis.related_raw_track_id = suppression.related_raw_track_id;
  }
  hypothesis.association = track.last_association;
  hypothesis.association.reject_reason = suppression.reason;
  pending_duplicate_output_hypotheses_.push_back(hypothesis);
}

bool MotTracker::DiagFrameEnabled() const {
  return observability_ != nullptr && observability_->EnabledForFrame(frame_id_);
}

void MotTracker::DiagWriteTracks(const std::vector<Detection> &, const std::vector<Detection> &) {
  if (!DiagFrameEnabled()) {
    return;
  }
  std::vector<MotTrackerTrackObservation> observations;
  observations.reserve(tracks_.size());
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    const auto &t = tracks_[i];
    const bool has_snapshot = (i < diag_snapshots_.size() && diag_snapshots_[i].valid);
    const TrackDiagSnapshot empty_snapshot;
    const auto &s = has_snapshot ? diag_snapshots_[i] : empty_snapshot;
    const cv::Rect2f pre_pred = has_snapshot ? s.pre_gmc_pred_bbox : t.predicted_bbox;
    const cv::Rect2f pre_last = has_snapshot ? s.pre_gmc_bbox : t.bbox;
    const cv::Rect2f post_pred = has_snapshot ? s.post_gmc_pred_bbox : t.predicted_bbox;
    const cv::Rect2f post_last = has_snapshot ? s.post_gmc_bbox : t.bbox;
    const cv::Mat pre_state_post = has_snapshot ? s.pre_state_post : t.kf.statePost;
    const cv::Mat post_predict_state_pre = has_snapshot ? s.post_predict_state_pre : t.kf.statePre;
    const cv::Mat post_predict_state_post = has_snapshot ? s.post_predict_state_post : t.kf.statePost;
    const cv::Mat post_gmc_state_pre = has_snapshot ? s.post_gmc_state_pre : t.kf.statePre;
    const cv::Mat post_gmc_state_post = has_snapshot ? s.post_gmc_state_post : t.kf.statePost;
    const cv::Mat pre_error_cov_post = has_snapshot ? s.pre_error_cov_post : t.kf.errorCovPost;
    const cv::Mat post_predict_error_cov_pre =
        has_snapshot ? s.post_predict_error_cov_pre : t.kf.errorCovPre;
    const cv::Mat post_predict_error_cov_post =
        has_snapshot ? s.post_predict_error_cov_post : t.kf.errorCovPost;
    const cv::Mat post_gmc_error_cov_pre = has_snapshot ? s.post_gmc_error_cov_pre : t.kf.errorCovPre;
    const cv::Mat post_gmc_error_cov_post = has_snapshot ? s.post_gmc_error_cov_post : t.kf.errorCovPost;

    MotTrackerTrackObservation observation;
    observation.track_idx = static_cast<int>(i);
    observation.track_id = t.id;
    observation.class_id = t.class_id;
    observation.state_code = static_cast<int>(t.life_state);
    observation.is_confirmed = t.is_confirmed;
    observation.hits = t.hits;
    observation.age = t.age;
    observation.time_since_update = t.time_since_update;
    observation.has_appearance = t.has_appearance;
    observation.predicted_bbox = t.predicted_bbox;
    observation.bbox = t.bbox;
    observation.pre_gmc_pred_bbox = pre_pred;
    observation.pre_gmc_bbox = pre_last;
    observation.post_gmc_pred_bbox = post_pred;
    observation.post_gmc_bbox = post_last;
    observation.pre_state_post = pre_state_post;
    observation.post_predict_state_pre = post_predict_state_pre;
    observation.post_predict_state_post = post_predict_state_post;
    observation.post_gmc_state_pre = post_gmc_state_pre;
    observation.post_gmc_state_post = post_gmc_state_post;
    observation.pre_error_cov_post = pre_error_cov_post;
    observation.post_predict_error_cov_pre = post_predict_error_cov_pre;
    observation.post_predict_error_cov_post = post_predict_error_cov_post;
    observation.post_gmc_error_cov_pre = post_gmc_error_cov_pre;
    observation.post_gmc_error_cov_post = post_gmc_error_cov_post;
    observations.push_back(std::move(observation));
  }
  observability_->WriteTracks(frame_id_, observations);
}

void MotTracker::DiagWriteDetections(const std::vector<Detection> &det_high, const std::vector<Detection> &det_low) const {
  if (!DiagFrameEnabled()) {
    return;
  }
  std::vector<MotTrackerDetectionObservation> observations;
  observations.reserve(det_high.size() + det_low.size());
  for (std::size_t i = 0; i < det_high.size(); ++i) {
    const auto &d = det_high[i];
    observations.push_back(MotTrackerDetectionObservation{"high", static_cast<int>(i), static_cast<int>(i),
                                                          d.class_id, d.confidence, d.bbox});
  }
  for (std::size_t i = 0; i < det_low.size(); ++i) {
    const auto &d = det_low[i];
    observations.push_back(MotTrackerDetectionObservation{"low", static_cast<int>(i), static_cast<int>(i),
                                                          d.class_id, d.confidence, d.bbox});
  }
  observability_->WriteDetections(frame_id_, observations);
}

void MotTracker::DiagWriteGmc(const bool gmc_ok, const cv::Mat &gmc_warp) const {
  if (!DiagFrameEnabled()) {
    return;
  }
  observability_->WriteGmc(frame_id_, MotTrackerGmcObservation{gmc_ok, gmc_warp});
}

void MotTracker::DiagWritePairs(const std::string &stage_name, const int track_idx, const int det_local_idx,
                                const int det_src_idx, const AssocTerms &terms, const bool selected) const {
  if (!DiagFrameEnabled()) {
    return;
  }
  if (track_idx < 0 || track_idx >= static_cast<int>(tracks_.size())) {
    return;
  }
  const auto &t = tracks_[track_idx];
  const bool has_snapshot = (track_idx < static_cast<int>(diag_snapshots_.size()) && diag_snapshots_[track_idx].valid);
  const TrackDiagSnapshot empty_snapshot;
  const auto &s = has_snapshot ? diag_snapshots_[track_idx] : empty_snapshot;
  const cv::Rect2f pre_pred = has_snapshot ? s.pre_gmc_pred_bbox : t.predicted_bbox;
  const cv::Rect2f post_pred = has_snapshot ? s.post_gmc_pred_bbox : t.predicted_bbox;
  const cv::Mat pre_state_post = has_snapshot ? s.pre_state_post : t.kf.statePost;
  const cv::Mat post_predict_state_pre = has_snapshot ? s.post_predict_state_pre : t.kf.statePre;
  const cv::Mat post_gmc_state_pre = has_snapshot ? s.post_gmc_state_pre : t.kf.statePre;

  MotTrackerPairObservation observation;
  observation.stage_name = stage_name;
  observation.track_idx = track_idx;
  observation.track_id = t.id;
  observation.track_state_code = static_cast<int>(t.life_state);
  observation.det_local_idx = det_local_idx;
  observation.det_src_idx = det_src_idx;
  observation.selected = selected;
  observation.pre_gmc_pred_bbox = pre_pred;
  observation.post_gmc_pred_bbox = post_pred;
  observation.pre_state_post = pre_state_post;
  observation.post_predict_state_pre = post_predict_state_pre;
  observation.post_gmc_state_pre = post_gmc_state_pre;
  observation.terms.iou = terms.iou;
  observation.terms.motion_dist = terms.motion_dist;
  observation.terms.gate_dist = terms.gate_dist;
  observation.terms.assoc_motion_dist = terms.assoc_motion_dist;
  observation.terms.motion_term_norm = terms.motion_term_norm;
  observation.terms.motion_ok = terms.motion_ok;
  observation.terms.motion_gate_pass = terms.motion_gate_pass;
  observation.terms.app_enabled = terms.app_enabled;
  observation.terms.app_available = terms.app_available;
  observation.terms.app_dist = terms.app_dist;
  observation.terms.app_gate_pass = terms.app_gate_pass;
  observation.terms.motion_gate_effective_thresh = terms.motion_gate_effective_thresh;
  observation.terms.iou_guard_pass = terms.iou_guard_pass;
  observation.terms.measurement_cx = terms.measurement_cx;
  observation.terms.measurement_cy = terms.measurement_cy;
  observation.terms.measurement_a = terms.measurement_a;
  observation.terms.measurement_h = terms.measurement_h;
  observation.terms.residual_cx = terms.residual_cx;
  observation.terms.residual_cy = terms.residual_cy;
  observation.terms.residual_a = terms.residual_a;
  observation.terms.residual_h = terms.residual_h;
  observation.terms.innovation_cov_s = terms.innovation_cov_s;
  observation.terms.kalman_gain_k = terms.kalman_gain_k;
  observation.terms.error_cov_pre_diag = terms.error_cov_pre_diag;
  observation.terms.error_cov_post_diag = terms.error_cov_post_diag;
  observation.terms.process_noise_q_diag = terms.process_noise_q_diag;
  observation.terms.measurement_noise_r_diag = terms.measurement_noise_r_diag;
  observation.terms.fused_cost = terms.fused_cost;
  observation.terms.eligible = terms.eligible;
  observation.terms.reject_reason = terms.reject_reason;
  observability_->WritePair(frame_id_, observation);
}

float MotTracker::ComputeIoU(const cv::Rect2f &a, const cv::Rect2f &b) const {
  return association::BBoxIoU(a, b);
}

MotTracker::NewTrackSuppression MotTracker::ShouldSuppressNewTrack(const Detection &det) const {
  for (const auto &track : tracks_) {
    if (track.life_state == TrackLifeState::kRemoved) {
      continue;
    }
    if (track.life_state == TrackLifeState::kTracked && track.time_since_update != 0) {
      continue;
    }
    if (track.life_state == TrackLifeState::kLost && track.time_since_update > config_.track_buffer) {
      continue;
    }
    if (track.class_id != det.class_id) {
      continue;
    }

    const cv::Rect2f duplicate_ref_bbox =
        track.life_state == TrackLifeState::kLost ? track.predicted_bbox : track.bbox;
    const float iou = ComputeIoU(duplicate_ref_bbox, det.bbox);
    if (track.life_state == TrackLifeState::kTracked && iou >= kDuplicateTrackedSpawnIou) {
      const float center_dist_norm = CenterDistanceNorm(duplicate_ref_bbox, det.bbox);
      const float center_suppress_thresh =
          iou < kDuplicateTrackedLowOverlapIou ? kDuplicateTrackedLowOverlapCenterDistNorm
                                               : kDuplicateTrackedCenterDistNorm;
      if (center_dist_norm <= center_suppress_thresh) {
        NewTrackSuppression suppression;
        suppression.suppressed = true;
        suppression.reason = "new_track_suppressed_duplicate_tracked";
        suppression.related_raw_track_id = track.id;
        return suppression;
      }
    }
    const bool recently_lost = track.time_since_update <= kLostCenterDuplicateSuppressFrames;
    if (track.life_state == TrackLifeState::kLost &&
        (iou >= config_.duplicate_lost_iou ||
         (recently_lost && CenterDistanceNorm(duplicate_ref_bbox, det.bbox) <= config_.duplicate_lost_center_dist_norm))) {
      NewTrackSuppression suppression;
      suppression.suppressed = true;
      suppression.reason = "new_track_suppressed_duplicate_lost";
      suppression.related_raw_track_id = track.id;
      return suppression;
    }
  }
  return NewTrackSuppression{};
}

bool MotTracker::IsDuplicateOutputTrack(const TrackState &candidate, const TrackState &other) const {
  if (candidate.id == other.id || candidate.class_id != other.class_id || candidate.class_id != ClassId::kPerson) {
    return false;
  }
  if (other.life_state != TrackLifeState::kTracked || other.time_since_update > 0) {
    return false;
  }
  if (candidate.score > other.score) {
    return false;
  }
  const float candidate_area = std::max(1.0F, candidate.bbox.area());
  const float other_area = std::max(1.0F, other.bbox.area());
  if (candidate_area > other_area * kDuplicateOutputMaxAreaRatio) {
    return false;
  }
  const float iou = ComputeIoU(candidate.bbox, other.bbox);
  const float center_dist_norm = CenterDistanceNorm(candidate.bbox, other.bbox);
  return iou >= kDuplicateOutputIou && center_dist_norm <= kDuplicateOutputCenterDistNorm;
}

MotTracker::DuplicateOutputSuppression MotTracker::FindDuplicateOutputSuppression(
    const std::size_t candidate_index) const {
  if (candidate_index >= tracks_.size()) {
    return DuplicateOutputSuppression{};
  }
  const auto &candidate = tracks_[candidate_index];
  for (std::size_t j = 0; j < tracks_.size(); ++j) {
    if (candidate_index == j) {
      continue;
    }
    if (IsDuplicateOutputTrack(candidate, tracks_[j])) {
      DuplicateOutputSuppression suppression;
      suppression.suppressed = true;
      suppression.reason = "duplicate_output_hidden";
      suppression.related_raw_track_id = tracks_[j].id;
      return suppression;
    }
  }
  return DuplicateOutputSuppression{};
}

std::vector<std::pair<int, int>> MotTracker::MatchByHungarian(const std::vector<int> &track_indices,
                                                              const std::vector<Detection> &detections,
                                                              const std::vector<cv::Mat> &det_feats,
                                                              const bool enable_appearance,
                                                              const float iou_min,
                                                              const float stage_max_cost,
                                                              const std::string &stage_name,
                                                              const std::vector<int> *det_src_indices) const {
  std::vector<std::pair<int, int>> matches;
  if (track_indices.empty() || detections.empty()) {
    return matches;
  }

  std::vector<std::vector<float>> cost(track_indices.size(), std::vector<float>(detections.size(), kLargeCost));
  std::vector<std::vector<AssocTerms>> terms_mat(track_indices.size(), std::vector<AssocTerms>(detections.size()));
  for (std::size_t r = 0; r < track_indices.size(); ++r) {
    const int t_idx = track_indices[r];
    if (t_idx < 0 || t_idx >= static_cast<int>(tracks_.size())) {
      continue;
    }
    for (std::size_t c = 0; c < detections.size(); ++c) {
      const cv::Mat &feat = (c < det_feats.size()) ? det_feats[c] : cv::Mat();
      terms_mat[r][c] = ComputeAssociationTerms(tracks_[t_idx], detections[c], feat, enable_appearance, iou_min);
      cost[r][c] = terms_mat[r][c].fused_cost;
    }
  }

  const std::vector<int> assign = HungarianAssign(cost);
  std::vector<bool> used_det(detections.size(), false);
  for (std::size_t r = 0; r < assign.size(); ++r) {
    const int c = assign[r];
    if (c < 0 || c >= static_cast<int>(detections.size())) {
      continue;
    }
    if (used_det[c]) {
      continue;
    }
    if (cost[r][c] >= kLargeCost * 0.5F) {
      continue;
    }
    if (cost[r][c] > stage_max_cost) {
      terms_mat[r][static_cast<std::size_t>(c)].eligible = false;
      terms_mat[r][static_cast<std::size_t>(c)].reject_reason = "stage_max_cost_reject";
      continue;
    }
    terms_mat[r][static_cast<std::size_t>(c)].eligible = true;
    used_det[c] = true;
    matches.emplace_back(track_indices[r], c);
  }

  if (DiagFrameEnabled()) {
    std::vector<bool> selected_row_col(track_indices.size() * std::max<std::size_t>(1, detections.size()), false);
    for (const auto &m : matches) {
      auto it = std::find(track_indices.begin(), track_indices.end(), m.first);
      if (it == track_indices.end()) {
        continue;
      }
      const std::size_t row = static_cast<std::size_t>(std::distance(track_indices.begin(), it));
      if (m.second >= 0 && m.second < static_cast<int>(detections.size())) {
        selected_row_col[row * detections.size() + static_cast<std::size_t>(m.second)] = true;
      }
    }
    for (std::size_t r = 0; r < track_indices.size(); ++r) {
      for (std::size_t c = 0; c < detections.size(); ++c) {
        const bool selected = selected_row_col[r * detections.size() + c];
        const int det_src_idx =
            (det_src_indices != nullptr && c < det_src_indices->size()) ? (*det_src_indices)[c] : static_cast<int>(c);
        DiagWritePairs(stage_name, track_indices[r], static_cast<int>(c), det_src_idx, terms_mat[r][c], selected);
      }
    }
  }

  return matches;
}

std::vector<std::pair<int, int>> MotTracker::GreedyMatch(const std::vector<int> &track_indices,
                                                         const std::vector<Detection> &detections,
                                                         const float min_iou) const {
  struct Candidate {
    int track_idx{-1};
    int det_idx{-1};
    float iou{0.0F};
  };

  std::vector<Candidate> candidates;
  candidates.reserve(track_indices.size() * std::max<std::size_t>(1, detections.size()));

  for (const int t_idx : track_indices) {
    if (t_idx < 0 || t_idx >= static_cast<int>(tracks_.size())) {
      continue;
    }
    for (std::size_t d = 0; d < detections.size(); ++d) {
      if (tracks_[t_idx].class_id != detections[d].class_id) {
        continue;
      }
      const float iou = ComputeIoU(tracks_[t_idx].bbox, detections[d].bbox);
      if (iou >= min_iou) {
        candidates.push_back(Candidate{t_idx, static_cast<int>(d), iou});
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate &l, const Candidate &r) {
    return l.iou > r.iou;
  });

  std::vector<bool> used_tracks(tracks_.size(), false);
  std::vector<bool> used_dets(detections.size(), false);
  std::vector<std::pair<int, int>> matches;

  for (const auto &c : candidates) {
    if (used_tracks[c.track_idx] || used_dets[c.det_idx]) {
      continue;
    }
    used_tracks[c.track_idx] = true;
    used_dets[c.det_idx] = true;
    matches.emplace_back(c.track_idx, c.det_idx);
  }

  return matches;
}

void MotTracker::ApplyGmc(const cv::Mat &frame, std::vector<TrackState> *tracks, bool *out_gmc_ok, cv::Mat *out_warp) {
  if (out_gmc_ok != nullptr) {
    *out_gmc_ok = false;
  }
  if (out_warp != nullptr) {
    *out_warp = cv::Mat::eye(2, 3, CV_32F);
  }
  if (!config_.gmc_enabled || frame.empty()) {
    return;
  }

  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  if (prev_gray_.empty()) {
    prev_gray_ = gray;
    return;
  }

  const int downscale = std::max(1, config_.gmc_downscale);
  const std::string method = ToLower(config_.gmc_method);

  cv::Mat prev_proc = prev_gray_;
  cv::Mat gray_proc = gray;
  if (downscale > 1) {
    const cv::Size ds_size(std::max(1, gray.cols / downscale), std::max(1, gray.rows / downscale));
    cv::resize(prev_gray_, prev_proc, ds_size, 0.0, 0.0, cv::INTER_AREA);
    cv::resize(gray, gray_proc, ds_size, 0.0, 0.0, cv::INTER_AREA);
  }

  cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
  bool gmc_ok = false;

  if (method == "ecc") {
    try {
      cv::findTransformECC(prev_proc, gray_proc, warp, cv::MOTION_EUCLIDEAN,
                           cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-4));
      gmc_ok = true;
    } catch (...) {
      gmc_ok = false;
    }
  } else if (method == "sparseoptflow") {
    std::vector<cv::Point2f> prev_pts;
    cv::goodFeaturesToTrack(prev_proc, prev_pts, 300, 0.01, 8.0);
    if (prev_pts.size() >= 8) {
      std::vector<cv::Point2f> curr_pts;
      std::vector<unsigned char> status;
      std::vector<float> err;
      cv::calcOpticalFlowPyrLK(prev_proc, gray_proc, prev_pts, curr_pts, status, err);

      std::vector<cv::Point2f> src;
      std::vector<cv::Point2f> dst;
      src.reserve(prev_pts.size());
      dst.reserve(prev_pts.size());
      for (std::size_t i = 0; i < prev_pts.size(); ++i) {
        if (i >= status.size() || status[i] == 0) {
          continue;
        }
        src.emplace_back(prev_pts[i].x * static_cast<float>(downscale),
                         prev_pts[i].y * static_cast<float>(downscale));
        dst.emplace_back(curr_pts[i].x * static_cast<float>(downscale),
                         curr_pts[i].y * static_cast<float>(downscale));
      }
      if (src.size() >= 6) {
        cv::Mat inliers;
        cv::Mat affine = cv::estimateAffinePartial2D(src, dst, inliers, cv::RANSAC, 3.0);
        if (!affine.empty() && affine.rows == 2 && affine.cols == 3) {
          affine.convertTo(warp, CV_32F);
          gmc_ok = true;
        }
      }
    }
  }

  if (gmc_ok) {
    const float a00 = warp.at<float>(0, 0);
    const float a01 = warp.at<float>(0, 1);
    const float a10 = warp.at<float>(1, 0);
    const float a11 = warp.at<float>(1, 1);
    const float tx = warp.at<float>(0, 2);
    const float ty = warp.at<float>(1, 2);

    auto warp_box = [&](const cv::Rect2f &b) {
      const cv::Point2f p1(b.x, b.y);
      const cv::Point2f p2(b.x + b.width, b.y + b.height);
      const float x1 = a00 * p1.x + a01 * p1.y + tx;
      const float y1 = a10 * p1.x + a11 * p1.y + ty;
      const float x2 = a00 * p2.x + a01 * p2.y + tx;
      const float y2 = a10 * p2.x + a11 * p2.y + ty;
      return ClampRect(cv::Rect2f(std::min(x1, x2), std::min(y1, y2), std::abs(x2 - x1), std::abs(y2 - y1)));
    };

    for (auto &t : *tracks) {
      t.predicted_bbox = warp_box(t.predicted_bbox);
      t.bbox = warp_box(t.bbox);

      if (!t.kf.statePost.empty()) {
        auto &sp = t.kf.statePost;
        const float cx = sp.at<float>(0);
        const float cy = sp.at<float>(1);
        const float vx = sp.at<float>(4);
        const float vy = sp.at<float>(5);
        sp.at<float>(0) = a00 * cx + a01 * cy + tx;
        sp.at<float>(1) = a10 * cx + a11 * cy + ty;
        sp.at<float>(4) = a00 * vx + a01 * vy;
        sp.at<float>(5) = a10 * vx + a11 * vy;
      }
      if (!t.kf.statePre.empty()) {
        auto &sp = t.kf.statePre;
        const float cx = sp.at<float>(0);
        const float cy = sp.at<float>(1);
        const float vx = sp.at<float>(4);
        const float vy = sp.at<float>(5);
        sp.at<float>(0) = a00 * cx + a01 * cy + tx;
        sp.at<float>(1) = a10 * cx + a11 * cy + ty;
        sp.at<float>(4) = a00 * vx + a01 * vy;
        sp.at<float>(5) = a10 * vx + a11 * vy;
      }
    }
  }

  if (out_gmc_ok != nullptr) {
    *out_gmc_ok = gmc_ok;
  }
  if (out_warp != nullptr) {
    *out_warp = warp.clone();
  }
  prev_gray_ = gray;
}

std::vector<Track> MotTracker::UpdateOldMinimal(const std::vector<Detection> &detections, const cv::Mat &frame) {
  for (auto &track : tracks_) {
    if (track.life_state == TrackLifeState::kRemoved) {
      continue;
    }
    track.bbox = PredictTrack(&track);
    track.time_since_update += 1;
    track.age += 1;
  }

  bool gmc_ok = false;
  cv::Mat gmc_warp;
  ApplyGmc(frame, &tracks_, &gmc_ok, &gmc_warp);
  (void)gmc_ok;
  (void)gmc_warp;

  std::vector<Detection> det_high;
  std::vector<Detection> det_low;
  det_high.reserve(detections.size());
  det_low.reserve(detections.size());

  for (const auto &det : detections) {
    if (det.confidence >= config_.track_high_thresh) {
      det_high.push_back(det);
    } else if (det.confidence >= config_.track_low_thresh) {
      det_low.push_back(det);
    }
  }

  std::vector<int> active_track_indices;
  active_track_indices.reserve(tracks_.size());
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (tracks_[i].life_state != TrackLifeState::kRemoved) {
      active_track_indices.push_back(static_cast<int>(i));
    }
  }

  const float min_iou = std::max(0.01F, 1.0F - config_.match_thresh);
  const auto high_matches = GreedyMatch(active_track_indices, det_high, min_iou);

  std::vector<bool> matched_tracks(tracks_.size(), false);
  std::vector<bool> matched_high(det_high.size(), false);

  for (const auto &m : high_matches) {
    matched_tracks[m.first] = true;
    matched_high[m.second] = true;
    UpdateTrack(&tracks_[m.first], det_high[m.second]);
    tracks_[m.first].life_state = TrackLifeState::kTracked;
  }

  std::vector<int> unmatched_tracks;
  unmatched_tracks.reserve(tracks_.size());
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (tracks_[i].life_state == TrackLifeState::kRemoved) {
      continue;
    }
    if (!matched_tracks[i]) {
      unmatched_tracks.push_back(static_cast<int>(i));
    }
  }

  const auto low_matches = GreedyMatch(unmatched_tracks, det_low, min_iou);
  std::vector<bool> matched_low(det_low.size(), false);

  for (const auto &m : low_matches) {
    matched_tracks[m.first] = true;
    matched_low[m.second] = true;
    UpdateTrack(&tracks_[m.first], det_low[m.second]);
    tracks_[m.first].life_state = TrackLifeState::kTracked;
  }

  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (tracks_[i].life_state == TrackLifeState::kRemoved) {
      continue;
    }
    if (!matched_tracks[i]) {
      tracks_[i].life_state = TrackLifeState::kLost;
    }
  }

  for (std::size_t d = 0; d < det_high.size(); ++d) {
    if (matched_high[d] || det_high[d].confidence < config_.new_track_thresh) {
      continue;
    }
    const auto suppression = ShouldSuppressNewTrack(det_high[d]);
    if (suppression.suppressed) {
      AppendSuppressedNewTrackHypothesis(det_high[d], suppression);
      continue;
    }

    TrackState t;
    t.id = next_track_id_++;
    t.class_id = det_high[d].class_id;
    t.score = det_high[d].confidence;
    t.bbox = ClampRect(det_high[d].bbox);
    t.hits = 1;
    t.age = 1;
    t.time_since_update = 0;
    t.is_confirmed = false;
    t.life_state = TrackLifeState::kTracked;
    t.stable_area = std::max(1.0F, t.bbox.area());
    InitializeKalman(&t, t.bbox);
    tracks_.push_back(t);
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const TrackState &t) {
                 return t.time_since_update > config_.track_buffer;
               }),
               tracks_.end());

  UpdateOcclusionProtection();

  std::vector<Track> out;
  out.reserve(tracks_.size());
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    const auto &t = tracks_[i];
    if (t.life_state != TrackLifeState::kTracked || t.time_since_update > 0) {
      continue;
    }
    const auto duplicate_output = FindDuplicateOutputSuppression(i);
    if (duplicate_output.suppressed) {
      AppendDuplicateOutputHypothesis(t, duplicate_output);
      continue;
    }

    Track track;
    track.id = t.id;
    track.class_id = t.class_id;
    track.confidence = t.score;
    track.bbox = ClampRect(t.bbox);
    track.is_confirmed = t.is_confirmed;
    track.time_since_update = t.time_since_update;
    track.authoritative = false;
    track.occlusion_suspect = t.occlusion_suspect && t.occlusion_protect_remaining > 0;
    track.appearance_feature = FeatureMatToVector(t.appearance_feat);
    track.association = t.last_association;
    track.just_recovered = t.just_recovered;
    track.low_score_update = t.low_score_update;
    out.push_back(track);
  }

  MirrorTrackedHypotheses(out);
  return out;
}

std::vector<Track> MotTracker::UpdateNewCore(const std::vector<Detection> &detections, const cv::Mat &frame) {
  frame_id_ += 1;
  if (observability_ != nullptr) {
    observability_->BeginFrame(frame_id_);
  }
  diag_snapshots_.clear();
  diag_snapshots_.resize(tracks_.size());

  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    auto &t = tracks_[i];
    if (t.life_state == TrackLifeState::kRemoved) {
      continue;
    }

    auto &snap = diag_snapshots_[i];
    snap.valid = true;
    snap.track_id = t.id;
    snap.class_id = t.class_id;
    snap.life_state = t.life_state;
    snap.pre_gmc_bbox = t.bbox;
    snap.pre_gmc_pred_bbox = t.predicted_bbox;
    snap.pre_state_post = t.kf.statePost.clone();
    snap.pre_error_cov_post = t.kf.errorCovPost.clone();

    PredictTrack(&t);
    snap.post_predict_state_pre = t.kf.statePre.clone();
    snap.post_predict_state_post = t.kf.statePost.clone();
    snap.post_predict_error_cov_pre = t.kf.errorCovPre.clone();
    snap.post_predict_error_cov_post = t.kf.errorCovPost.clone();
    snap.pre_gmc_pred_bbox = t.predicted_bbox;

    t.time_since_update += 1;
    t.age += 1;
  }

  bool gmc_ok = false;
  cv::Mat gmc_warp;
  ApplyGmc(frame, &tracks_, &gmc_ok, &gmc_warp);
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    auto &t = tracks_[i];
    if (t.life_state == TrackLifeState::kRemoved) {
      continue;
    }
    if (i >= diag_snapshots_.size()) {
      continue;
    }
    auto &snap = diag_snapshots_[i];
    if (!snap.valid) {
      continue;
    }
    snap.post_gmc_bbox = t.bbox;
    snap.post_gmc_pred_bbox = t.predicted_bbox;
    snap.post_gmc_state_pre = t.kf.statePre.clone();
    snap.post_gmc_state_post = t.kf.statePost.clone();
    snap.post_gmc_error_cov_pre = t.kf.errorCovPre.clone();
    snap.post_gmc_error_cov_post = t.kf.errorCovPost.clone();
  }
  DiagWriteGmc(gmc_ok, gmc_warp);

  std::vector<Detection> det_high;
  std::vector<cv::Mat> feat_high;
  std::vector<Detection> det_low;
  std::vector<cv::Mat> feat_low;
  det_high.reserve(detections.size());
  feat_high.reserve(detections.size());
  det_low.reserve(detections.size());
  feat_low.reserve(detections.size());

  for (const auto &det : detections) {
    if (det.confidence >= config_.track_high_thresh) {
      det_high.push_back(det);
      feat_high.push_back(ExtractAppearanceFeature(frame, det.bbox));
    } else if (det.confidence >= config_.track_low_thresh) {
      det_low.push_back(det);
      feat_low.push_back(ExtractAppearanceFeature(frame, det.bbox));
    }
  }

  DiagWriteTracks(det_high, det_low);
  DiagWriteDetections(det_high, det_low);

  std::vector<int> confirmed_tracks;
  std::vector<int> unconfirmed_tracks;
  std::vector<int> lost_tracks;
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    const auto &t = tracks_[i];
    if (t.life_state == TrackLifeState::kRemoved) {
      continue;
    }
    if (t.life_state == TrackLifeState::kLost) {
      lost_tracks.push_back(static_cast<int>(i));
      continue;
    }
    if (t.is_confirmed) {
      confirmed_tracks.push_back(static_cast<int>(i));
    } else {
      unconfirmed_tracks.push_back(static_cast<int>(i));
    }
  }

  std::vector<bool> matched_track(tracks_.size(), false);
  std::vector<bool> matched_high(det_high.size(), false);
  std::vector<bool> matched_low(det_low.size(), false);

  // Stage-1: confirmed tracks with high-confidence detections.
  std::vector<int> det_high_src(det_high.size(), -1);
  for (std::size_t i = 0; i < det_high.size(); ++i) {
    det_high_src[i] = static_cast<int>(i);
  }
  const auto stage1_matches =
      MatchByHungarian(confirmed_tracks, det_high, feat_high,
                       true,
                       config_.stage1_iou_min, config_.stage1_max_cost, "stage1_confirmed_high", &det_high_src);
  for (const auto &m : stage1_matches) {
    matched_track[m.first] = true;
    matched_high[m.second] = true;
    const auto terms = ComputeAssociationTerms(tracks_[m.first], det_high[m.second], feat_high[m.second], true,
                                               config_.stage1_iou_min);
    SetAcceptedAssociation(&tracks_[m.first], terms, "stage1_confirmed_high", false, false);
    UpdateTrackNewCore(&tracks_[m.first], det_high[m.second], feat_high[m.second]);
  }

  // Stage-2: unmatched confirmed tracks with low-confidence detections.
  std::vector<int> stage2_tracks;
  for (const int idx : confirmed_tracks) {
    if (!matched_track[idx] && tracks_[idx].life_state == TrackLifeState::kTracked) {
      stage2_tracks.push_back(idx);
    }
  }
  std::vector<int> det_low_src(det_low.size(), -1);
  for (std::size_t i = 0; i < det_low.size(); ++i) {
    det_low_src[i] = static_cast<int>(i);
  }
  const auto stage2_matches = MatchByHungarian(stage2_tracks, det_low, feat_low,
                                               config_.use_low_score_appearance_gate,
                                               config_.stage2_iou_min, config_.stage2_max_cost,
                                               "stage2_confirmed_low", &det_low_src);
  for (const auto &m : stage2_matches) {
    matched_track[m.first] = true;
    matched_low[m.second] = true;
    const auto terms = ComputeAssociationTerms(tracks_[m.first], det_low[m.second], feat_low[m.second],
                                               config_.use_low_score_appearance_gate, config_.stage2_iou_min);
    SetAcceptedAssociation(&tracks_[m.first], terms, "stage2_confirmed_low", true, false);
    UpdateTrackNewCore(&tracks_[m.first], det_low[m.second], cv::Mat());
  }

  // Lost-track recovery with remaining high-confidence detections.
  std::vector<Detection> det_high_unmatched;
  std::vector<cv::Mat> feat_high_unmatched;
  std::vector<int> det_high_unmatched_src;
  for (std::size_t i = 0; i < det_high.size(); ++i) {
    if (!matched_high[i]) {
      det_high_unmatched.push_back(det_high[i]);
      feat_high_unmatched.push_back(feat_high[i]);
      det_high_unmatched_src.push_back(static_cast<int>(i));
    }
  }

  const auto lost_matches =
      MatchByHungarian(lost_tracks, det_high_unmatched, feat_high_unmatched,
                       true,  // Appearance is mandatory in runtime matching.
                       std::max(0.30F, config_.stage1_iou_min), config_.lost_recovery_max_cost,
                       "lost_recovery", &det_high_unmatched_src);
  for (const auto &m : lost_matches) {
    matched_track[m.first] = true;
    const int src_idx = det_high_unmatched_src[m.second];
    matched_high[src_idx] = true;
    const auto terms = ComputeAssociationTerms(tracks_[m.first], det_high[src_idx], feat_high[src_idx], true,
                                               std::max(0.30F, config_.stage1_iou_min));
    SetAcceptedAssociation(&tracks_[m.first], terms, "lost_recovery", false, true);
    UpdateTrackNewCore(&tracks_[m.first], det_high[src_idx], cv::Mat());
  }

  det_high_unmatched.clear();
  feat_high_unmatched.clear();
  det_high_unmatched_src.clear();
  for (std::size_t i = 0; i < det_high.size(); ++i) {
    if (!matched_high[i]) {
      det_high_unmatched.push_back(det_high[i]);
      feat_high_unmatched.push_back(feat_high[i]);
      det_high_unmatched_src.push_back(static_cast<int>(i));
    }
  }

  // Unconfirmed tracks consume remaining high detections.
  const auto unconfirmed_matches =
      MatchByHungarian(unconfirmed_tracks, det_high_unmatched, feat_high_unmatched,
                       false, config_.unconfirmed_iou_min, config_.unconfirmed_max_cost,
                       "unconfirmed_high", &det_high_unmatched_src);
  for (const auto &m : unconfirmed_matches) {
    matched_track[m.first] = true;
    const int src_idx = det_high_unmatched_src[m.second];
    matched_high[src_idx] = true;
    const auto terms = ComputeAssociationTerms(tracks_[m.first], det_high[src_idx], feat_high[src_idx], false,
                                               config_.unconfirmed_iou_min);
    SetAcceptedAssociation(&tracks_[m.first], terms, "unconfirmed_high", false, false);
    UpdateTrackNewCore(&tracks_[m.first], det_high[src_idx], feat_high[src_idx]);
  }

  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    auto &t = tracks_[i];
    if (t.life_state == TrackLifeState::kRemoved) {
      continue;
    }
    if (matched_track[i]) {
      continue;
    }

    if (!t.is_confirmed && t.time_since_update > 2) {
      t.life_state = TrackLifeState::kRemoved;
      continue;
    }

    if (t.life_state == TrackLifeState::kTracked) {
      t.life_state = TrackLifeState::kLost;
    }

    if (t.time_since_update > config_.track_buffer) {
      t.life_state = TrackLifeState::kRemoved;
    }
  }

  for (std::size_t d = 0; d < det_high.size(); ++d) {
    if (matched_high[d] || det_high[d].confidence < config_.new_track_thresh) {
      continue;
    }
    const auto suppression = ShouldSuppressNewTrack(det_high[d]);
    if (suppression.suppressed) {
      AppendSuppressedNewTrackHypothesis(det_high[d], suppression);
      continue;
    }

    TrackState t;
    t.id = next_track_id_++;
    t.class_id = det_high[d].class_id;
    t.score = det_high[d].confidence;
    t.bbox = ClampRect(det_high[d].bbox);
    t.predicted_bbox = t.bbox;
    t.hits = 1;
    t.age = 1;
    t.time_since_update = 0;
    t.is_confirmed = (config_.confirm_hits <= 1);
    t.life_state = TrackLifeState::kTracked;
    t.stable_area = std::max(1.0F, t.bbox.area());
    t.last_association.stage = "new_track_high";
    t.last_association.fused_cost = 0.0F;
    t.last_association.iou = 1.0F;
    t.last_association.motion_dist = 0.0F;
    t.last_association.motion_term = 0.0F;
    t.last_association.app_dist = feat_high[d].empty() ? 1.0F : 0.0F;
    t.last_association.appearance_used = !feat_high[d].empty();
    t.last_association.passed_final_cost_gate = true;
    InitializeKalman(&t, t.bbox);
    if (!feat_high[d].empty()) {
      t.appearance_feat = feat_high[d].clone();
      t.has_appearance = true;
    }
    tracks_.push_back(t);
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const TrackState &t) { return t.life_state == TrackLifeState::kRemoved; }),
               tracks_.end());

  UpdateOcclusionProtection();

  std::vector<Track> out;
  out.reserve(tracks_.size());
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    const auto &t = tracks_[i];
    if (t.life_state != TrackLifeState::kTracked || t.time_since_update > 0) {
      continue;
    }
    const auto duplicate_output = FindDuplicateOutputSuppression(i);
    if (duplicate_output.suppressed) {
      AppendDuplicateOutputHypothesis(t, duplicate_output);
      continue;
    }

    Track track;
    track.id = t.id;
    track.class_id = t.class_id;
    track.confidence = t.score;
    track.bbox = ClampRect(t.bbox);
    track.is_confirmed = t.is_confirmed;
    track.time_since_update = t.time_since_update;
    track.authoritative = false;
    track.occlusion_suspect = t.occlusion_suspect && t.occlusion_protect_remaining > 0;
    track.appearance_feature = FeatureMatToVector(t.appearance_feat);
    track.association = t.last_association;
    track.just_recovered = t.just_recovered;
    track.low_score_update = t.low_score_update;
    out.push_back(track);
  }

  MirrorTrackedHypotheses(out);
  return out;
}

std::vector<Track> MotTracker::Update(const std::vector<Detection> &detections, const cv::Mat &frame) {
  if (ToLower(config_.core_mode) == "old_minimal") {
    return UpdateOldMinimal(detections, frame);
  }
  return UpdateNewCore(detections, frame);
}

}  // namespace vision_demo_host
