#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "vision_demo_host/modules/det_filter.hpp"
#include "vision_demo_host/modules/identity_manager.hpp"
#include "vision_demo_host/modules/mot_tracker.hpp"
#include "vision_demo_host/modules/preprocess_infer.hpp"
#include "vision_demo_host/modules/primary_recovery_debug.hpp"
#include "vision_demo_host/modules/primary_target_manager.hpp"
#include "vision_demo_host/tools/offline_eval_input.hpp"
#include "vision_demo_host/tools/offline_eval_schema.hpp"
#include "vision_demo_host/tools/identity_offline_metrics.hpp"
#include "vision_demo_host/types.hpp"

namespace {

struct Options {
  std::filesystem::path recordings_root{"data/captures"};
  std::filesystem::path results_root{"data/eval_results"};
  std::string run_name{"oe"};
  std::string detector_engine_path{
      "/path/to/my_workplace/vision_demo_ws/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine"};
  float det_raw_conf_threshold{0.10F};
  float det_person_conf_threshold{0.10F};
  float det_car_conf_threshold{0.10F};
  std::string tracker_config_path{"/path/to/my_workplace/vision_demo_ws/src/vision_demo_host/config/bot_sort.yaml"};
  bool tracker_gmc_enabled{vision_demo_host::MotTracker::Config::kDefaultGmcEnabled};
  std::string tracker_reid_backend{"light"};
  std::string tracker_reid_model_path{};
  int tracker_reid_input_width{128};
  int tracker_reid_input_height{256};
  bool save_frame_csv{true};
  bool save_sid_scores{true};
  bool save_tracks_csv{true};
  bool overlay_preview{false};
  bool overlay_record{false};
  bool short_dataset_dir_names{true};
  std::string overlay_video_name{"eval_overlay.mkv"};
  std::filesystem::path explicit_video_path;
  int target_lost_threshold_frames{180};
  int sid_feat_bank_size{30};
  float sid_recover_sim_thresh_strict{0.85F};
  float sid_recover_sim_thresh_relaxed{0.75F};
  int sid_recover_relaxed_max_missing_frames{180};
  int sid_occlusion_protect_frames{30};
  float sid_missing_assign_min_area_ratio{0.40F};
  float sid_missing_assign_max_area_ratio{4.00F};
  float sid_missing_assign_max_center_dist_norm{2.50F};
  float sid_missing_assign_max_app_cost{0.50F};
  float sid_overlap_iou_freeze{0.10F};
  int sid_split_stable_frames{3};
  int sid_merge_hold_frames{2};
  float sid_app_w{0.70F};
  float sid_geo_w{0.20F};
  float sid_time_w{0.10F};
  float sid_active_assign_max_cost{0.55F};
  float sid_recovery_max_cost{0.45F};
  float sid_raw_continuity_max_cost{0.55F};
  float sid_min_assignment_margin{0.08F};
  int sid_stable_frames_before_feature_update{3};
  bool sid_merged_requires_overlap{true};
  bool sid_reid_enable{true};  // compatibility input, runtime forces true.
  std::string sid_reid_backend{"light"};
  std::string sid_reid_model_path{};
  int sid_reid_input_width{128};
  int sid_reid_input_height{256};
  std::vector<std::string> datasets;
};

struct DatasetMetrics {
  std::string dataset_name;
  std::filesystem::path dataset_dir;
  std::filesystem::path source_video_path;
  std::string source_kind;
  std::string overlay_mode;
  bool capture_metadata_present{false};
  std::size_t capture_metadata_written_frames{0};
  std::size_t timestamp_rows{0};
  bool ok{false};
  std::string error;

  std::size_t total_frames{0};
  std::size_t det_positive_frames{0};
  std::size_t tracks_positive_frames{0};
  std::size_t locked_frames{0};
  std::size_t occluded_frames{0};
  std::size_t lost_frames{0};
  std::size_t primary_switch_count{0};
  std::size_t locked_to_lost_count{0};

  double avg_fps{0.0};
};

struct EvaluationRequest {
  std::size_t index{0U};
  std::string name;
  std::string short_name;
  std::filesystem::path dataset_dir;
  std::filesystem::path explicit_video_path;
};

std::string Trim(const std::string &s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

std::string JsonEscape(const std::string &s) {
  std::ostringstream oss;
  for (const char c : s) {
    switch (c) {
      case '"':
        oss << "\\\"";
        break;
      case '\\':
        oss << "\\\\";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        oss << c;
        break;
    }
  }
  return oss.str();
}

std::string TrackletHypothesisStatusToCsv(const vision_demo_host::TrackletHypothesisStatus status) {
  switch (status) {
    case vision_demo_host::TrackletHypothesisStatus::kTracked:
      return "tracked";
    case vision_demo_host::TrackletHypothesisStatus::kTentative:
      return "tentative";
    case vision_demo_host::TrackletHypothesisStatus::kLostPrediction:
      return "lost_prediction";
    case vision_demo_host::TrackletHypothesisStatus::kSuppressedDuplicateCandidate:
      return "suppressed_duplicate_candidate";
    case vision_demo_host::TrackletHypothesisStatus::kSplitCandidate:
      return "split_candidate";
    case vision_demo_host::TrackletHypothesisStatus::kLowQualityCandidate:
      return "low_quality_candidate";
  }
  return "unknown";
}

std::string TimestampCompactNow() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

void PrintUsage() {
  std::cout
      << "Usage: offline_eval_recordings [options]\n"
      << "  --recordings-root <path>      (default: data/captures)\n"
      << "  --results-root <path>         (default: data/eval_results)\n"
      << "  --run-name <name>             (default: oe)\n"
      << "  --detector-engine <path>\n"
      << "  --det-raw-conf <f>             (default: 0.10)\n"
      << "  --det-person-conf <f>          (default: 0.10)\n"
      << "  --det-car-conf <f>             (default: 0.10)\n"
      << "  --tracker-config <path>\n"
      << "  --tracker-gmc-enabled <true|false> (default: false)\n"
      << "  --tracker-reid-backend <light|osnet_onnx> (default: light)\n"
      << "  --tracker-reid-model-path <path>      (default: \"\")\n"
      << "  --tracker-reid-input-width <n>        (default: 128)\n"
      << "  --tracker-reid-input-height <n>       (default: 256)\n"
      << "  --datasets <a,b,c>            (FFV1 take directories below --recordings-root)\n"
      << "  --video <path>                (one explicit FFV1/MKV take; overrides --datasets)\n"
      << "  One of --video or --datasets is required. Historical MP4 is migration-regression input only.\n"
      << "  --save-frame-csv <true|false> (default: true)\n"
      << "  --save-sid-scores <true|false> (default: true)\n"
      << "  --save-tracks-csv <true|false> (default: true)\n"
      << "  --overlay-preview <true|false> (default: false)\n"
      << "  --overlay-record <true|false>  (default: false; FFV1 MKV)\n"
      << "  --save-eval-video <true|false> (compatibility alias for --overlay-record)\n"
      << "  --short-dataset-dir-names <true|false> (default: true, output s01/s02/...)\n"
      << "  --overlay-video-name <name>   (default: eval_overlay.mkv; filename only)\n"
      << "  --eval-video-name <name>      (compatibility alias for --overlay-video-name)\n"
      << "  --target-lost-threshold-frames <n>     (default: 180)\n"
      << "  --sid-feat-bank-size <n>               (default: 30)\n"
      << "  --sid-recover-sim-thresh-strict <f>    (default: 0.85)\n"
      << "  --sid-recover-sim-thresh-relaxed <f>   (default: 0.75)\n"
      << "  --sid-recover-relaxed-max-missing-frames <n> (default: 180)\n"
      << "  --sid-occlusion-protect-frames <n>           (default: 30)\n"
      << "  --sid-missing-assign-min-area-ratio <f>      (default: 0.40)\n"
      << "  --sid-missing-assign-max-area-ratio <f>      (default: 4.00)\n"
      << "  --sid-missing-assign-max-center-dist-norm <f> (default: 2.50)\n"
      << "  --sid-missing-assign-max-app-cost <f>  (default: 0.50)\n"
      << "  --sid-overlap-iou-freeze <f>           (default: 0.10)\n"
      << "  --sid-split-stable-frames <n>          (default: 3)\n"
      << "  --sid-merge-hold-frames <n>            (default: 2)\n"
      << "  --sid-app-w <f>                        (default: 0.70)\n"
      << "  --sid-geo-w <f>                        (default: 0.20)\n"
      << "  --sid-time-w <f>                       (default: 0.10)\n"
      << "  --sid-active-assign-max-cost <f>       (default: 0.55)\n"
      << "  --sid-recovery-max-cost <f>            (default: 0.45)\n"
      << "  --sid-raw-continuity-max-cost <f>      (default: 0.55)\n"
      << "  --sid-min-assignment-margin <f>        (default: 0.08)\n"
      << "  --sid-stable-frames-before-feature-update <n> (default: 3)\n"
      << "  --sid-merged-requires-overlap <true|false> (default: true)\n"
      << "  --sid-reid-enable <true|false>         (compat-only; runtime forces true)\n"
      << "  --sid-reid-backend <light|osnet_onnx> (default: light)\n"
      << "  --sid-reid-model-path <path>           (default: \"\")\n"
      << "  --sid-reid-input-width <n>             (default: 128)\n"
      << "  --sid-reid-input-height <n>            (default: 256)\n"
      << "  --help\n\n"
      << vision_demo_host::tools::PerFrameCsvHelp() << "\n"
      << vision_demo_host::tools::TrackletHypothesesCsvHelp() << "\n"
      << vision_demo_host::tools::Phase3ShadowStateCsvHelp() << "\n"
      << vision_demo_host::tools::IdentityOfflineMetricsHelp();
}

bool ParseBool(const std::string &v, bool *out) {
  const std::string s = Trim(v);
  if (s == "true" || s == "1" || s == "yes" || s == "on") {
    *out = true;
    return true;
  }
  if (s == "false" || s == "0" || s == "no" || s == "off") {
    *out = false;
    return true;
  }
  return false;
}

bool ParseArgs(int argc, char **argv, Options *opt, std::string *error) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const std::string &name) -> std::string {
      if (i + 1 >= argc) {
        if (error != nullptr) {
          *error = "Missing value for " + name;
        }
        return "";
      }
      return argv[++i];
    };

    if (arg == "--recordings-root") {
      opt->recordings_root = need(arg);
    } else if (arg == "--results-root") {
      opt->results_root = need(arg);
    } else if (arg == "--run-name") {
      opt->run_name = need(arg);
    } else if (arg == "--detector-engine") {
      opt->detector_engine_path = need(arg);
    } else if (arg == "--det-raw-conf") {
      const std::string s = need(arg);
      try {
        opt->det_raw_conf_threshold = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --det-raw-conf: " + s;
        }
        return false;
      }
    } else if (arg == "--det-person-conf") {
      const std::string s = need(arg);
      try {
        opt->det_person_conf_threshold = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --det-person-conf: " + s;
        }
        return false;
      }
    } else if (arg == "--det-car-conf") {
      const std::string s = need(arg);
      try {
        opt->det_car_conf_threshold = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --det-car-conf: " + s;
        }
        return false;
      }
    } else if (arg == "--tracker-config") {
      opt->tracker_config_path = need(arg);
    } else if (arg == "--tracker-gmc-enabled") {
      bool v = true;
      if (!ParseBool(need(arg), &v)) {
        if (error != nullptr) {
          *error = "Invalid value for --tracker-gmc-enabled";
        }
        return false;
      }
      opt->tracker_gmc_enabled = v;
    } else if (arg == "--tracker-reid-backend") {
      opt->tracker_reid_backend = need(arg);
    } else if (arg == "--tracker-reid-model-path") {
      opt->tracker_reid_model_path = need(arg);
    } else if (arg == "--tracker-reid-input-width") {
      const std::string s = need(arg);
      try {
        opt->tracker_reid_input_width = std::max(16, std::stoi(s));
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --tracker-reid-input-width: " + s;
        }
        return false;
      }
    } else if (arg == "--tracker-reid-input-height") {
      const std::string s = need(arg);
      try {
        opt->tracker_reid_input_height = std::max(16, std::stoi(s));
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --tracker-reid-input-height: " + s;
        }
        return false;
      }
    } else if (arg == "--datasets") {
      opt->datasets.clear();
      std::stringstream ss(need(arg));
      std::string item;
      while (std::getline(ss, item, ',')) {
        item = Trim(item);
        if (!item.empty()) {
          opt->datasets.push_back(item);
        }
      }
    } else if (arg == "--save-frame-csv") {
      bool v = true;
      if (!ParseBool(need(arg), &v)) {
        if (error != nullptr) {
          *error = "Invalid value for --save-frame-csv";
        }
        return false;
      }
      opt->save_frame_csv = v;
    } else if (arg == "--save-sid-scores") {
      bool v = true;
      if (!ParseBool(need(arg), &v)) {
        if (error != nullptr) {
          *error = "Invalid value for --save-sid-scores";
        }
        return false;
      }
      opt->save_sid_scores = v;
    } else if (arg == "--save-tracks-csv") {
      bool v = true;
      if (!ParseBool(need(arg), &v)) {
        if (error != nullptr) {
          *error = "Invalid value for --save-tracks-csv";
        }
        return false;
      }
      opt->save_tracks_csv = v;
    } else if (arg == "--video") {
      opt->explicit_video_path = need(arg);
    } else if (arg == "--overlay-preview") {
      bool v = false;
      if (!ParseBool(need(arg), &v)) {
        if (error != nullptr) {
          *error = "Invalid value for --overlay-preview";
        }
        return false;
      }
      opt->overlay_preview = v;
    } else if (arg == "--overlay-record") {
      bool v = false;
      if (!ParseBool(need(arg), &v)) {
        if (error != nullptr) {
          *error = "Invalid value for --overlay-record";
        }
        return false;
      }
      opt->overlay_record = v;
    } else if (arg == "--save-eval-video") {
      bool v = true;
      if (!ParseBool(need(arg), &v)) {
        if (error != nullptr) {
          *error = "Invalid value for --save-eval-video";
        }
        return false;
      }
      opt->overlay_record = v;
    } else if (arg == "--short-dataset-dir-names") {
      bool v = true;
      if (!ParseBool(need(arg), &v)) {
        if (error != nullptr) {
          *error = "Invalid value for --short-dataset-dir-names";
        }
        return false;
      }
      opt->short_dataset_dir_names = v;
    } else if (arg == "--overlay-video-name" || arg == "--eval-video-name") {
      opt->overlay_video_name = need(arg);
    } else if (arg == "--target-lost-threshold-frames") {
      const std::string s = need(arg);
      try {
        opt->target_lost_threshold_frames = std::stoi(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --target-lost-threshold-frames: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-feat-bank-size") {
      const std::string s = need(arg);
      try {
        opt->sid_feat_bank_size = std::stoi(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-feat-bank-size: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-recover-sim-thresh-strict") {
      const std::string s = need(arg);
      try {
        opt->sid_recover_sim_thresh_strict = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-recover-sim-thresh-strict: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-recover-sim-thresh-relaxed") {
      const std::string s = need(arg);
      try {
        opt->sid_recover_sim_thresh_relaxed = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-recover-sim-thresh-relaxed: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-recover-relaxed-max-missing-frames") {
      const std::string s = need(arg);
      try {
        opt->sid_recover_relaxed_max_missing_frames = std::stoi(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-recover-relaxed-max-missing-frames: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-occlusion-protect-frames") {
      const std::string s = need(arg);
      try {
        opt->sid_occlusion_protect_frames = std::stoi(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-occlusion-protect-frames: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-missing-assign-min-area-ratio") {
      const std::string s = need(arg);
      try {
        opt->sid_missing_assign_min_area_ratio = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-missing-assign-min-area-ratio: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-missing-assign-max-area-ratio") {
      const std::string s = need(arg);
      try {
        opt->sid_missing_assign_max_area_ratio = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-missing-assign-max-area-ratio: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-missing-assign-max-center-dist-norm") {
      const std::string s = need(arg);
      try {
        opt->sid_missing_assign_max_center_dist_norm = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-missing-assign-max-center-dist-norm: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-missing-assign-max-app-cost") {
      const std::string s = need(arg);
      try {
        opt->sid_missing_assign_max_app_cost = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-missing-assign-max-app-cost: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-overlap-iou-freeze") {
      const std::string s = need(arg);
      try {
        opt->sid_overlap_iou_freeze = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-overlap-iou-freeze: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-split-stable-frames") {
      const std::string s = need(arg);
      try {
        opt->sid_split_stable_frames = std::stoi(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-split-stable-frames: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-merge-hold-frames") {
      const std::string s = need(arg);
      try {
        opt->sid_merge_hold_frames = std::stoi(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-merge-hold-frames: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-app-w") {
      const std::string s = need(arg);
      try {
        opt->sid_app_w = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-app-w: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-geo-w") {
      const std::string s = need(arg);
      try {
        opt->sid_geo_w = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-geo-w: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-time-w") {
      const std::string s = need(arg);
      try {
        opt->sid_time_w = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-time-w: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-active-assign-max-cost") {
      const std::string s = need(arg);
      try {
        opt->sid_active_assign_max_cost = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-active-assign-max-cost: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-recovery-max-cost") {
      const std::string s = need(arg);
      try {
        opt->sid_recovery_max_cost = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-recovery-max-cost: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-raw-continuity-max-cost") {
      const std::string s = need(arg);
      try {
        opt->sid_raw_continuity_max_cost = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-raw-continuity-max-cost: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-min-assignment-margin") {
      const std::string s = need(arg);
      try {
        opt->sid_min_assignment_margin = std::stof(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-min-assignment-margin: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-stable-frames-before-feature-update") {
      const std::string s = need(arg);
      try {
        opt->sid_stable_frames_before_feature_update = std::stoi(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-stable-frames-before-feature-update: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-merged-requires-overlap") {
      bool v = true;
      if (!ParseBool(need(arg), &v)) {
        if (error != nullptr) {
          *error = "Invalid value for --sid-merged-requires-overlap";
        }
        return false;
      }
      opt->sid_merged_requires_overlap = v;
    } else if (arg == "--sid-reid-enable") {
      bool v = true;
      if (!ParseBool(need(arg), &v)) {
        if (error != nullptr) {
          *error = "Invalid value for --sid-reid-enable";
        }
        return false;
      }
      opt->sid_reid_enable = v;
    } else if (arg == "--sid-reid-backend") {
      opt->sid_reid_backend = need(arg);
    } else if (arg == "--sid-reid-model-path") {
      opt->sid_reid_model_path = need(arg);
    } else if (arg == "--sid-reid-input-width") {
      const std::string s = need(arg);
      try {
        opt->sid_reid_input_width = std::max(16, std::stoi(s));
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-reid-input-width: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-reid-input-height") {
      const std::string s = need(arg);
      try {
        opt->sid_reid_input_height = std::max(16, std::stoi(s));
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-reid-input-height: " + s;
        }
        return false;
      }
    } else if (arg == "--help" || arg == "-h") {
      PrintUsage();
      return false;
    } else {
      if (error != nullptr) {
        *error = "Unknown argument: " + arg;
      }
      return false;
    }
  }
  if (opt->explicit_video_path.empty() && opt->datasets.empty()) {
    if (error != nullptr) {
      *error = "Choose an FFV1 capture with --video or --datasets; there is no historical default dataset.";
    }
    return false;
  }
  return true;
}

double Ratio(const std::size_t n, const std::size_t d) {
  if (d == 0) {
    return 0.0;
  }
  return static_cast<double>(n) / static_cast<double>(d);
}

void WriteDatasetJson(const std::filesystem::path &out_path, const DatasetMetrics &m) {
  std::ofstream ofs(out_path);
  ofs << "{\n"
      << "  \"dataset_name\": \"" << JsonEscape(m.dataset_name) << "\",\n"
      << "  \"dataset_dir\": \"" << JsonEscape(m.dataset_dir.string()) << "\",\n"
      << "  \"source_video_path\": \"" << JsonEscape(m.source_video_path.string()) << "\",\n"
      << "  \"source_kind\": \"" << JsonEscape(m.source_kind) << "\",\n"
      << "  \"overlay_mode\": \"" << JsonEscape(m.overlay_mode) << "\",\n"
      << "  \"capture_metadata_present\": " << (m.capture_metadata_present ? "true" : "false") << ",\n"
      << "  \"capture_metadata_written_frames\": " << m.capture_metadata_written_frames << ",\n"
      << "  \"timestamp_rows\": " << m.timestamp_rows << ",\n"
      << "  \"ok\": " << (m.ok ? "true" : "false") << ",\n"
      << "  \"error\": \"" << JsonEscape(m.error) << "\",\n"
      << "  \"total_frames\": " << m.total_frames << ",\n"
      << "  \"avg_fps\": " << std::fixed << std::setprecision(3) << m.avg_fps << ",\n"
      << "  \"det_positive_frames\": " << m.det_positive_frames << ",\n"
      << "  \"det_positive_ratio\": " << Ratio(m.det_positive_frames, m.total_frames) << ",\n"
      << "  \"tracks_positive_frames\": " << m.tracks_positive_frames << ",\n"
      << "  \"tracks_positive_ratio\": " << Ratio(m.tracks_positive_frames, m.total_frames) << ",\n"
      << "  \"locked_frames\": " << m.locked_frames << ",\n"
      << "  \"locked_ratio\": " << Ratio(m.locked_frames, m.total_frames) << ",\n"
      << "  \"occluded_frames\": " << m.occluded_frames << ",\n"
      << "  \"occluded_ratio\": " << Ratio(m.occluded_frames, m.total_frames) << ",\n"
      << "  \"lost_frames\": " << m.lost_frames << ",\n"
      << "  \"lost_ratio\": " << Ratio(m.lost_frames, m.total_frames) << ",\n"
      << "  \"primary_switch_count\": " << m.primary_switch_count << ",\n"
      << "  \"locked_to_lost_count\": " << m.locked_to_lost_count << "\n"
      << "}\n";
}

void DrawEvalOverlay(cv::Mat *canvas, const std::vector<vision_demo_host::Track> &tracks,
                     const vision_demo_host::IdentityManagerResult &identity_result,
                     const vision_demo_host::PrimaryTargetResult &primary,
                     const std::size_t frame_idx, const std::size_t det_count,
                     const std::string &primary_decision_reason,
                     const std::string &primary_reject_reason) {
  const int primary_semantic_id = primary.primary_target_id;

  auto find_identity_by_raw = [&](const int raw_track_id) -> const vision_demo_host::IdentityObservation * {
    for (const auto &identity : identity_result.identities) {
      if (identity.supporting_raw_track_id.has_value() && *identity.supporting_raw_track_id == raw_track_id) {
        return &identity;
      }
    }
    return nullptr;
  };

  for (const auto &track : tracks) {
    const auto *identity = find_identity_by_raw(track.id);
    if (identity == nullptr || identity->semantic_id < 0) {
      continue;
    }
    const int semantic_id = identity->semantic_id;
    const bool is_primary = (primary_semantic_id > 0 && semantic_id == primary_semantic_id);
    const cv::Scalar color = is_primary ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
    cv::rectangle(*canvas, track.bbox, color, 2);
    std::ostringstream label;
    label << "id=" << semantic_id << " " << vision_demo_host::IdentityStateToString(identity->state)
          << " raw=" << track.id;
    cv::putText(*canvas, label.str(), vision_demo_host::CompactOverlayTrackLabelPoint(canvas->size(), track.bbox),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
  }

  std::ostringstream line1;
  line1 << "frame=" << frame_idx << " det=" << det_count << " tracks=" << tracks.size();
  const std::string line2 = vision_demo_host::BuildPrimaryOverlayLine(
      primary, identity_result, primary_decision_reason, primary_reject_reason);
  cv::putText(*canvas, line1.str(), cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
  cv::putText(*canvas, line2, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}

void WriteDatasetMd(const std::filesystem::path &out_path, const DatasetMetrics &m) {
  std::ofstream ofs(out_path);
  ofs << "# Offline Eval Summary: " << m.dataset_name << "\n\n";
  ofs << "- dataset_dir: `" << m.dataset_dir.string() << "`\n";
  ofs << "- source_video_path: `" << m.source_video_path.string() << "`\n";
  ofs << "- source_kind: `" << m.source_kind << "`\n";
  ofs << "- overlay_mode: `" << m.overlay_mode << "`\n";
  ofs << "- capture_metadata_present: `" << (m.capture_metadata_present ? "true" : "false") << "`\n";
  if (m.capture_metadata_present) {
    ofs << "- capture_metadata_written_frames: `" << m.capture_metadata_written_frames << "`\n";
    ofs << "- timestamp_rows: `" << m.timestamp_rows << "`\n";
  }
  ofs << "- ok: `" << (m.ok ? "true" : "false") << "`\n";
  if (!m.error.empty()) {
    ofs << "- error: `" << m.error << "`\n";
  }
  ofs << "- total_frames: `" << m.total_frames << "`\n";
  ofs << "- avg_fps: `" << std::fixed << std::setprecision(3) << m.avg_fps << "`\n";
  ofs << "- det>0: `" << m.det_positive_frames << "` (" << std::setprecision(2)
      << Ratio(m.det_positive_frames, m.total_frames) * 100.0 << "%)\n";
  ofs << "- tracks>0: `" << m.tracks_positive_frames << "` (" << std::setprecision(2)
      << Ratio(m.tracks_positive_frames, m.total_frames) * 100.0 << "%)\n";
  ofs << "- state=LOCKED: `" << m.locked_frames << "` (" << std::setprecision(2)
      << Ratio(m.locked_frames, m.total_frames) * 100.0 << "%)\n";
  ofs << "- state=OCCLUDED: `" << m.occluded_frames << "` (" << std::setprecision(2)
      << Ratio(m.occluded_frames, m.total_frames) * 100.0 << "%)\n";
  ofs << "- state=LOST: `" << m.lost_frames << "` (" << std::setprecision(2)
      << Ratio(m.lost_frames, m.total_frames) * 100.0 << "%)\n";
  ofs << "- primary_switch_count: `" << m.primary_switch_count << "`\n";
  ofs << "- locked_to_lost_count: `" << m.locked_to_lost_count << "`\n";
}

void WriteGlobalSummary(const std::filesystem::path &out_path, const std::vector<DatasetMetrics> &all) {
  std::ofstream ofs(out_path);
  ofs << "# Offline Eval Global Summary\n\n";
  ofs << "| dataset | ok | frames | avg_fps | det>0 ratio | tracks>0 ratio | LOCKED ratio | OCCLUDED ratio | LOST ratio | switches | locked->lost |\n";
  ofs << "|---|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  for (const auto &m : all) {
    ofs << "| " << m.dataset_name << " | " << (m.ok ? "yes" : "no") << " | " << m.total_frames << " | "
        << std::fixed << std::setprecision(2) << m.avg_fps << " | " << std::setprecision(2)
        << Ratio(m.det_positive_frames, m.total_frames) * 100.0 << "% | "
        << Ratio(m.tracks_positive_frames, m.total_frames) * 100.0 << "% | "
        << Ratio(m.locked_frames, m.total_frames) * 100.0 << "% | "
        << Ratio(m.occluded_frames, m.total_frames) * 100.0 << "% | "
        << Ratio(m.lost_frames, m.total_frames) * 100.0 << "% | " << m.primary_switch_count << " | "
        << m.locked_to_lost_count << " |\n";
  }
}

DatasetMetrics EvaluateOne(const Options &opt, const std::filesystem::path &dataset_dir,
                           const std::filesystem::path &result_dir,
                           const std::filesystem::path &explicit_video_path = {}) {
  DatasetMetrics m;
  m.dataset_name = dataset_dir.filename().string();
  m.dataset_dir = dataset_dir;

  const auto input_discovery =
      vision_demo_host::tools::DiscoverOfflineEvalInput({dataset_dir, explicit_video_path});
  if (!input_discovery.ok) {
    m.error = input_discovery.error;
    return m;
  }
  const auto &input = input_discovery.input;
  const std::filesystem::path &video_path = input.video_path;
  m.source_video_path = video_path;
  m.source_kind = vision_demo_host::tools::OfflineEvalSourceKindToString(input.source_kind);
  m.overlay_mode = vision_demo_host::tools::OfflineEvalOverlayModeToString(
      vision_demo_host::tools::OfflineEvalOverlayModeFor(opt.overlay_preview, opt.overlay_record));
  m.capture_metadata_present = input.capture.has_value();
  if (input.capture.has_value()) {
    m.capture_metadata_written_frames = input.capture->written_frames;
    m.timestamp_rows = input.timestamp_validation.rows;
  }

  const auto overlay_plan = vision_demo_host::tools::PlanOfflineEvalOverlayArtifacts(
      input, result_dir, opt.overlay_record, opt.overlay_video_name);
  if (!overlay_plan.ok) {
    m.error = overlay_plan.error;
    return m;
  }

  vision_demo_host::PreprocessInfer::Config infer_cfg;
  infer_cfg.detector_runtime_path = opt.detector_engine_path;
  infer_cfg.raw_conf_threshold = std::max(0.0F, opt.det_raw_conf_threshold);
  infer_cfg.input_width = 640;
  infer_cfg.input_height = 640;
  infer_cfg.enable_fake_detection = false;
  vision_demo_host::PreprocessInfer infer(infer_cfg);

  std::string err;
  if (!infer.Initialize(&err)) {
    m.error = "preprocess_infer init failed: " + err;
    return m;
  }

  vision_demo_host::DetFilter::Config filter_cfg;
  filter_cfg.person_conf_threshold = std::max(0.0F, opt.det_person_conf_threshold);
  filter_cfg.car_conf_threshold = std::max(0.0F, opt.det_car_conf_threshold);
  vision_demo_host::DetFilter det_filter(filter_cfg);

  vision_demo_host::MotTracker::Config tracker_cfg;
  tracker_cfg.tracker_yaml_path = opt.tracker_config_path;
  tracker_cfg.gmc_enabled = opt.tracker_gmc_enabled;
  tracker_cfg.reid_enabled = true;
  tracker_cfg.gmc_method = "sparseOptFlow";
  tracker_cfg.gmc_downscale = 4;
  tracker_cfg.with_reid = true;
  tracker_cfg.reid_backend = opt.tracker_reid_backend;
  tracker_cfg.reid_model_path = opt.tracker_reid_model_path;
  tracker_cfg.reid_input_width = std::max(16, opt.tracker_reid_input_width);
  tracker_cfg.reid_input_height = std::max(16, opt.tracker_reid_input_height);
  vision_demo_host::MotTracker tracker(tracker_cfg);
  if (!tracker.Initialize(&err)) {
    m.error = "mot_tracker init failed: " + err;
    return m;
  }

  if (infer_cfg.raw_conf_threshold > tracker.EffectiveConfig().track_low_thresh) {
    std::cout << "[offline_eval] warning: det raw threshold " << infer_cfg.raw_conf_threshold
              << " is above tracker.track_low_thresh " << tracker.EffectiveConfig().track_low_thresh
              << "; low-score recovery is pre-filtered before tracker" << std::endl;
  }
  if (filter_cfg.person_conf_threshold > tracker.EffectiveConfig().track_low_thresh ||
      filter_cfg.car_conf_threshold > tracker.EffectiveConfig().track_low_thresh) {
    std::cout << "[offline_eval] warning: detector class thresholds person=" << filter_cfg.person_conf_threshold
              << " car=" << filter_cfg.car_conf_threshold << " exceed tracker.track_low_thresh "
              << tracker.EffectiveConfig().track_low_thresh << "; low-score recovery may be pre-filtered" << std::endl;
  }

  vision_demo_host::PrimaryTargetManager::Config target_cfg;
  target_cfg.lost_threshold_frames = opt.target_lost_threshold_frames;
  target_cfg.min_person_area_px = 1000.0F;
  vision_demo_host::PrimaryTargetManager primary_mgr(target_cfg);

  vision_demo_host::IdentityManager::Config sid_cfg;
  sid_cfg.max_missing_frames = std::max(1, opt.target_lost_threshold_frames);
  sid_cfg.feat_bank_size = std::max(1, opt.sid_feat_bank_size);
  sid_cfg.recover_sim_thresh_strict = std::clamp(opt.sid_recover_sim_thresh_strict, 0.0F, 1.0F);
  sid_cfg.recover_sim_thresh_relaxed =
      std::clamp(opt.sid_recover_sim_thresh_relaxed, 0.0F, 1.0F);
  sid_cfg.recover_relaxed_max_missing_frames = std::max(1, opt.sid_recover_relaxed_max_missing_frames);
  sid_cfg.occlusion_protect_frames = std::max(0, opt.sid_occlusion_protect_frames);
  sid_cfg.missing_assign_min_area_ratio = std::max(0.01F, opt.sid_missing_assign_min_area_ratio);
  sid_cfg.missing_assign_max_area_ratio =
      std::max(sid_cfg.missing_assign_min_area_ratio, opt.sid_missing_assign_max_area_ratio);
  sid_cfg.missing_assign_max_center_dist_norm = std::max(0.1F, opt.sid_missing_assign_max_center_dist_norm);
  sid_cfg.missing_assign_max_app_cost = std::clamp(opt.sid_missing_assign_max_app_cost, 0.0F, 1.0F);
  sid_cfg.overlap_iou_freeze = std::max(0.0F, opt.sid_overlap_iou_freeze);
  sid_cfg.split_stable_frames = std::max(1, opt.sid_split_stable_frames);
  sid_cfg.merge_hold_frames = std::max(1, opt.sid_merge_hold_frames);
  sid_cfg.app_w = std::max(0.0F, opt.sid_app_w);
  sid_cfg.geo_w = std::max(0.0F, opt.sid_geo_w);
  sid_cfg.time_w = std::max(0.0F, opt.sid_time_w);
  sid_cfg.active_assign_max_cost = std::clamp(opt.sid_active_assign_max_cost, 0.0F, 1.0F);
  sid_cfg.recovery_max_cost = std::clamp(opt.sid_recovery_max_cost, 0.0F, 1.0F);
  sid_cfg.raw_continuity_max_cost = std::clamp(opt.sid_raw_continuity_max_cost, 0.0F, 1.0F);
  sid_cfg.min_assignment_margin = std::max(0.0F, opt.sid_min_assignment_margin);
  sid_cfg.stable_frames_before_feature_update = std::max(1, opt.sid_stable_frames_before_feature_update);
  sid_cfg.merged_requires_overlap = opt.sid_merged_requires_overlap;
  if (!opt.sid_reid_enable) {
    std::cout << "[offline_eval] reid is mandatory; override --sid-reid-enable=false to true" << std::endl;
  }
  sid_cfg.reid_enable = true;
  sid_cfg.reid_backend = opt.sid_reid_backend;
  sid_cfg.reid_model_path = opt.sid_reid_model_path;
  sid_cfg.reid_input_width = std::max(16, opt.sid_reid_input_width);
  sid_cfg.reid_input_height = std::max(16, opt.sid_reid_input_height);
  vision_demo_host::IdentityManager identity_manager(sid_cfg);
  if (!identity_manager.Initialize(&err)) {
    m.error = "identity_manager init failed: " + err;
    return m;
  }

  cv::VideoCapture cap(video_path.string());
  if (!cap.isOpened()) {
    m.error = "Failed to open video: " + video_path.string();
    return m;
  }

  std::ofstream frame_csv;
  std::ofstream det_raw_csv;
  std::ofstream det_filtered_csv;
  std::ofstream sid_scores_csv;
  std::ofstream tracks_csv;
  std::ofstream hypotheses_csv;
  std::ofstream phase3_shadow_csv;
  std::ofstream identities_csv;
  if (opt.save_frame_csv) {
    frame_csv.open(result_dir / "per_frame.csv");
    frame_csv << vision_demo_host::tools::PerFrameCsvHeader() << "\n";

    det_raw_csv.open(result_dir / "det_raw.csv");
    det_raw_csv << "frame_idx,det_idx,class_id,score,x,y,w,h\n";
    det_filtered_csv.open(result_dir / "det_filtered.csv");
    det_filtered_csv << "frame_idx,det_idx,class_id,score,x,y,w,h\n";
  }
  if (opt.save_sid_scores) {
    sid_scores_csv.open(result_dir / "sid_scores.csv");
    sid_scores_csv << vision_demo_host::tools::SidScoresCsvHeader() << "\n";
  }
  if (opt.save_tracks_csv) {
    tracks_csv.open(result_dir / "tracks.csv");
    tracks_csv << "frame_idx,track_idx,raw_track_id,semantic_id,class_id,score,x,y,w,h,is_confirmed,time_since_update,"
                  "assoc_stage,assoc_cost,assoc_iou,assoc_motion_dist,assoc_app_dist,assoc_appearance_used,low_score_update,just_recovered,assoc_final_gate,assoc_reject_reason,occlusion_suspect\n";
    hypotheses_csv.open(result_dir / "tracklet_hypotheses.csv");
    hypotheses_csv << vision_demo_host::tools::TrackletHypothesesCsvHeader() << "\n";
    phase3_shadow_csv.open(result_dir / "phase3_shadow_state.csv");
    phase3_shadow_csv << vision_demo_host::tools::Phase3ShadowStateCsvHeader() << "\n";
    identities_csv.open(result_dir / "identities.csv");
    identities_csv << "frame_idx,semantic_id,identity_state,visible,supporting_raw_track_id,class_id,score,x,y,w,h,"
                      "missing_frames,primary,occlusion_suspect,low_score_update,just_recovered,assignment_stage,"
                      "assignment_accepted,assignment_reject_reason\n";
  }

  cv::VideoWriter overlay_writer;
  bool overlay_writer_open_failed = false;
  bool preview_window_open = false;
  constexpr const char *kOverlayWindowName = "offline_eval_overlay";

  std::optional<int> prev_primary_semantic_id;
  std::optional<vision_demo_host::PrimaryState> prev_state;

  const auto t0 = std::chrono::steady_clock::now();
  cv::Mat frame;
  std::size_t frame_idx = 0;
  while (cap.read(frame)) {
    if (frame.empty()) {
      break;
    }

    const auto detections = infer.Infer(frame);
    const auto filtered = det_filter.Filter(detections);
    const auto tracks = tracker.Update(filtered, frame);
    const auto primary_prev = primary_mgr.GetState();
    const auto &tracklet_hypotheses = tracker.LastTrackletHypotheses();
    const auto identity_result = identity_manager.Update(
        vision_demo_host::TrackletObservationsFromTracks(tracks), tracklet_hypotheses, primary_prev, &frame);
    const auto primary = primary_mgr.Update(identity_result.identities);

    const int primary_semantic_id = primary.primary_target_id;
    int raw_track_id = primary.raw_track_id;

    m.total_frames++;
    if (!detections.empty()) {
      m.det_positive_frames++;
    }
    if (!tracks.empty()) {
      m.tracks_positive_frames++;
    }
    if (primary.state == vision_demo_host::PrimaryState::kLocked) {
      m.locked_frames++;
    } else if (primary.state == vision_demo_host::PrimaryState::kOccluded) {
      m.occluded_frames++;
    } else if (primary.state == vision_demo_host::PrimaryState::kLost) {
      m.lost_frames++;
    }

    if (prev_primary_semantic_id.has_value() && primary_semantic_id > 0 && prev_primary_semantic_id.value() > 0 &&
        primary_semantic_id != prev_primary_semantic_id.value()) {
      m.primary_switch_count++;
    }
    if (prev_state.has_value() && prev_state.value() == vision_demo_host::PrimaryState::kLocked &&
        primary.state == vision_demo_host::PrimaryState::kLost) {
      m.locked_to_lost_count++;
    }

    prev_primary_semantic_id = primary_semantic_id;
    prev_state = primary.state;

    if (frame_csv.is_open()) {
      std::vector<int> visible_semantic_ids;
      visible_semantic_ids.reserve(tracks.size());
      for (const auto &track : tracks) {
        const int sid = identity_result.SemanticIdForRawTrack(track.id);
        if (sid > 0) {
          visible_semantic_ids.push_back(sid);
        }
      }
      std::sort(visible_semantic_ids.begin(), visible_semantic_ids.end());
      visible_semantic_ids.erase(std::unique(visible_semantic_ids.begin(), visible_semantic_ids.end()),
                                 visible_semantic_ids.end());
      std::ostringstream sid_list;
      for (std::size_t i = 0; i < visible_semantic_ids.size(); ++i) {
        if (i > 0) {
          sid_list << "|";
        }
        sid_list << visible_semantic_ids[i];
      }
      frame_csv << frame_idx << "," << detections.size() << "," << tracks.size() << ","
                << vision_demo_host::PrimaryStateToString(primary.state) << "," << primary_semantic_id << ","
                << raw_track_id << ","
                << vision_demo_host::IdentityModeToString(identity_manager.CurrentMode()) << ","
                << (identity_manager.IsFeatureUpdateFrozen() ? "1" : "0") << ","
                << sid_list.str() << "," << primary_mgr.LastDecisionReason() << ","
                << primary_mgr.LastRejectReason() << ","
                << vision_demo_host::PrimaryRecoveryReasonToken(primary, identity_result,
                                                                primary_mgr.LastDecisionReason(),
                                                                primary_mgr.LastRejectReason()) << ","
                << vision_demo_host::PrimarySupportingRawTrackIdDebug(primary, identity_result) << "\n";
    }
    if (det_raw_csv.is_open()) {
      for (std::size_t i = 0; i < detections.size(); ++i) {
        const auto &d = detections[i];
        det_raw_csv << frame_idx << "," << i << "," << static_cast<int>(d.class_id) << ","
                    << std::fixed << std::setprecision(6) << d.confidence << ","
                    << d.bbox.x << "," << d.bbox.y << "," << d.bbox.width << "," << d.bbox.height << "\n";
      }
    }
    if (det_filtered_csv.is_open()) {
      for (std::size_t i = 0; i < filtered.size(); ++i) {
        const auto &d = filtered[i];
        det_filtered_csv << frame_idx << "," << i << "," << static_cast<int>(d.class_id) << ","
                         << std::fixed << std::setprecision(6) << d.confidence << ","
                         << d.bbox.x << "," << d.bbox.y << "," << d.bbox.width << "," << d.bbox.height << "\n";
      }
    }
    if (sid_scores_csv.is_open()) {
      for (const auto &row : identity_manager.LastScoreDebugRows()) {
        sid_scores_csv << row.frame_idx << "," << vision_demo_host::IdentityModeToString(row.mode) << ","
                       << row.track_idx << "," << row.raw_track_id << "," << row.semantic_id << ","
                       << std::fixed << std::setprecision(6) << row.app_cost << "," << row.geo_cost << ","
                       << row.time_cost << "," << row.final_score << "," << row.stage << ","
                       << (row.selected ? "1" : "0") << "," << row.margin << ","
                       << (row.accepted ? "1" : "0") << "," << row.reject_reason << ","
                       << (row.continuity_used ? "1" : "0") << ","
                       << (row.feature_update_allowed ? "1" : "0") << ","
                       << (row.geometry_update_allowed ? "1" : "0") << ","
                       << row.feature_update_reason << "," << row.geometry_update_reason << "\n";
      }
    }
    if (tracks_csv.is_open()) {
      for (std::size_t i = 0; i < tracks.size(); ++i) {
        const auto &t = tracks[i];
        const int sid = identity_result.SemanticIdForRawTrack(t.id);
        tracks_csv << frame_idx << "," << i << "," << t.id << "," << sid << ","
                   << static_cast<int>(t.class_id) << "," << std::fixed << std::setprecision(6) << t.confidence
                   << "," << t.bbox.x << "," << t.bbox.y << "," << t.bbox.width << "," << t.bbox.height << ","
                   << (t.is_confirmed ? "1" : "0") << "," << t.time_since_update << ","
                   << t.association.stage << "," << t.association.fused_cost << "," << t.association.iou << ","
                   << t.association.motion_dist << "," << t.association.app_dist << ","
                   << (t.association.appearance_used ? "1" : "0") << ","
                   << (t.low_score_update ? "1" : "0") << "," << (t.just_recovered ? "1" : "0") << ","
                   << (t.association.passed_final_cost_gate ? "1" : "0") << ","
                   << t.association.reject_reason << "," << (t.occlusion_suspect ? "1" : "0") << "\n";
      }
    }
    if (hypotheses_csv.is_open()) {
      for (std::size_t i = 0; i < tracklet_hypotheses.size(); ++i) {
        const auto &h = tracklet_hypotheses[i];
        hypotheses_csv << frame_idx << "," << i << "," << TrackletHypothesisStatusToCsv(h.status) << ","
                       << h.raw_track_id << "," << static_cast<int>(h.class_id) << ","
                       << std::fixed << std::setprecision(6) << h.confidence << ","
                       << h.bbox.x << "," << h.bbox.y << "," << h.bbox.width << "," << h.bbox.height << ","
                       << h.candidate_reason << "," << h.related_raw_track_id.value_or(-1) << ","
                       << h.association.stage << "," << h.association.fused_cost << "," << h.association.iou << ","
                       << h.association.motion_dist << "," << h.association.app_dist << ","
                       << (h.association.appearance_used ? "1" : "0") << ","
                       << (h.association.passed_final_cost_gate ? "1" : "0") << ","
                       << h.association.reject_reason << "\n";
      }
    }
    if (phase3_shadow_csv.is_open()) {
      const auto &rows = identity_manager.LastPhase3ShadowDebugRows();
      for (std::size_t i = 0; i < rows.size(); ++i) {
        const auto &row = rows[i];
        const int frame_for_csv = row.frame_idx >= 0 ? row.frame_idx : static_cast<int>(frame_idx);
        phase3_shadow_csv << frame_for_csv << "," << row.event_idx << "," << row.event_type << ","
                          << row.group_id << "," << row.semantic_ids << "," << row.carrier_semantic_id << ","
                          << row.carrier_raw_track_id << "," << row.candidate_raw_track_id << ","
                          << row.candidate_semantic_id << "," << std::fixed << std::setprecision(6)
                          << row.candidate_confidence << "," << row.candidate_bbox.x << ","
                          << row.candidate_bbox.y << "," << row.candidate_bbox.width << ","
                          << row.candidate_bbox.height << "," << row.reason << "," << row.related_raw_track_id
                          << "," << row.hypothesis_status << "," << row.candidate_stable_frames << ","
                          << row.group_age_frames << "," << row.group_last_update_frame << ","
                          << row.decision_app_cost << "," << row.decision_geo_cost << ","
                          << row.decision_time_cost << "," << row.decision_final_score << ","
                          << row.decision_margin << "," << (row.decision_selected ? "1" : "0") << ","
                          << (row.decision_accepted ? "1" : "0") << "," << row.pairwise_selected_pairs
                          << "," << row.pairwise_alternate_pairs << "," << row.pairwise_selected_final_cost
                          << "," << row.pairwise_alternate_final_cost << "," << row.pairwise_selected_app_cost
                          << "," << row.pairwise_alternate_app_cost << "," << row.pairwise_margin << ","
                          << (row.pairwise_appearance_override ? "1" : "0") << "\n";
      }
    }
    if (identities_csv.is_open()) {
      for (const auto &identity : identity_result.identities) {
        const int raw_id = identity.supporting_raw_track_id.value_or(-1);
        identities_csv << frame_idx << "," << identity.semantic_id << ","
                       << vision_demo_host::IdentityStateToString(identity.state) << ","
                       << (identity.visible ? "1" : "0") << "," << raw_id << ","
                       << static_cast<int>(identity.class_id) << "," << std::fixed << std::setprecision(6)
                       << identity.confidence << "," << identity.bbox.x << "," << identity.bbox.y << ","
                       << identity.bbox.width << "," << identity.bbox.height << "," << identity.missing_frames
                       << "," << (identity.primary ? "1" : "0") << ","
                       << (identity.occlusion_suspect ? "1" : "0") << ","
                       << (identity.low_score_update ? "1" : "0") << ","
                       << (identity.just_recovered ? "1" : "0") << "," << identity.assignment.stage << ","
                       << (identity.assignment.accepted ? "1" : "0") << ","
                       << identity.assignment.reject_reason << "\n";
      }
    }

    if (opt.overlay_record && !overlay_writer.isOpened() && !overlay_writer_open_failed) {
      const int fourcc = cv::VideoWriter::fourcc('F', 'F', 'V', '1');
      const double src_fps = cap.get(cv::CAP_PROP_FPS);
      const double write_fps = (src_fps > 1.0 && src_fps < 120.0) ? src_fps : 25.0;
      if (!overlay_writer.open(overlay_plan.output_path.string(), fourcc, write_fps, frame.size(), true)) {
        overlay_writer_open_failed = true;
        m.error = "Failed to open FFV1 overlay writer: " + overlay_plan.output_path.string();
      } else {
        std::cout << "  [offline_eval] FFV1 overlay writer opened: " << overlay_plan.output_path
                  << " fps=" << write_fps << " size=" << frame.cols << "x" << frame.rows << std::endl;
      }
    }

    if (opt.overlay_preview || overlay_writer.isOpened()) {
      cv::Mat canvas = frame.clone();
      DrawEvalOverlay(&canvas, tracks, identity_result, primary, frame_idx, detections.size(),
                      primary_mgr.LastDecisionReason(), primary_mgr.LastRejectReason());
      if (opt.overlay_preview) {
        if (!preview_window_open) {
          try {
            cv::namedWindow(kOverlayWindowName, cv::WINDOW_NORMAL);
            preview_window_open = true;
          } catch (const cv::Exception &exception) {
            m.error = "Failed to create overlay preview window: " + std::string(exception.what());
          }
        }
        if (preview_window_open) {
          try {
            cv::imshow(kOverlayWindowName, canvas);
            (void)cv::waitKey(1);
          } catch (const cv::Exception &exception) {
            m.error = "Failed to render overlay preview: " + std::string(exception.what());
          }
        }
      }
      if (overlay_writer.isOpened()) {
        overlay_writer.write(canvas);
      }
    }
    frame_idx++;
  }
  const auto t1 = std::chrono::steady_clock::now();

  const double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0;
  m.avg_fps = (elapsed > 1e-6) ? static_cast<double>(m.total_frames) / elapsed : 0.0;

  if (preview_window_open) {
    cv::destroyWindow(kOverlayWindowName);
  }

  const auto replay_validation = vision_demo_host::tools::ValidateOfflineEvalReplay(input, m.total_frames);
  if (!replay_validation.ok && m.error.empty()) {
    m.error = replay_validation.error;
  }

  if (!m.error.empty()) {
    m.ok = false;
    return m;
  }

  m.ok = true;
  return m;
}

}  // namespace

int main(int argc, char **argv) {
  Options opt;
  std::string arg_error;
  if (!ParseArgs(argc, argv, &opt, &arg_error)) {
    if (!arg_error.empty()) {
      std::cerr << "Argument error: " << arg_error << std::endl;
      PrintUsage();
      return 2;
    }
    return 0;
  }

  const std::filesystem::path run_dir = opt.results_root / (opt.run_name + "_" + TimestampCompactNow());
  const bool use_explicit_video = !opt.explicit_video_path.empty();
  const std::size_t requested_input_count = use_explicit_video ? 1U : opt.datasets.size();
  std::vector<EvaluationRequest> requests;
  requests.reserve(requested_input_count);
  for (std::size_t i = 0; i < requested_input_count; ++i) {
    EvaluationRequest request;
    request.index = i + 1U;
    request.name = use_explicit_video ? opt.explicit_video_path.parent_path().filename().string() : opt.datasets[i];
    request.dataset_dir = use_explicit_video ? opt.explicit_video_path.parent_path() : opt.recordings_root / request.name;
    request.explicit_video_path = use_explicit_video ? opt.explicit_video_path : std::filesystem::path{};
    std::ostringstream short_name_ss;
    short_name_ss << "s" << std::setfill('0') << std::setw(2) << request.index;
    request.short_name = short_name_ss.str();
    requests.push_back(std::move(request));
  }

  // Reject a result root inside a valid source before creating the run directory.
  // This protects clean capture datasets even when every output switch is disabled.
  for (const auto &request : requests) {
    vision_demo_host::tools::OfflineEvalInput source_boundary;
    source_boundary.dataset_directory = request.dataset_dir;
    const auto preflight_result_dir =
        run_dir / (opt.short_dataset_dir_names ? request.short_name : request.name);
    const auto artifact_plan = vision_demo_host::tools::PlanOfflineEvalOverlayArtifacts(
        source_boundary, preflight_result_dir, false, opt.overlay_video_name);
    if (!artifact_plan.ok) {
      std::cerr << "Result directory error: " << artifact_plan.error << std::endl;
      return 2;
    }
  }

  std::filesystem::create_directories(run_dir);
  std::vector<DatasetMetrics> all_results;
  all_results.reserve(requested_input_count);
  std::ofstream dataset_map_csv(run_dir / "dataset_dir_map.csv");
  dataset_map_csv << "index,short_dir,dataset_name,dataset_dir,source_video_path,source_kind\n";

  std::cout << "[offline_eval] run_dir: " << run_dir << std::endl;
  std::cout << "[offline_eval] overlay_mode="
            << vision_demo_host::tools::OfflineEvalOverlayModeToString(
                   vision_demo_host::tools::OfflineEvalOverlayModeFor(opt.overlay_preview, opt.overlay_record))
            << " overlay_video_name=" << opt.overlay_video_name << std::endl;
  std::cout << "[offline_eval] det_thresholds raw=" << opt.det_raw_conf_threshold
            << " person=" << opt.det_person_conf_threshold
            << " car=" << opt.det_car_conf_threshold << std::endl;
  std::cout << "[offline_eval] tracker_gmc_enabled="
            << (opt.tracker_gmc_enabled ? "true" : "false") << std::endl;
  std::cout << "[offline_eval] target_lost_threshold_frames=" << opt.target_lost_threshold_frames << std::endl;
  bool all_ok = true;
  for (const auto &request : requests) {
    const std::filesystem::path out_dir =
        run_dir / (opt.short_dataset_dir_names ? request.short_name : request.name);
    std::filesystem::create_directories(out_dir);
    std::cout << "[offline_eval] processing[" << request.short_name << "]: " << request.dataset_dir;
    if (!request.explicit_video_path.empty()) {
      std::cout << " video=" << request.explicit_video_path;
    }
    std::cout << std::endl;
    DatasetMetrics m = EvaluateOne(opt, request.dataset_dir, out_dir, request.explicit_video_path);
    dataset_map_csv << request.index << "," << request.short_name << "," << request.name << ","
                    << request.dataset_dir.string() << "," << m.source_video_path.string() << ","
                    << m.source_kind << "\n";
    WriteDatasetJson(out_dir / "summary.json", m);
    WriteDatasetMd(out_dir / "summary.md", m);
    const auto identity_metrics = vision_demo_host::tools::BuildIdentityOfflineMetrics(out_dir, request.name);
    std::string metrics_error;
    if (!vision_demo_host::tools::WriteIdentityOfflineMetricsFiles(out_dir, identity_metrics, &metrics_error)) {
      std::cout << "  [offline_eval] warning: identity metrics write failed: " << metrics_error << std::endl;
    }
    all_results.push_back(m);

    if (m.ok) {
      std::cout << "  ok: frames=" << m.total_frames << " avg_fps=" << std::fixed << std::setprecision(2) << m.avg_fps
                << " locked_ratio=" << std::setprecision(2) << Ratio(m.locked_frames, m.total_frames) * 100.0 << "%"
                << std::endl;
    } else {
      all_ok = false;
      std::cout << "  failed: " << m.error << std::endl;
    }
  }

  WriteGlobalSummary(run_dir / "global_summary.md", all_results);
  std::cout << "[offline_eval] global summary: " << (run_dir / "global_summary.md") << std::endl;
  return all_ok ? 0 : 1;
}
