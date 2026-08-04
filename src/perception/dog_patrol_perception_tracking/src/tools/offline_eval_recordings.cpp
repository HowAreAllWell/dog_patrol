#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "dog_patrol_perception_tracking/modules/perception_config_materializer.hpp"
#include "dog_patrol_perception_tracking/tools/identity_offline_metrics.hpp"
#include "dog_patrol_perception_tracking/tools/offline_eval_schema.hpp"
#include "dog_patrol_perception_tracking/tools/offline_replay_run.hpp"

namespace {

using PerceptionConfigMaterializer = dog_patrol_perception_tracking::PerceptionConfigMaterializer;

struct Options {
  std::filesystem::path recordings_root{"data/captures"};
  std::filesystem::path results_root{"data/eval_results"};
  std::string run_name{"oe"};
  std::string detector_engine_path;
  float det_raw_conf_threshold{0.10F};
  float det_person_conf_threshold{0.10F};
  float det_car_conf_threshold{0.10F};
  std::string tracker_config_path;
  bool tracker_gmc_enabled{PerceptionConfigMaterializer::TrackerInput{}.gmc_enabled};
  std::string tracker_reid_backend{PerceptionConfigMaterializer::kDefaultReidBackend};
  std::string tracker_reid_model_path{};
  int tracker_reid_input_width{PerceptionConfigMaterializer::kDefaultReidInputWidth};
  int tracker_reid_input_height{PerceptionConfigMaterializer::kDefaultReidInputHeight};
  bool save_frame_csv{true};
  bool save_sid_scores{true};
  bool save_tracks_csv{true};
  bool overlay_preview{false};
  bool overlay_record{false};
  bool short_dataset_dir_names{true};
  std::string overlay_video_name{"eval_overlay.mkv"};
  std::filesystem::path explicit_video_path;
  int target_lost_threshold_frames{PerceptionConfigMaterializer::IdentityInput{}.target_lost_threshold_frames};
  int sid_feat_bank_size{PerceptionConfigMaterializer::IdentityInput{}.feat_bank_size};
  float sid_recover_sim_thresh_strict{PerceptionConfigMaterializer::IdentityInput{}.recover_sim_thresh_strict};
  float sid_recover_sim_thresh_relaxed{PerceptionConfigMaterializer::IdentityInput{}.recover_sim_thresh_relaxed};
  int sid_recover_relaxed_max_missing_frames{
      PerceptionConfigMaterializer::IdentityInput{}.recover_relaxed_max_missing_frames};
  int sid_occlusion_protect_frames{PerceptionConfigMaterializer::IdentityInput{}.occlusion_protect_frames};
  float sid_missing_assign_min_area_ratio{
      PerceptionConfigMaterializer::IdentityInput{}.missing_assign_min_area_ratio};
  float sid_missing_assign_max_area_ratio{
      PerceptionConfigMaterializer::IdentityInput{}.missing_assign_max_area_ratio};
  float sid_missing_assign_max_center_dist_norm{
      PerceptionConfigMaterializer::IdentityInput{}.missing_assign_max_center_dist_norm};
  float sid_missing_assign_max_app_cost{
      PerceptionConfigMaterializer::IdentityInput{}.missing_assign_max_app_cost};
  float sid_overlap_iou_freeze{PerceptionConfigMaterializer::IdentityInput{}.overlap_iou_freeze};
  int sid_split_stable_frames{PerceptionConfigMaterializer::IdentityInput{}.split_stable_frames};
  int sid_merge_hold_frames{PerceptionConfigMaterializer::IdentityInput{}.merge_hold_frames};
  float sid_app_w{PerceptionConfigMaterializer::IdentityInput{}.app_w};
  float sid_geo_w{PerceptionConfigMaterializer::IdentityInput{}.geo_w};
  float sid_time_w{PerceptionConfigMaterializer::IdentityInput{}.time_w};
  float sid_active_assign_max_cost{PerceptionConfigMaterializer::IdentityInput{}.active_assign_max_cost};
  float sid_recovery_max_cost{PerceptionConfigMaterializer::IdentityInput{}.recovery_max_cost};
  float sid_raw_continuity_max_cost{PerceptionConfigMaterializer::IdentityInput{}.raw_continuity_max_cost};
  float sid_min_assignment_margin{PerceptionConfigMaterializer::IdentityInput{}.min_assignment_margin};
  int sid_stable_frames_before_feature_update{
      PerceptionConfigMaterializer::IdentityInput{}.stable_frames_before_feature_update};
  bool sid_merged_requires_overlap{PerceptionConfigMaterializer::IdentityInput{}.merged_requires_overlap};
  bool sid_reid_enable{PerceptionConfigMaterializer::IdentityInput{}.reid_enable};  // compatibility input.
  std::string sid_reid_backend{PerceptionConfigMaterializer::kDefaultReidBackend};
  std::string sid_reid_model_path{};
  int sid_reid_input_width{PerceptionConfigMaterializer::kDefaultReidInputWidth};
  int sid_reid_input_height{PerceptionConfigMaterializer::kDefaultReidInputHeight};
  std::vector<std::string> datasets;
};

std::string Trim(const std::string &s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

const char *BoolLiteral(bool value) { return value ? "true" : "false"; }

void PrintUsage() {
  const Options defaults;
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
      << "  --tracker-gmc-enabled <true|false> (default: "
      << BoolLiteral(defaults.tracker_gmc_enabled) << ")\n"
      << "  --tracker-reid-backend <light|osnet_onnx> (default: "
      << defaults.tracker_reid_backend << ")\n"
      << "  --tracker-reid-model-path <path>      (default: \"\")\n"
      << "  --tracker-reid-input-width <n>        (default: "
      << defaults.tracker_reid_input_width << ")\n"
      << "  --tracker-reid-input-height <n>       (default: "
      << defaults.tracker_reid_input_height << ")\n"
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
      << "  --target-lost-threshold-frames <n>     (default: "
      << defaults.target_lost_threshold_frames << ")\n"
      << "  --sid-feat-bank-size <n>               (default: "
      << defaults.sid_feat_bank_size << ")\n"
      << "  --sid-recover-sim-thresh-strict <f>    (default: "
      << defaults.sid_recover_sim_thresh_strict << ")\n"
      << "  --sid-recover-sim-thresh-relaxed <f>   (default: "
      << defaults.sid_recover_sim_thresh_relaxed << ")\n"
      << "  --sid-recover-relaxed-max-missing-frames <n> (default: "
      << defaults.sid_recover_relaxed_max_missing_frames << ")\n"
      << "  --sid-occlusion-protect-frames <n>           (default: "
      << defaults.sid_occlusion_protect_frames << ")\n"
      << "  --sid-missing-assign-min-area-ratio <f>      (default: "
      << defaults.sid_missing_assign_min_area_ratio << ")\n"
      << "  --sid-missing-assign-max-area-ratio <f>      (default: "
      << defaults.sid_missing_assign_max_area_ratio << ")\n"
      << "  --sid-missing-assign-max-center-dist-norm <f> (default: "
      << defaults.sid_missing_assign_max_center_dist_norm << ")\n"
      << "  --sid-missing-assign-max-app-cost <f>  (default: "
      << defaults.sid_missing_assign_max_app_cost << ")\n"
      << "  --sid-overlap-iou-freeze <f>           (default: "
      << defaults.sid_overlap_iou_freeze << ")\n"
      << "  --sid-split-stable-frames <n>          (default: "
      << defaults.sid_split_stable_frames << ")\n"
      << "  --sid-merge-hold-frames <n>            (default: "
      << defaults.sid_merge_hold_frames << ")\n"
      << "  --sid-app-w <f>                        (default: "
      << defaults.sid_app_w << ")\n"
      << "  --sid-geo-w <f>                        (default: "
      << defaults.sid_geo_w << ")\n"
      << "  --sid-time-w <f>                       (default: "
      << defaults.sid_time_w << ")\n"
      << "  --sid-active-assign-max-cost <f>       (default: "
      << defaults.sid_active_assign_max_cost << ")\n"
      << "  --sid-recovery-max-cost <f>            (default: "
      << defaults.sid_recovery_max_cost << ")\n"
      << "  --sid-raw-continuity-max-cost <f>      (default: "
      << defaults.sid_raw_continuity_max_cost << ")\n"
      << "  --sid-min-assignment-margin <f>        (default: "
      << defaults.sid_min_assignment_margin << ")\n"
      << "  --sid-stable-frames-before-feature-update <n> (default: "
      << defaults.sid_stable_frames_before_feature_update << ")\n"
      << "  --sid-merged-requires-overlap <true|false> (default: "
      << BoolLiteral(defaults.sid_merged_requires_overlap) << ")\n"
      << "  --sid-reid-enable <true|false>         (compat-only; runtime forces true)\n"
      << "  --sid-reid-backend <light|osnet_onnx> (default: "
      << defaults.sid_reid_backend << ")\n"
      << "  --sid-reid-model-path <path>           (default: \"\")\n"
      << "  --sid-reid-input-width <n>             (default: "
      << defaults.sid_reid_input_width << ")\n"
      << "  --sid-reid-input-height <n>            (default: "
      << defaults.sid_reid_input_height << ")\n"
      << "  --help\n\n"
      << dog_patrol_perception_tracking::tools::PerFrameCsvHelp() << "\n"
      << dog_patrol_perception_tracking::tools::TrackletHypothesesCsvHelp() << "\n"
      << dog_patrol_perception_tracking::tools::Phase3ShadowStateCsvHelp() << "\n"
      << dog_patrol_perception_tracking::tools::IdentityOfflineMetricsHelp();
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
        opt->tracker_reid_input_width = std::stoi(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --tracker-reid-input-width: " + s;
        }
        return false;
      }
    } else if (arg == "--tracker-reid-input-height") {
      const std::string s = need(arg);
      try {
        opt->tracker_reid_input_height = std::stoi(s);
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
        opt->sid_reid_input_width = std::stoi(s);
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid --sid-reid-input-width: " + s;
        }
        return false;
      }
    } else if (arg == "--sid-reid-input-height") {
      const std::string s = need(arg);
      try {
        opt->sid_reid_input_height = std::stoi(s);
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
  if (opt->detector_engine_path.empty() || opt->tracker_config_path.empty()) {
    if (error != nullptr) {
      *error = "--detector-engine and --tracker-config are required local asset/config paths.";
    }
    return false;
  }
  return true;
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

  dog_patrol_perception_tracking::tools::OfflineReplayRun::Request request;
  request.recordings_root = opt.recordings_root;
  request.results_root = opt.results_root;
  request.run_name = opt.run_name;
  request.datasets = opt.datasets;
  request.explicit_video_path = opt.explicit_video_path;

  request.detector.engine_path = opt.detector_engine_path;
  request.detector.raw_conf_threshold = opt.det_raw_conf_threshold;
  request.detector.person_conf_threshold = opt.det_person_conf_threshold;
  request.detector.car_conf_threshold = opt.det_car_conf_threshold;

  request.tracker.config_path = opt.tracker_config_path;
  request.tracker.gmc_enabled = opt.tracker_gmc_enabled;
  request.tracker.reid_backend = opt.tracker_reid_backend;
  request.tracker.reid_model_path = opt.tracker_reid_model_path;
  request.tracker.reid_input_width = opt.tracker_reid_input_width;
  request.tracker.reid_input_height = opt.tracker_reid_input_height;

  request.output.save_frame_csv = opt.save_frame_csv;
  request.output.save_sid_scores = opt.save_sid_scores;
  request.output.save_tracks_csv = opt.save_tracks_csv;
  request.output.overlay_preview = opt.overlay_preview;
  request.output.overlay_record = opt.overlay_record;
  request.output.short_dataset_dir_names = opt.short_dataset_dir_names;
  request.output.overlay_video_name = opt.overlay_video_name;

  request.identity.target_lost_threshold_frames = opt.target_lost_threshold_frames;
  request.identity.feat_bank_size = opt.sid_feat_bank_size;
  request.identity.recover_sim_thresh_strict = opt.sid_recover_sim_thresh_strict;
  request.identity.recover_sim_thresh_relaxed = opt.sid_recover_sim_thresh_relaxed;
  request.identity.recover_relaxed_max_missing_frames = opt.sid_recover_relaxed_max_missing_frames;
  request.identity.occlusion_protect_frames = opt.sid_occlusion_protect_frames;
  request.identity.missing_assign_min_area_ratio = opt.sid_missing_assign_min_area_ratio;
  request.identity.missing_assign_max_area_ratio = opt.sid_missing_assign_max_area_ratio;
  request.identity.missing_assign_max_center_dist_norm = opt.sid_missing_assign_max_center_dist_norm;
  request.identity.missing_assign_max_app_cost = opt.sid_missing_assign_max_app_cost;
  request.identity.overlap_iou_freeze = opt.sid_overlap_iou_freeze;
  request.identity.split_stable_frames = opt.sid_split_stable_frames;
  request.identity.merge_hold_frames = opt.sid_merge_hold_frames;
  request.identity.app_w = opt.sid_app_w;
  request.identity.geo_w = opt.sid_geo_w;
  request.identity.time_w = opt.sid_time_w;
  request.identity.active_assign_max_cost = opt.sid_active_assign_max_cost;
  request.identity.recovery_max_cost = opt.sid_recovery_max_cost;
  request.identity.raw_continuity_max_cost = opt.sid_raw_continuity_max_cost;
  request.identity.min_assignment_margin = opt.sid_min_assignment_margin;
  request.identity.stable_frames_before_feature_update = opt.sid_stable_frames_before_feature_update;
  request.identity.merged_requires_overlap = opt.sid_merged_requires_overlap;
  request.identity.reid_enable = opt.sid_reid_enable;
  request.identity.reid_backend = opt.sid_reid_backend;
  request.identity.reid_model_path = opt.sid_reid_model_path;
  request.identity.reid_input_width = opt.sid_reid_input_width;
  request.identity.reid_input_height = opt.sid_reid_input_height;

  const auto result = dog_patrol_perception_tracking::tools::OfflineReplayRun::Run(request);
  return result.exit_code;
}
