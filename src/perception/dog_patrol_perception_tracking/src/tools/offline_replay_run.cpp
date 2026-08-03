#include "vision_demo_host/tools/offline_replay_run.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "vision_demo_host/modules/det_filter.hpp"
#include "vision_demo_host/modules/identity_manager.hpp"
#include "vision_demo_host/modules/mot_tracker.hpp"
#include "vision_demo_host/modules/perception_config_materializer.hpp"
#include "vision_demo_host/modules/preprocess_infer.hpp"
#include "vision_demo_host/modules/primary_recovery_debug.hpp"
#include "vision_demo_host/modules/primary_target_manager.hpp"
#include "vision_demo_host/tools/identity_offline_metrics.hpp"
#include "vision_demo_host/tools/offline_eval_input.hpp"
#include "vision_demo_host/tools/offline_eval_schema.hpp"
#include "vision_demo_host/types.hpp"

namespace vision_demo_host::tools {

OfflineReplayRun::TrackerConfig::TrackerConfig() {
  const PerceptionConfigMaterializer::TrackerInput defaults;
  config_path = "/path/to/my_workplace/vision_demo_ws/src/vision_demo_host/config/bot_sort.yaml";
  gmc_enabled = defaults.gmc_enabled;
  reid_backend = defaults.reid_backend;
  reid_model_path = defaults.reid_model_path;
  reid_input_width = defaults.reid_input_width;
  reid_input_height = defaults.reid_input_height;
}

OfflineReplayRun::IdentityConfig::IdentityConfig() {
  const PerceptionConfigMaterializer::IdentityInput defaults;
  target_lost_threshold_frames = defaults.target_lost_threshold_frames;
  feat_bank_size = defaults.feat_bank_size;
  recover_sim_thresh_strict = defaults.recover_sim_thresh_strict;
  recover_sim_thresh_relaxed = defaults.recover_sim_thresh_relaxed;
  recover_relaxed_max_missing_frames = defaults.recover_relaxed_max_missing_frames;
  occlusion_protect_frames = defaults.occlusion_protect_frames;
  missing_assign_min_area_ratio = defaults.missing_assign_min_area_ratio;
  missing_assign_max_area_ratio = defaults.missing_assign_max_area_ratio;
  missing_assign_max_center_dist_norm = defaults.missing_assign_max_center_dist_norm;
  missing_assign_max_app_cost = defaults.missing_assign_max_app_cost;
  overlap_iou_freeze = defaults.overlap_iou_freeze;
  split_stable_frames = defaults.split_stable_frames;
  merge_hold_frames = defaults.merge_hold_frames;
  app_w = defaults.app_w;
  geo_w = defaults.geo_w;
  time_w = defaults.time_w;
  active_assign_max_cost = defaults.active_assign_max_cost;
  recovery_max_cost = defaults.recovery_max_cost;
  raw_continuity_max_cost = defaults.raw_continuity_max_cost;
  min_assignment_margin = defaults.min_assignment_margin;
  stable_frames_before_feature_update = defaults.stable_frames_before_feature_update;
  merged_requires_overlap = defaults.merged_requires_overlap;
  reid_enable = defaults.reid_enable;
  reid_backend = defaults.reid_backend;
  reid_model_path = defaults.reid_model_path;
  reid_input_width = defaults.reid_input_width;
  reid_input_height = defaults.reid_input_height;
}

namespace {

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

std::string TrackletHypothesisStatusToCsv(const TrackletHypothesisStatus status) {
  switch (status) {
    case TrackletHypothesisStatus::kTracked:
      return "tracked";
    case TrackletHypothesisStatus::kTentative:
      return "tentative";
    case TrackletHypothesisStatus::kLostPrediction:
      return "lost_prediction";
    case TrackletHypothesisStatus::kSuppressedDuplicateCandidate:
      return "suppressed_duplicate_candidate";
    case TrackletHypothesisStatus::kSplitCandidate:
      return "split_candidate";
    case TrackletHypothesisStatus::kLowQualityCandidate:
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

void DrawEvalOverlay(cv::Mat *canvas, const std::vector<Track> &tracks,
                     const IdentityManagerResult &identity_result,
                     const PrimaryTargetResult &primary,
                     const std::size_t frame_idx, const std::size_t det_count,
                     const std::string &primary_decision_reason,
                     const std::string &primary_reject_reason) {
  const int primary_semantic_id = primary.primary_target_id;

  auto find_identity_by_raw = [&](const int raw_track_id) -> const IdentityObservation * {
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
    label << "id=" << semantic_id << " " << IdentityStateToString(identity->state)
          << " raw=" << track.id;
    cv::putText(*canvas, label.str(), CompactOverlayTrackLabelPoint(canvas->size(), track.bbox),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
  }

  std::ostringstream line1;
  line1 << "frame=" << frame_idx << " det=" << det_count << " tracks=" << tracks.size();
  const std::string line2 = BuildPrimaryOverlayLine(
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

DatasetMetrics EvaluateOne(const OfflineReplayRun::Request &request,
                           const std::filesystem::path &dataset_dir,
                           const std::filesystem::path &result_dir,
                           const std::filesystem::path &explicit_video_path = {}) {
  DatasetMetrics m;
  m.dataset_name = dataset_dir.filename().string();
  m.dataset_dir = dataset_dir;

  const auto input_discovery = DiscoverOfflineEvalInput({dataset_dir, explicit_video_path});
  if (!input_discovery.ok) {
    m.error = input_discovery.error;
    return m;
  }
  const auto &input = input_discovery.input;
  const std::filesystem::path &video_path = input.video_path;
  m.source_video_path = video_path;
  m.source_kind = OfflineEvalSourceKindToString(input.source_kind);
  m.overlay_mode = OfflineEvalOverlayModeToString(
      OfflineEvalOverlayModeFor(request.output.overlay_preview, request.output.overlay_record));
  m.capture_metadata_present = input.capture.has_value();
  if (input.capture.has_value()) {
    m.capture_metadata_written_frames = input.capture->written_frames;
    m.timestamp_rows = input.timestamp_validation.rows;
  }

  const auto overlay_plan = PlanOfflineEvalOverlayArtifacts(
      input, result_dir, request.output.overlay_record, request.output.overlay_video_name);
  if (!overlay_plan.ok) {
    m.error = overlay_plan.error;
    return m;
  }

  PreprocessInfer::Config infer_cfg;
  infer_cfg.detector_runtime_path = request.detector.engine_path;
  infer_cfg.raw_conf_threshold = std::max(0.0F, request.detector.raw_conf_threshold);
  infer_cfg.input_width = 640;
  infer_cfg.input_height = 640;
  infer_cfg.enable_fake_detection = false;
  PreprocessInfer infer(infer_cfg);

  std::string err;
  if (!infer.Initialize(&err)) {
    m.error = "preprocess_infer init failed: " + err;
    return m;
  }

  DetFilter::Config filter_cfg;
  filter_cfg.person_conf_threshold = std::max(0.0F, request.detector.person_conf_threshold);
  filter_cfg.car_conf_threshold = std::max(0.0F, request.detector.car_conf_threshold);
  DetFilter det_filter(filter_cfg);

  PerceptionConfigMaterializer::TrackerInput tracker_input;
  tracker_input.config_path = request.tracker.config_path;
  tracker_input.gmc_enabled = request.tracker.gmc_enabled;
  tracker_input.reid_backend = request.tracker.reid_backend;
  tracker_input.reid_model_path = request.tracker.reid_model_path;
  tracker_input.reid_input_width = request.tracker.reid_input_width;
  tracker_input.reid_input_height = request.tracker.reid_input_height;
  PerceptionConfigMaterializer::Diagnostics tracker_config_diagnostics;
  const auto tracker_cfg =
      PerceptionConfigMaterializer::MaterializeTrackerConfig(tracker_input, &tracker_config_diagnostics);
  MotTracker tracker(tracker_cfg);
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

  PrimaryTargetManager::Config target_cfg;
  target_cfg.lost_threshold_frames = request.identity.target_lost_threshold_frames;
  target_cfg.min_person_area_px = 1000.0F;
  PrimaryTargetManager primary_mgr(target_cfg);

  PerceptionConfigMaterializer::IdentityInput sid_input;
  sid_input.target_lost_threshold_frames = request.identity.target_lost_threshold_frames;
  sid_input.feat_bank_size = request.identity.feat_bank_size;
  sid_input.recover_sim_thresh_strict = request.identity.recover_sim_thresh_strict;
  sid_input.recover_sim_thresh_relaxed = request.identity.recover_sim_thresh_relaxed;
  sid_input.recover_relaxed_max_missing_frames = request.identity.recover_relaxed_max_missing_frames;
  sid_input.occlusion_protect_frames = request.identity.occlusion_protect_frames;
  sid_input.missing_assign_min_area_ratio = request.identity.missing_assign_min_area_ratio;
  sid_input.missing_assign_max_area_ratio = request.identity.missing_assign_max_area_ratio;
  sid_input.missing_assign_max_center_dist_norm = request.identity.missing_assign_max_center_dist_norm;
  sid_input.missing_assign_max_app_cost = request.identity.missing_assign_max_app_cost;
  sid_input.overlap_iou_freeze = request.identity.overlap_iou_freeze;
  sid_input.split_stable_frames = request.identity.split_stable_frames;
  sid_input.merge_hold_frames = request.identity.merge_hold_frames;
  sid_input.app_w = request.identity.app_w;
  sid_input.geo_w = request.identity.geo_w;
  sid_input.time_w = request.identity.time_w;
  sid_input.active_assign_max_cost = request.identity.active_assign_max_cost;
  sid_input.recovery_max_cost = request.identity.recovery_max_cost;
  sid_input.raw_continuity_max_cost = request.identity.raw_continuity_max_cost;
  sid_input.min_assignment_margin = request.identity.min_assignment_margin;
  sid_input.stable_frames_before_feature_update = request.identity.stable_frames_before_feature_update;
  sid_input.merged_requires_overlap = request.identity.merged_requires_overlap;
  sid_input.reid_enable = request.identity.reid_enable;
  sid_input.reid_backend = request.identity.reid_backend;
  sid_input.reid_model_path = request.identity.reid_model_path;
  sid_input.reid_input_width = request.identity.reid_input_width;
  sid_input.reid_input_height = request.identity.reid_input_height;
  PerceptionConfigMaterializer::Diagnostics identity_config_diagnostics;
  const auto sid_cfg =
      PerceptionConfigMaterializer::MaterializeIdentityConfig(sid_input, &identity_config_diagnostics);
  if (identity_config_diagnostics.identity_reid_forced) {
    std::cout << "[offline_eval] reid is mandatory; override --sid-reid-enable=false to true" << std::endl;
  }
  IdentityManager identity_manager(sid_cfg);
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
  if (request.output.save_frame_csv) {
    frame_csv.open(result_dir / "per_frame.csv");
    frame_csv << PerFrameCsvHeader() << "\n";

    det_raw_csv.open(result_dir / "det_raw.csv");
    det_raw_csv << "frame_idx,det_idx,class_id,score,x,y,w,h\n";
    det_filtered_csv.open(result_dir / "det_filtered.csv");
    det_filtered_csv << "frame_idx,det_idx,class_id,score,x,y,w,h\n";
  }
  if (request.output.save_sid_scores) {
    sid_scores_csv.open(result_dir / "sid_scores.csv");
    sid_scores_csv << SidScoresCsvHeader() << "\n";
  }
  if (request.output.save_tracks_csv) {
    tracks_csv.open(result_dir / "tracks.csv");
    tracks_csv << "frame_idx,track_idx,raw_track_id,semantic_id,class_id,score,x,y,w,h,is_confirmed,time_since_update,"
                  "assoc_stage,assoc_cost,assoc_iou,assoc_motion_dist,assoc_app_dist,assoc_appearance_used,low_score_update,just_recovered,assoc_final_gate,assoc_reject_reason,occlusion_suspect\n";
    hypotheses_csv.open(result_dir / "tracklet_hypotheses.csv");
    hypotheses_csv << TrackletHypothesesCsvHeader() << "\n";
    phase3_shadow_csv.open(result_dir / "phase3_shadow_state.csv");
    phase3_shadow_csv << Phase3ShadowStateCsvHeader() << "\n";
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
  std::optional<PrimaryState> prev_state;

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
        TrackletObservationsFromTracks(tracks), tracklet_hypotheses, primary_prev, &frame);
    const auto primary = primary_mgr.Update(identity_result.identities);

    const int primary_semantic_id = primary.primary_target_id;
    const int raw_track_id = primary.raw_track_id;

    m.total_frames++;
    if (!detections.empty()) {
      m.det_positive_frames++;
    }
    if (!tracks.empty()) {
      m.tracks_positive_frames++;
    }
    if (primary.state == PrimaryState::kLocked) {
      m.locked_frames++;
    } else if (primary.state == PrimaryState::kOccluded) {
      m.occluded_frames++;
    } else if (primary.state == PrimaryState::kLost) {
      m.lost_frames++;
    }

    if (prev_primary_semantic_id.has_value() && primary_semantic_id > 0 &&
        prev_primary_semantic_id.value() > 0 && primary_semantic_id != prev_primary_semantic_id.value()) {
      m.primary_switch_count++;
    }
    if (prev_state.has_value() && prev_state.value() == PrimaryState::kLocked &&
        primary.state == PrimaryState::kLost) {
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
                << PrimaryStateToString(primary.state) << "," << primary_semantic_id << ","
                << raw_track_id << ","
                << IdentityModeToString(identity_manager.CurrentMode()) << ","
                << (identity_manager.IsFeatureUpdateFrozen() ? "1" : "0") << ","
                << sid_list.str() << "," << primary_mgr.LastDecisionReason() << ","
                << primary_mgr.LastRejectReason() << ","
                << PrimaryRecoveryReasonToken(primary, identity_result,
                                               primary_mgr.LastDecisionReason(),
                                               primary_mgr.LastRejectReason()) << ","
                << PrimarySupportingRawTrackIdDebug(primary, identity_result) << "\n";
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
        sid_scores_csv << row.frame_idx << "," << IdentityModeToString(row.mode) << ","
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
                       << IdentityStateToString(identity.state) << ","
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

    if (request.output.overlay_record && !overlay_writer.isOpened() && !overlay_writer_open_failed) {
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

    if (request.output.overlay_preview || overlay_writer.isOpened()) {
      cv::Mat canvas = frame.clone();
      DrawEvalOverlay(&canvas, tracks, identity_result, primary, frame_idx, detections.size(),
                      primary_mgr.LastDecisionReason(), primary_mgr.LastRejectReason());
      if (request.output.overlay_preview) {
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

  const auto replay_validation = ValidateOfflineEvalReplay(input, m.total_frames);
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

std::vector<EvaluationRequest> BuildEvaluationRequests(const OfflineReplayRun::Request &request) {
  const bool use_explicit_video = !request.explicit_video_path.empty();
  const std::size_t requested_input_count = use_explicit_video ? 1U : request.datasets.size();
  std::vector<EvaluationRequest> requests;
  requests.reserve(requested_input_count);
  for (std::size_t i = 0; i < requested_input_count; ++i) {
    EvaluationRequest evaluation_request;
    evaluation_request.index = i + 1U;
    evaluation_request.name = use_explicit_video ? request.explicit_video_path.parent_path().filename().string()
                                                 : request.datasets[i];
    evaluation_request.dataset_dir = use_explicit_video ? request.explicit_video_path.parent_path()
                                                        : request.recordings_root / evaluation_request.name;
    evaluation_request.explicit_video_path = use_explicit_video ? request.explicit_video_path
                                                                : std::filesystem::path{};
    std::ostringstream short_name_ss;
    short_name_ss << "s" << std::setfill('0') << std::setw(2) << evaluation_request.index;
    evaluation_request.short_name = short_name_ss.str();
    requests.push_back(std::move(evaluation_request));
  }
  return requests;
}

}  // namespace

OfflineReplayRun::Result OfflineReplayRun::Run(const Request &request) {
  Result result;
  result.run_dir = request.results_root / (request.run_name + "_" + TimestampCompactNow());
  const auto requests = BuildEvaluationRequests(request);

  // Reject a result root inside a valid source before creating the run directory.
  // This protects clean capture datasets even when every output switch is disabled.
  for (const auto &evaluation_request : requests) {
    OfflineEvalInput source_input;
    source_input.dataset_directory = evaluation_request.dataset_dir;
    const auto preflight_result_dir =
        result.run_dir / (request.output.short_dataset_dir_names ? evaluation_request.short_name
                                                                  : evaluation_request.name);
    const auto artifact_plan = PlanOfflineEvalOverlayArtifacts(
        source_input, preflight_result_dir, false, request.output.overlay_video_name);
    if (!artifact_plan.ok) {
      std::cerr << "Result directory error: " << artifact_plan.error << std::endl;
      result.exit_code = 2;
      return result;
    }
  }

  std::filesystem::create_directories(result.run_dir);
  std::vector<DatasetMetrics> all_results;
  all_results.reserve(requests.size());
  std::ofstream dataset_map_csv(result.run_dir / "dataset_dir_map.csv");
  dataset_map_csv << "index,short_dir,dataset_name,dataset_dir,source_video_path,source_kind\n";

  std::cout << "[offline_eval] run_dir: " << result.run_dir << std::endl;
  std::cout << "[offline_eval] overlay_mode="
            << OfflineEvalOverlayModeToString(
                   OfflineEvalOverlayModeFor(request.output.overlay_preview, request.output.overlay_record))
            << " overlay_video_name=" << request.output.overlay_video_name << std::endl;
  std::cout << "[offline_eval] det_thresholds raw=" << request.detector.raw_conf_threshold
            << " person=" << request.detector.person_conf_threshold
            << " car=" << request.detector.car_conf_threshold << std::endl;
  std::cout << "[offline_eval] tracker_gmc_enabled="
            << (request.tracker.gmc_enabled ? "true" : "false") << std::endl;
  std::cout << "[offline_eval] target_lost_threshold_frames="
            << request.identity.target_lost_threshold_frames << std::endl;

  bool all_ok = true;
  for (const auto &evaluation_request : requests) {
    const std::filesystem::path out_dir =
        result.run_dir / (request.output.short_dataset_dir_names ? evaluation_request.short_name
                                                                 : evaluation_request.name);
    std::filesystem::create_directories(out_dir);
    std::cout << "[offline_eval] processing[" << evaluation_request.short_name << "]: "
              << evaluation_request.dataset_dir;
    if (!evaluation_request.explicit_video_path.empty()) {
      std::cout << " video=" << evaluation_request.explicit_video_path;
    }
    std::cout << std::endl;
    DatasetMetrics metrics = EvaluateOne(request, evaluation_request.dataset_dir, out_dir,
                                         evaluation_request.explicit_video_path);
    dataset_map_csv << evaluation_request.index << "," << evaluation_request.short_name << ","
                    << evaluation_request.name << "," << evaluation_request.dataset_dir.string() << ","
                    << metrics.source_video_path.string() << "," << metrics.source_kind << "\n";
    WriteDatasetJson(out_dir / "summary.json", metrics);
    WriteDatasetMd(out_dir / "summary.md", metrics);
    const auto identity_metrics = BuildIdentityOfflineMetrics(out_dir, evaluation_request.name);
    std::string metrics_error;
    if (!WriteIdentityOfflineMetricsFiles(out_dir, identity_metrics, &metrics_error)) {
      std::cout << "  [offline_eval] warning: identity metrics write failed: " << metrics_error << std::endl;
    }
    all_results.push_back(metrics);

    if (metrics.ok) {
      std::cout << "  ok: frames=" << metrics.total_frames << " avg_fps=" << std::fixed
                << std::setprecision(2) << metrics.avg_fps << " locked_ratio="
                << std::setprecision(2) << Ratio(metrics.locked_frames, metrics.total_frames) * 100.0 << "%"
                << std::endl;
    } else {
      all_ok = false;
      std::cout << "  failed: " << metrics.error << std::endl;
    }
  }

  WriteGlobalSummary(result.run_dir / "global_summary.md", all_results);
  std::cout << "[offline_eval] global summary: " << (result.run_dir / "global_summary.md") << std::endl;
  result.all_ok = all_ok;
  result.exit_code = all_ok ? 0 : 1;
  return result;
}

}  // namespace vision_demo_host::tools
