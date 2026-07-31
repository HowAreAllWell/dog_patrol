#include "vision_demo_host/tools/offline_eval_input.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <regex>
#include <sstream>
#include <vector>

namespace vision_demo_host::tools {
namespace {

constexpr const char *kTimestampHeader =
    "capture_index,source_timestamp_ns,sdk_host_timestamp,camera_frame_number,"
    "camera_frame_number_available,device_timestamp_ticks,source_pixel_type,"
    "source_pixel_type_name,width,height,source_payload_bytes,camera_lost_packets";

std::string ToLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

bool ReadTextFile(const std::filesystem::path &path, std::string *text, std::string *error) {
  std::ifstream input(path);
  if (!input) {
    *error = "Failed to open artifact: " + path.string();
    return false;
  }
  *text = std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  if (!input.good() && !input.eof()) {
    *error = "Failed to read artifact: " + path.string();
    return false;
  }
  return true;
}

std::optional<std::string> JsonString(const std::string &text, const std::string &key) {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) {
    return std::nullopt;
  }
  return match[1].str();
}

std::optional<std::size_t> JsonUnsigned(const std::string &text, const std::string &key) {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*([0-9]+)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) {
    return std::nullopt;
  }
  try {
    return static_cast<std::size_t>(std::stoull(match[1].str()));
  } catch (...) {
    return std::nullopt;
  }
}

bool ParseUnsigned(const std::string &value, std::uint64_t *parsed) {
  if (value.empty()) {
    return false;
  }
  try {
    std::size_t consumed = 0U;
    const auto number = std::stoull(value, &consumed);
    if (consumed != value.size()) {
      return false;
    }
    *parsed = number;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseCsvLine(const std::string &line, std::vector<std::string> *columns) {
  columns->clear();
  std::string value;
  bool quoted = false;
  for (std::size_t index = 0U; index < line.size(); ++index) {
    const char ch = line[index];
    if (ch == '"') {
      if (quoted && index + 1U < line.size() && line[index + 1U] == '"') {
        value.push_back(ch);
        ++index;
      } else {
        quoted = !quoted;
      }
    } else if (ch == ',' && !quoted) {
      columns->push_back(value);
      value.clear();
    } else {
      value.push_back(ch);
    }
  }
  if (quoted) {
    return false;
  }
  columns->push_back(value);
  return true;
}

OfflineEvalTimestampValidation ValidateTimestampArtifact(const std::filesystem::path &path,
                                                          const std::size_t expected_rows) {
  OfflineEvalTimestampValidation validation;
  std::ifstream input(path);
  if (!input) {
    validation.error = "FFV1 capture metadata requires frame_timestamps.csv: " + path.string();
    return validation;
  }

  std::string line;
  if (!std::getline(input, line) || line != kTimestampHeader) {
    validation.error = "Unexpected FFV1 frame_timestamps.csv header: " + path.string();
    return validation;
  }

  std::optional<std::uint64_t> previous_capture_index;
  std::optional<std::uint64_t> previous_timestamp_ns;
  std::size_t line_number = 1U;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty()) {
      validation.error = "Empty row in frame_timestamps.csv at line " + std::to_string(line_number);
      return validation;
    }
    std::vector<std::string> columns;
    if (!ParseCsvLine(line, &columns) || columns.size() != 12U) {
      validation.error = "Malformed row in frame_timestamps.csv at line " + std::to_string(line_number);
      return validation;
    }
    std::uint64_t capture_index = 0U;
    std::uint64_t timestamp_ns = 0U;
    if (!ParseUnsigned(columns[0], &capture_index) || !ParseUnsigned(columns[1], &timestamp_ns)) {
      validation.error = "Invalid capture index or source timestamp at line " + std::to_string(line_number);
      return validation;
    }
    if (previous_capture_index.has_value() && capture_index <= *previous_capture_index) {
      validation.error = "Non-increasing capture index at line " + std::to_string(line_number);
      return validation;
    }
    if (previous_timestamp_ns.has_value() && timestamp_ns <= *previous_timestamp_ns) {
      validation.error = "Non-increasing source timestamp at line " + std::to_string(line_number);
      return validation;
    }
    previous_capture_index = capture_index;
    previous_timestamp_ns = timestamp_ns;
    ++validation.rows;
  }
  if (!input.eof()) {
    validation.error = "Failed while reading frame_timestamps.csv: " + path.string();
    return validation;
  }
  if (validation.rows != expected_rows) {
    validation.error = "frame_timestamps.csv rows=" + std::to_string(validation.rows) +
                       " disagree with metadata counts.written_frames=" + std::to_string(expected_rows);
    return validation;
  }
  validation.ok = true;
  return validation;
}

bool ResolveReplayBoundaryPath(const std::filesystem::path &path, const char *name,
                               std::filesystem::path *resolved, std::string *error) {
  std::error_code filesystem_error;
  const auto absolute_path = std::filesystem::absolute(path, filesystem_error).lexically_normal();
  if (filesystem_error) {
    *error = std::string("Unable to resolve ") + name + ": " + filesystem_error.message();
    return false;
  }
  *resolved = std::filesystem::weakly_canonical(absolute_path, filesystem_error);
  if (filesystem_error) {
    *error = std::string("Unable to canonicalize ") + name + ": " + filesystem_error.message();
    return false;
  }
  return true;
}

bool IsWithin(const std::filesystem::path &ancestor, const std::filesystem::path &candidate,
              std::string *error) {
  std::filesystem::path resolved_ancestor;
  std::filesystem::path resolved_candidate;
  if (!ResolveReplayBoundaryPath(ancestor, "source dataset", &resolved_ancestor, error) ||
      !ResolveReplayBoundaryPath(candidate, "result directory", &resolved_candidate, error)) {
    return false;
  }
  auto ancestor_it = resolved_ancestor.begin();
  auto candidate_it = resolved_candidate.begin();
  for (; ancestor_it != resolved_ancestor.end() && candidate_it != resolved_candidate.end();
       ++ancestor_it, ++candidate_it) {
    if (*ancestor_it != *candidate_it) {
      return false;
    }
  }
  return ancestor_it == resolved_ancestor.end();
}

bool IsHistoricalH264MigrationPath(const std::filesystem::path &video_path) {
  bool has_historical_dataset_component = false;
  for (const auto &component : video_path.lexically_normal()) {
    if (component == "orin_hik_h264_MOT") {
      has_historical_dataset_component = true;
      break;
    }
  }
  return has_historical_dataset_component &&
         ToLower(video_path.filename().string()) == "video.mp4";
}

}  // namespace

OfflineEvalInputDiscovery DiscoverOfflineEvalInput(const OfflineEvalInputRequest &request) {
  OfflineEvalInputDiscovery discovery;
  OfflineEvalInput input;
  input.dataset_directory = request.dataset_directory;

  if (!request.explicit_video_path.empty()) {
    input.video_path = request.explicit_video_path;
    if (input.dataset_directory.empty()) {
      input.dataset_directory = input.video_path.parent_path();
    }
  } else {
    if (input.dataset_directory.empty()) {
      discovery.error = "Dataset directory is required when --video is not supplied";
      return discovery;
    }
    const auto canonical_video = input.dataset_directory / "video.mkv";
    const auto historical_video = input.dataset_directory / "video.mp4";
    if (std::filesystem::exists(canonical_video)) {
      input.video_path = canonical_video;
    } else if (std::filesystem::exists(historical_video)) {
      input.video_path = historical_video;
    } else {
      discovery.error = "No replay video found; expected video.mkv or historical video.mp4 in " +
                        input.dataset_directory.string();
      return discovery;
    }
  }

  if (!std::filesystem::exists(input.video_path)) {
    discovery.error = "Explicit replay video does not exist: " + input.video_path.string();
    return discovery;
  }

  const auto metadata_path = input.video_path.parent_path() / "metadata.json";
  const std::string source_extension = ToLower(input.video_path.extension().string());
  const bool source_is_mkv = source_extension == ".mkv";
  if (!source_is_mkv && source_extension != ".mp4") {
    discovery.error = "Explicit replay video must be FFV1/MKV or historical MP4: " +
                      input.video_path.string();
    return discovery;
  }
  if (source_extension == ".mp4" && !IsHistoricalH264MigrationPath(input.video_path)) {
    discovery.error =
        "MP4 replay is limited to historical orin_hik_h264_MOT migration data: " +
        input.video_path.string();
    return discovery;
  }
  if (source_is_mkv && std::filesystem::exists(metadata_path)) {
    std::string metadata_text;
    if (!ReadTextFile(metadata_path, &metadata_text, &discovery.error)) {
      return discovery;
    }
    const auto state = JsonString(metadata_text, "state");
    const auto codec = JsonString(metadata_text, "codec");
    const auto container = JsonString(metadata_text, "container");
    const auto written_frames = JsonUnsigned(metadata_text, "written_frames");
    if (!state.has_value() || !codec.has_value() || !container.has_value() || !written_frames.has_value()) {
      discovery.error = "Incomplete capture metadata for replay source: " + metadata_path.string();
      return discovery;
    }
    input.capture = OfflineEvalCaptureMetadata{*state, *codec, *container, *written_frames};
    if (input.capture->state != "complete") {
      discovery.error = "Capture metadata state is not complete: " + input.capture->state;
      return discovery;
    }
    if (ToLower(input.capture->codec) != "ffv1" || ToLower(input.capture->container) != "mkv") {
      discovery.error = "Capture metadata is not canonical FFV1/MKV";
      return discovery;
    }
    input.timestamp_validation =
        ValidateTimestampArtifact(input.video_path.parent_path() / "frame_timestamps.csv", input.capture->written_frames);
    if (!input.timestamp_validation.ok) {
      discovery.error = input.timestamp_validation.error;
      return discovery;
    }
    input.source_kind = OfflineEvalSourceKind::kFfv1Capture;
  } else if (source_extension == ".mp4") {
    input.source_kind = OfflineEvalSourceKind::kHistoricalH264;
  } else {
    input.source_kind = OfflineEvalSourceKind::kExplicitVideo;
  }

  discovery.ok = true;
  discovery.input = std::move(input);
  return discovery;
}

OfflineEvalReplayValidation ValidateOfflineEvalReplay(const OfflineEvalInput &input,
                                                       const std::size_t decoded_frames) {
  OfflineEvalReplayValidation validation;
  if (input.capture.has_value() && decoded_frames != input.capture->written_frames) {
    validation.error = "decoded frame count=" + std::to_string(decoded_frames) +
                       " disagrees with capture metadata counts.written_frames=" +
                       std::to_string(input.capture->written_frames);
    return validation;
  }
  validation.ok = true;
  return validation;
}

OfflineEvalOverlayArtifactPlan PlanOfflineEvalOverlayArtifacts(
    const OfflineEvalInput &input, const std::filesystem::path &result_directory, const bool record_overlay,
    const std::string &overlay_video_name) {
  OfflineEvalOverlayArtifactPlan plan;
  std::string boundary_error;
  if (IsWithin(input.dataset_directory, result_directory, &boundary_error)) {
    plan.error = "Offline evaluation artifacts must be written outside the source dataset";
    return plan;
  }
  if (!boundary_error.empty()) {
    plan.error = boundary_error;
    return plan;
  }
  if (!record_overlay) {
    plan.ok = true;
    return plan;
  }
  const std::filesystem::path filename(overlay_video_name);
  if (overlay_video_name.empty() || filename.is_absolute() || filename.has_parent_path()) {
    plan.error = "Overlay video name must be a relative filename";
    return plan;
  }
  if (ToLower(filename.extension().string()) != ".mkv") {
    plan.error = "Overlay recording must use an .mkv filename";
    return plan;
  }
  plan.output_path = result_directory / filename;
  if (std::filesystem::absolute(plan.output_path).lexically_normal() ==
          std::filesystem::absolute(input.video_path).lexically_normal()) {
    plan.error = "Overlay artifacts must be written outside the source dataset";
    plan.output_path.clear();
    return plan;
  }
  plan.ok = true;
  return plan;
}

OfflineEvalOverlayMode OfflineEvalOverlayModeFor(const bool preview_overlay, const bool record_overlay) {
  if (preview_overlay && record_overlay) {
    return OfflineEvalOverlayMode::kPreviewAndRecord;
  }
  if (preview_overlay) {
    return OfflineEvalOverlayMode::kPreviewOnly;
  }
  if (record_overlay) {
    return OfflineEvalOverlayMode::kRecordOnly;
  }
  return OfflineEvalOverlayMode::kHeadless;
}

std::string OfflineEvalSourceKindToString(const OfflineEvalSourceKind source_kind) {
  switch (source_kind) {
    case OfflineEvalSourceKind::kFfv1Capture:
      return "ffv1_capture";
    case OfflineEvalSourceKind::kHistoricalH264:
      return "historical_h264";
    case OfflineEvalSourceKind::kExplicitVideo:
      return "explicit_video";
  }
  return "unknown";
}

std::string OfflineEvalOverlayModeToString(const OfflineEvalOverlayMode mode) {
  switch (mode) {
    case OfflineEvalOverlayMode::kHeadless:
      return "headless";
    case OfflineEvalOverlayMode::kPreviewOnly:
      return "preview_only";
    case OfflineEvalOverlayMode::kRecordOnly:
      return "record_only";
    case OfflineEvalOverlayMode::kPreviewAndRecord:
      return "preview_and_record";
  }
  return "unknown";
}

}  // namespace vision_demo_host::tools
