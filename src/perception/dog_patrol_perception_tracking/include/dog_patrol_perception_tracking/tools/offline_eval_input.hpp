#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>

namespace dog_patrol_perception_tracking::tools {

// An FFV1 capture take is the canonical replay source. Historical MP4 files remain
// deliberately readable only to support migration regressions.
enum class OfflineEvalSourceKind {
  kFfv1Capture,
  kHistoricalH264,
  kExplicitVideo,
};

enum class OfflineEvalOverlayMode {
  kHeadless,
  kPreviewOnly,
  kRecordOnly,
  kPreviewAndRecord,
};

struct OfflineEvalCaptureMetadata {
  std::string state;
  std::string codec;
  std::string container;
  std::size_t written_frames{0U};
};

struct OfflineEvalTimestampValidation {
  bool ok{false};
  std::size_t rows{0U};
  std::string error;
};

struct OfflineEvalInputRequest {
  std::filesystem::path dataset_directory;
  std::filesystem::path explicit_video_path;
};

struct OfflineEvalInput {
  std::filesystem::path dataset_directory;
  std::filesystem::path video_path;
  OfflineEvalSourceKind source_kind{OfflineEvalSourceKind::kExplicitVideo};
  std::optional<OfflineEvalCaptureMetadata> capture;
  OfflineEvalTimestampValidation timestamp_validation;
};

struct OfflineEvalInputDiscovery {
  bool ok{false};
  OfflineEvalInput input;
  std::string error;
};

struct OfflineEvalReplayValidation {
  bool ok{false};
  std::string error;
};

struct OfflineEvalOverlayArtifactPlan {
  bool ok{false};
  std::filesystem::path output_path;
  std::string error;
};

OfflineEvalInputDiscovery DiscoverOfflineEvalInput(const OfflineEvalInputRequest &request);
OfflineEvalReplayValidation ValidateOfflineEvalReplay(const OfflineEvalInput &input,
                                                       std::size_t decoded_frames);
OfflineEvalOverlayArtifactPlan PlanOfflineEvalOverlayArtifacts(
    const OfflineEvalInput &input, const std::filesystem::path &result_directory, bool record_overlay,
    const std::string &overlay_video_name);

OfflineEvalOverlayMode OfflineEvalOverlayModeFor(bool preview_overlay, bool record_overlay);
std::string OfflineEvalSourceKindToString(OfflineEvalSourceKind source_kind);
std::string OfflineEvalOverlayModeToString(OfflineEvalOverlayMode mode);

}  // namespace dog_patrol_perception_tracking::tools
