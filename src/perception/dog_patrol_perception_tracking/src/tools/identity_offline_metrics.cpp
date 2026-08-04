#include "dog_patrol_perception_tracking/tools/identity_offline_metrics.hpp"

#include "identity_observation_projection.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace dog_patrol_perception_tracking::tools {
namespace {

constexpr const char *kPerFrameCsv = "per_frame.csv";
constexpr const char *kIdentitiesCsv = "identities.csv";
constexpr const char *kSidScoresCsv = "sid_scores.csv";
constexpr const char *kPhase3ShadowCsv = "phase3_shadow_state.csv";
constexpr const char *kTrackletHypothesesCsv = "tracklet_hypotheses.csv";

std::string Trim(const std::string &s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

std::vector<std::string> SplitCsvLine(const std::string &line) {
  std::vector<std::string> out;
  std::string current;
  bool in_quotes = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (in_quotes && i + 1 < line.size() && line[i + 1] == '"') {
        current.push_back('"');
        ++i;
      } else {
        in_quotes = !in_quotes;
      }
    } else if (c == ',' && !in_quotes) {
      out.push_back(Trim(current));
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  out.push_back(Trim(current));
  return out;
}

std::map<std::string, std::size_t> HeaderIndex(const std::vector<std::string> &header) {
  std::map<std::string, std::size_t> out;
  for (std::size_t i = 0; i < header.size(); ++i) {
    out[header[i]] = i;
  }
  return out;
}

std::string Field(const std::vector<std::string> &row, const std::map<std::string, std::size_t> &index,
                  const std::string &name) {
  const auto it = index.find(name);
  if (it == index.end() || it->second >= row.size()) {
    return "";
  }
  return row[it->second];
}

void CountIfPresent(const std::string &value, std::map<std::string, std::size_t> *counts) {
  const std::string key = Trim(value);
  if (!key.empty()) {
    (*counts)[key]++;
  }
}

bool StartsWith(const std::string &value, const std::string &prefix) {
  return value.rfind(prefix, 0) == 0;
}

bool IsBirthStage(const std::string &stage) {
  return stage == "birth_candidate" || stage == "new_semantic" || stage == "phase5_birth_candidate" ||
         stage == "phase5_new_semantic";
}

IdentityState IdentityStateFromString(const std::string &state) {
  if (state == "ACTIVE" || state == "VISIBLE") {
    return IdentityState::kActive;
  }
  if (state == "OCCLUDED") {
    return IdentityState::kOccluded;
  }
  if (state == "INACTIVE") {
    return IdentityState::kInactive;
  }
  if (state == "LOST") {
    return IdentityState::kLost;
  }
  if (state == "MERGED") {
    return IdentityState::kMerged;
  }
  if (state == "SPLIT_RECOVERY") {
    return IdentityState::kSplitRecovery;
  }
  return IdentityState::kUnknown;
}

IdentityManager::Mode IdentityModeFromString(const std::string &mode) {
  if (mode == "MERGED") {
    return IdentityManager::Mode::kMerged;
  }
  if (mode == "SPLIT_RECOVERY") {
    return IdentityManager::Mode::kSplitRecovery;
  }
  if (mode == "NORMAL_RESUMED") {
    return IdentityManager::Mode::kNormalResumed;
  }
  return IdentityManager::Mode::kNormal;
}

int ParseIntOrDefault(const std::string &value, const int fallback) {
  try {
    return std::stoi(value);
  } catch (...) {
    return fallback;
  }
}

bool ParseBoolCsv(const std::string &value) {
  return value == "1" || value == "true" || value == "TRUE";
}

void MarkUnavailable(IdentityOfflineMetrics *metrics, const std::string &filename, const std::string &note) {
  auto &input = metrics->inputs[filename];
  input.available = false;
  input.rows = 0;
  input.note = note;
}

template <typename RowFn>
void ReadCsv(const std::filesystem::path &dir, const std::string &filename, IdentityOfflineMetrics *metrics,
             RowFn row_fn) {
  const std::filesystem::path path = dir / filename;
  if (!std::filesystem::exists(path)) {
    MarkUnavailable(metrics, filename, "unavailable: file not written");
    return;
  }
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    MarkUnavailable(metrics, filename, "unavailable: failed to open");
    return;
  }

  std::string line;
  if (!std::getline(ifs, line)) {
    MarkUnavailable(metrics, filename, "unavailable: empty file");
    return;
  }
  const auto index = HeaderIndex(SplitCsvLine(line));
  auto &input = metrics->inputs[filename];
  input.available = true;
  input.rows = 0;
  input.note = "available";

  while (std::getline(ifs, line)) {
    if (Trim(line).empty()) {
      continue;
    }
    const auto row = SplitCsvLine(line);
    row_fn(row, index);
    input.rows++;
  }
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

void WriteCountMapJson(std::ostream &os, const std::string &name, const std::map<std::string, std::size_t> &counts,
                       const bool trailing_comma) {
  os << "  \"" << name << "\": {";
  if (!counts.empty()) {
    os << "\n";
    std::size_t i = 0;
    for (const auto &[key, count] : counts) {
      os << "    \"" << JsonEscape(key) << "\": " << count;
      if (++i < counts.size()) {
        os << ",";
      }
      os << "\n";
    }
    os << "  }";
  } else {
    os << "}";
  }
  if (trailing_comma) {
    os << ",";
  }
  os << "\n";
}

void WriteCountMapMd(std::ostream &os, const std::string &title, const std::map<std::string, std::size_t> &counts) {
  os << "## " << title << "\n\n";
  if (counts.empty()) {
    os << "- unavailable_or_zero: 0\n\n";
    return;
  }
  for (const auto &[key, count] : counts) {
    os << "- " << key << ": " << count << "\n";
  }
  os << "\n";
}

std::map<int, IdentityManager::Mode> ReadFrameModes(const std::filesystem::path &dir) {
  std::map<int, IdentityManager::Mode> modes;
  std::ifstream ifs(dir / kPerFrameCsv);
  if (!ifs.is_open()) {
    return modes;
  }
  std::string line;
  if (!std::getline(ifs, line)) {
    return modes;
  }
  const auto index = HeaderIndex(SplitCsvLine(line));
  while (std::getline(ifs, line)) {
    if (Trim(line).empty()) {
      continue;
    }
    const auto row = SplitCsvLine(line);
    const int frame_idx = ParseIntOrDefault(Field(row, index, "frame_idx"), -1);
    if (frame_idx >= 0) {
      modes[frame_idx] = IdentityModeFromString(Field(row, index, "sid_mode"));
    }
  }
  return modes;
}

std::map<int, std::vector<IdentityManager::Phase3ShadowDebugRow>> ReadPhase3RowsByFrame(
    const std::filesystem::path &dir) {
  std::map<int, std::vector<IdentityManager::Phase3ShadowDebugRow>> rows_by_frame;
  std::ifstream ifs(dir / kPhase3ShadowCsv);
  if (!ifs.is_open()) {
    return rows_by_frame;
  }
  std::string line;
  if (!std::getline(ifs, line)) {
    return rows_by_frame;
  }
  const auto index = HeaderIndex(SplitCsvLine(line));
  while (std::getline(ifs, line)) {
    if (Trim(line).empty()) {
      continue;
    }
    const auto row = SplitCsvLine(line);
    const int frame_idx = ParseIntOrDefault(Field(row, index, "frame_idx"), -1);
    if (frame_idx < 0) {
      continue;
    }
    IdentityManager::Phase3ShadowDebugRow debug_row;
    debug_row.event_type = Field(row, index, "event_type");
    debug_row.semantic_ids = Field(row, index, "semantic_ids");
    debug_row.carrier_semantic_id = ParseIntOrDefault(Field(row, index, "carrier_semantic_id"), -1);
    debug_row.carrier_raw_track_id = ParseIntOrDefault(Field(row, index, "carrier_raw_track_id"), -1);
    debug_row.candidate_raw_track_id = ParseIntOrDefault(Field(row, index, "candidate_raw_track_id"), -1);
    debug_row.candidate_semantic_id = ParseIntOrDefault(Field(row, index, "candidate_semantic_id"), -1);
    debug_row.reason = Field(row, index, "reason");
    debug_row.related_raw_track_id = ParseIntOrDefault(Field(row, index, "related_raw_track_id"), -1);
    debug_row.hypothesis_status = Field(row, index, "hypothesis_status");
    rows_by_frame[frame_idx].push_back(std::move(debug_row));
  }
  return rows_by_frame;
}

}  // namespace

IdentityOfflineMetrics BuildIdentityOfflineMetrics(const std::filesystem::path &dataset_result_dir,
                                                   const std::string &dataset_name) {
  IdentityOfflineMetrics metrics;
  metrics.dataset_name = dataset_name;
  const auto frame_modes = ReadFrameModes(dataset_result_dir);
  const auto phase3_rows_by_frame = ReadPhase3RowsByFrame(dataset_result_dir);
  for (const std::string filename :
       {kPerFrameCsv, kIdentitiesCsv, kSidScoresCsv, kPhase3ShadowCsv, kTrackletHypothesesCsv}) {
    MarkUnavailable(&metrics, filename, "unavailable: file not written");
  }

  ReadCsv(dataset_result_dir, kPerFrameCsv, &metrics,
          [&](const std::vector<std::string> &row, const std::map<std::string, std::size_t> &index) {
            CountIfPresent(Field(row, index, "track_state"), &metrics.primary_state_counts);
            CountIfPresent(Field(row, index, "primary_decision_reason"), &metrics.primary_decision_reason_counts);
            CountIfPresent(Field(row, index, "primary_reject_reason"), &metrics.primary_reject_reason_counts);
            CountIfPresent(Field(row, index, "primary_recovery_reason"), &metrics.primary_recovery_reason_counts);
          });

  ReadCsv(dataset_result_dir, kIdentitiesCsv, &metrics,
          [&](const std::vector<std::string> &row, const std::map<std::string, std::size_t> &index) {
            CountIfPresent(Field(row, index, "identity_state"), &metrics.identity_state_counts);
            CountIfPresent(Field(row, index, "assignment_stage"), &metrics.assignment_stage_counts);
            CountIfPresent(Field(row, index, "assignment_reject_reason"), &metrics.assignment_reject_reason_counts);
            IdentityObservation identity;
            const int frame_idx = ParseIntOrDefault(Field(row, index, "frame_idx"), -1);
            identity.semantic_id = ParseIntOrDefault(Field(row, index, "semantic_id"), -1);
            identity.state = IdentityStateFromString(Field(row, index, "identity_state"));
            identity.visible = ParseBoolCsv(Field(row, index, "visible"));
            const int raw_track_id = ParseIntOrDefault(Field(row, index, "supporting_raw_track_id"), -1);
            if (raw_track_id > 0) {
              identity.supporting_raw_track_id = raw_track_id;
            }
            identity.missing_frames = ParseIntOrDefault(Field(row, index, "missing_frames"), 0);
            const auto mode_it = frame_modes.find(frame_idx);
            const auto rows_it = phase3_rows_by_frame.find(frame_idx);
            const auto lifecycle = IdentityObservationProjection::ProjectTargetLifecycle(
                identity,
                mode_it == frame_modes.end() ? IdentityManager::Mode::kNormal : mode_it->second,
                rows_it == phase3_rows_by_frame.end()
                    ? std::vector<IdentityManager::Phase3ShadowDebugRow>{}
                    : rows_it->second);
            CountIfPresent(TargetLifecycleToString(lifecycle), &metrics.target_lifecycle_counts);
          });

  ReadCsv(dataset_result_dir, kSidScoresCsv, &metrics,
          [&](const std::vector<std::string> &row, const std::map<std::string, std::size_t> &index) {
            const std::string stage = Field(row, index, "stage");
            const std::string reject_reason = Field(row, index, "reject_reason");
            CountIfPresent(stage, &metrics.assignment_stage_counts);
            CountIfPresent(reject_reason, &metrics.assignment_reject_reason_counts);
            CountIfPresent(Field(row, index, "feature_update_reason"), &metrics.feature_update_reason_counts);
            CountIfPresent(Field(row, index, "geometry_update_reason"), &metrics.geometry_update_reason_counts);
            if (IsBirthStage(stage)) {
              CountIfPresent(reject_reason, &metrics.birth_reason_counts);
            }
          });

  ReadCsv(dataset_result_dir, kPhase3ShadowCsv, &metrics,
          [&](const std::vector<std::string> &row, const std::map<std::string, std::size_t> &index) {
            const std::string event_type = Field(row, index, "event_type");
            const std::string reason = Field(row, index, "reason");
            CountIfPresent(event_type, &metrics.phase3_event_type_counts);
            if (StartsWith(event_type, "new_birth_candidate_")) {
              CountIfPresent(reason, &metrics.birth_reason_counts);
            }
            if (StartsWith(event_type, "phase4_")) {
              CountIfPresent(event_type, &metrics.phase4_handoff_event_counts);
            }
          });

  ReadCsv(dataset_result_dir, kTrackletHypothesesCsv, &metrics,
          [&](const std::vector<std::string> &row, const std::map<std::string, std::size_t> &index) {
            CountIfPresent(Field(row, index, "status"), &metrics.tracklet_hypothesis_status_counts);
            CountIfPresent(Field(row, index, "reason"), &metrics.tracklet_hypothesis_reason_counts);
          });

  return metrics;
}

bool WriteIdentityOfflineMetricsFiles(const std::filesystem::path &dataset_result_dir,
                                      const IdentityOfflineMetrics &metrics, std::string *error) {
  std::ofstream json(dataset_result_dir / "identity_metrics.json");
  if (!json.is_open()) {
    if (error != nullptr) {
      *error = "failed to open identity_metrics.json";
    }
    return false;
  }
  json << "{\n"
       << "  \"dataset_name\": \"" << JsonEscape(metrics.dataset_name) << "\",\n"
       << "  \"inputs\": {\n";
  std::size_t input_i = 0;
  for (const auto &[filename, input] : metrics.inputs) {
    json << "    \"" << JsonEscape(filename) << "\": {"
         << "\"available\": " << (input.available ? "true" : "false") << ", "
         << "\"rows\": " << input.rows << ", "
         << "\"note\": \"" << JsonEscape(input.note) << "\"}";
    if (++input_i < metrics.inputs.size()) {
      json << ",";
    }
    json << "\n";
  }
  json << "  },\n";
  WriteCountMapJson(json, "primary_state_counts", metrics.primary_state_counts, true);
  WriteCountMapJson(json, "primary_decision_reason_counts", metrics.primary_decision_reason_counts, true);
  WriteCountMapJson(json, "primary_reject_reason_counts", metrics.primary_reject_reason_counts, true);
  WriteCountMapJson(json, "primary_recovery_reason_counts", metrics.primary_recovery_reason_counts, true);
  WriteCountMapJson(json, "identity_state_counts", metrics.identity_state_counts, true);
  WriteCountMapJson(json, "target_lifecycle_counts", metrics.target_lifecycle_counts, true);
  WriteCountMapJson(json, "assignment_stage_counts", metrics.assignment_stage_counts, true);
  WriteCountMapJson(json, "assignment_reject_reason_counts", metrics.assignment_reject_reason_counts, true);
  WriteCountMapJson(json, "feature_update_reason_counts", metrics.feature_update_reason_counts, true);
  WriteCountMapJson(json, "geometry_update_reason_counts", metrics.geometry_update_reason_counts, true);
  WriteCountMapJson(json, "phase3_event_type_counts", metrics.phase3_event_type_counts, true);
  WriteCountMapJson(json, "birth_reason_counts", metrics.birth_reason_counts, true);
  WriteCountMapJson(json, "phase4_handoff_event_counts", metrics.phase4_handoff_event_counts, true);
  WriteCountMapJson(json, "tracklet_hypothesis_status_counts", metrics.tracklet_hypothesis_status_counts, true);
  WriteCountMapJson(json, "tracklet_hypothesis_reason_counts", metrics.tracklet_hypothesis_reason_counts, false);
  json << "}\n";

  std::ofstream md(dataset_result_dir / "identity_metrics.md");
  if (!md.is_open()) {
    if (error != nullptr) {
      *error = "failed to open identity_metrics.md";
    }
    return false;
  }
  md << "# Identity Metrics\n\n"
     << "- dataset_name: " << metrics.dataset_name << "\n\n"
     << "## Inputs\n\n";
  for (const auto &[filename, input] : metrics.inputs) {
    md << "- " << filename << ": " << (input.available ? "available" : "unavailable") << ", rows=" << input.rows
       << ", note=" << input.note << "\n";
  }
  md << "\n";
  WriteCountMapMd(md, "Primary States", metrics.primary_state_counts);
  WriteCountMapMd(md, "Primary Decision Reasons", metrics.primary_decision_reason_counts);
  WriteCountMapMd(md, "Primary Reject Reasons", metrics.primary_reject_reason_counts);
  WriteCountMapMd(md, "Primary Recovery Reasons", metrics.primary_recovery_reason_counts);
  WriteCountMapMd(md, "Identity States", metrics.identity_state_counts);
  WriteCountMapMd(md, "Target Lifecycle", metrics.target_lifecycle_counts);
  WriteCountMapMd(md, "Assignment Stages", metrics.assignment_stage_counts);
  WriteCountMapMd(md, "Assignment Reject Reasons", metrics.assignment_reject_reason_counts);
  WriteCountMapMd(md, "Feature Update Reasons", metrics.feature_update_reason_counts);
  WriteCountMapMd(md, "Geometry Update Reasons", metrics.geometry_update_reason_counts);
  WriteCountMapMd(md, "Phase 3 Shadow Event Types", metrics.phase3_event_type_counts);
  WriteCountMapMd(md, "Birth Hidden Pending Allocated Reasons", metrics.birth_reason_counts);
  WriteCountMapMd(md, "Phase 4 Handoff Events", metrics.phase4_handoff_event_counts);
  WriteCountMapMd(md, "Tracklet Hypothesis Status", metrics.tracklet_hypothesis_status_counts);
  WriteCountMapMd(md, "Tracklet Hypothesis Reasons", metrics.tracklet_hypothesis_reason_counts);
  return true;
}

std::string IdentityOfflineMetricsHelp() {
  return
      "Identity offline acceptance metrics:\n"
      "  identity_metrics.json and identity_metrics.md are additive per-dataset summaries written by offline eval.\n"
      "  They aggregate existing debug CSVs without changing tracker, identity, primary, mission ROS, overlay, or CSV schemas.\n"
      "  Inputs are per_frame.csv, identities.csv, sid_scores.csv, phase3_shadow_state.csv, and\n"
      "  tracklet_hypotheses.csv. Missing optional inputs are marked unavailable with zero-count distributions.\n"
      "  Metrics include primary states such as PENDING_RECOVERY, primary decision/reject/recovery reasons,\n"
      "  identity states, target lifecycle counts (VisibleIdentity, OccludedIdentity, MergedGroup,\n"
      "  SplitCandidate, NewBirthCandidate, LostIdentity), assignment stages/reject reasons,\n"
      "  feature_update_reason, geometry_update_reason,\n"
      "  Phase 3 shadow event_type counts, NewBirthCandidate hidden/pending/allocated reasons,\n"
      "  Phase 4 handoff event counts, and tracklet hypothesis status/reason distributions.\n";
}

}  // namespace dog_patrol_perception_tracking::tools
