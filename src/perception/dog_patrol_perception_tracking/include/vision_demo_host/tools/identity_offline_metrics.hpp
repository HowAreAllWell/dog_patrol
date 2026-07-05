#pragma once

#include <cstddef>
#include <filesystem>
#include <map>
#include <string>

namespace vision_demo_host::tools {

struct IdentityOfflineMetricsInput {
  bool available{false};
  std::size_t rows{0};
  std::string note;
};

struct IdentityOfflineMetrics {
  std::string dataset_name;
  std::map<std::string, IdentityOfflineMetricsInput> inputs;

  std::map<std::string, std::size_t> primary_state_counts;
  std::map<std::string, std::size_t> primary_decision_reason_counts;
  std::map<std::string, std::size_t> primary_reject_reason_counts;
  std::map<std::string, std::size_t> primary_recovery_reason_counts;
  std::map<std::string, std::size_t> identity_state_counts;
  std::map<std::string, std::size_t> assignment_stage_counts;
  std::map<std::string, std::size_t> assignment_reject_reason_counts;
  std::map<std::string, std::size_t> feature_update_reason_counts;
  std::map<std::string, std::size_t> geometry_update_reason_counts;
  std::map<std::string, std::size_t> phase3_event_type_counts;
  std::map<std::string, std::size_t> birth_reason_counts;
  std::map<std::string, std::size_t> phase4_handoff_event_counts;
  std::map<std::string, std::size_t> tracklet_hypothesis_status_counts;
  std::map<std::string, std::size_t> tracklet_hypothesis_reason_counts;
};

IdentityOfflineMetrics BuildIdentityOfflineMetrics(const std::filesystem::path &dataset_result_dir,
                                                   const std::string &dataset_name);

bool WriteIdentityOfflineMetricsFiles(const std::filesystem::path &dataset_result_dir,
                                      const IdentityOfflineMetrics &metrics, std::string *error);

std::string IdentityOfflineMetricsHelp();

}  // namespace vision_demo_host::tools
