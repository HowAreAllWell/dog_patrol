#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "dog_patrol_perception_tracking/modules/mission_coordinator.hpp"
#include "dog_patrol_perception_tracking/modules/mission_state_sequence.hpp"

namespace dog_patrol_perception_tracking {

// A capability is either operational, still initializing, or has reported a
// failure that requires its owning runtime to recover before perception is ready.
enum class PerceptionReadiness : std::uint8_t {
  kNotReady = 0,
  kReady = 1,
  kFailure = 2,
};

struct PerceptionReadinessContribution {
  std::string capability;
  PerceptionReadiness readiness{PerceptionReadiness::kNotReady};
  std::string detail;
};

// A replaceable provider for one required perception capability. The
// aggregator only consumes this contract; transport and mission-supervisor
// policy stay outside the contributor implementation.
class PerceptionReadinessContributor {
 public:
  virtual ~PerceptionReadinessContributor() = default;

  virtual PerceptionReadinessContribution Contribution() const = 0;
};

// The vision-owned, non-placeholder contribution. The live adapter reports
// detector/tracker runtime state through this narrow seam; #84 owns that ROS
// adapter and therefore need not change aggregation policy when it wires it.
class DetectionTrackingReadinessContributor final : public PerceptionReadinessContributor {
 public:
  static constexpr const char *kCapability = "detection_tracking";

  struct RuntimeStatus {
    bool detector_initialized{false};
    bool tracker_initialized{false};
    std::string failure_detail;
  };

  // Called from the vision node's existing detector/tracker initialization and
  // frame-processing path. It does not publish ROS messages or consume policy.
  void ReportRuntimeStatus(RuntimeStatus status);
  PerceptionReadinessContribution Contribution() const override;

 private:
  RuntimeStatus status_;
};

// An explicit temporary contract for a capability owned outside this package.
// `owner` and `replacement_seam` are recorded so an integration can replace it
// without changing aggregation, vision-node, or mission-supervisor policy.
class PlaceholderReadinessContributor final : public PerceptionReadinessContributor {
 public:
  PlaceholderReadinessContributor(std::string capability, std::string owner,
                                  std::string replacement_seam,
                                  PerceptionReadiness readiness = PerceptionReadiness::kNotReady,
                                  std::string detail = {});

  const std::string &owner() const { return owner_; }
  const std::string &replacement_seam() const { return replacement_seam_; }
  PerceptionReadinessContribution Contribution() const override;

 private:
  std::string capability_;
  std::string owner_;
  std::string replacement_seam_;
  PerceptionReadiness readiness_;
  std::string detail_;
};

// A lightweight contributor for an owning module's adapter. It has the same
// public contract as the placeholder it replaces, so callers do not need a
// policy branch while an external capability is integrated.
class MutableReadinessContributor final : public PerceptionReadinessContributor {
 public:
  MutableReadinessContributor(std::string capability, PerceptionReadiness readiness,
                              std::string detail = {});

  void Report(PerceptionReadiness readiness, std::string detail = {});
  PerceptionReadinessContribution Contribution() const override;

 private:
  std::string capability_;
  PerceptionReadiness readiness_;
  std::string detail_;
};

struct PerceptionReadyAction {
  std::uint32_t observed_state_seq{0};
};

class PerceptionReadinessAggregator {
 public:
  struct Output {
    std::optional<PerceptionReadyAction> ready;
  };

  // Every required capability must have a unique, non-empty name.
  void AddRequiredContributor(std::unique_ptr<PerceptionReadinessContributor> contributor);

  // Replaces one required capability in place. The replacement must retain the
  // same capability name so outside mission and node policy is unchanged.
  bool ReplaceRequiredContributor(std::string capability,
                                  std::unique_ptr<PerceptionReadinessContributor> contributor);

  // Consumes the same authoritative mission snapshot / state-sequence terms as
  // MissionCoordinator. READY is only eligible in the current STARTUP state.
  Output Update(const MissionSnapshot &mission);

  std::vector<PerceptionReadinessContribution> RequiredContributions() const;

 private:
  bool AllRequiredContributorsReady() const;

  std::vector<std::unique_ptr<PerceptionReadinessContributor>> required_contributors_;
  MissionStateSequenceCursor state_sequence_;
  std::optional<std::uint32_t> emitted_startup_state_seq_;
};

}  // namespace dog_patrol_perception_tracking
