#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "vision_demo_host/modules/mission_coordinator.hpp"
#include "vision_demo_host/modules/primary_target_manager.hpp"

namespace vision_demo_host {

// Metadata carried alongside each processed camera frame. The patrol
// interface has no camera-frame-number field, so the number remains runtime
// diagnostic metadata while timestamp, optical frame, and source dimensions
// are represented by TargetBoundingBox's Header and fields.
struct SourceFrameMetadata {
  std::uint64_t source_timestamp_ns{0};
  std::uint32_t camera_frame_number{0};
  bool camera_frame_number_available{false};
  int image_width{0};
  int image_height{0};
  std::string optical_frame_id;
};

class MissionFrameTransaction {
 public:
  struct Config {
    PrimaryTargetManager::Config primary;
    MissionCoordinator::Config coordinator;
  };

  struct FrameInput {
    std::optional<MissionSnapshot> mission;
    std::optional<MissionSnapshot> previous_mission;
    const std::vector<IdentityObservation> &identities;
    MissionCoordinator::TimePoint source_time{};
    SourceFrameMetadata metadata;
  };

  struct Output {
    PrimaryTargetResult primary;
    std::optional<FreshTargetBoxAction> target_box;
    std::vector<MissionEventAction> events;
    std::string primary_decision_reason;
    std::string primary_reject_reason;
  };

  MissionFrameTransaction();
  explicit MissionFrameTransaction(Config config);
  MissionFrameTransaction(PrimaryTargetManager::Config primary_config,
                          MissionCoordinator::Config coordinator_config);

  Output Update(const FrameInput &input);
  PrimaryTargetResult CurrentPrimary() const;

 private:
  static bool IsTrustedPrimary(const PrimaryTargetResult &primary, int semantic_id);
  static bool CanRepresentTargetBox(const FreshTargetBoxAction &action,
                                    const SourceFrameMetadata &metadata);

  PrimaryTargetManager primary_manager_;
  MissionCoordinator coordinator_;
  std::optional<std::uint32_t> confirmed_patrol_state_seq_;
};

}  // namespace vision_demo_host
