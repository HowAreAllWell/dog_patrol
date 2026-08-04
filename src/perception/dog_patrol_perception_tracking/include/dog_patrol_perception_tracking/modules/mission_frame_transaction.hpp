#pragma once

#include <optional>
#include <string>
#include <vector>

#include "dog_patrol_perception_tracking/modules/mission_coordinator.hpp"
#include "dog_patrol_perception_tracking/modules/primary_target_manager.hpp"
#include "dog_patrol_perception_tracking/source_frame_metadata.hpp"

namespace dog_patrol_perception_tracking {

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

}  // namespace dog_patrol_perception_tracking
