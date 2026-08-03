#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "vision_demo_host/modules/mission_state_sequence.hpp"
#include "vision_demo_host/types.hpp"

namespace vision_demo_host {

// ROS-independent representation of the dog_patrol MissionState contract.
// The ROS adapter maps the corresponding message constants at its transport
// boundary; this coordinator owns no publisher, subscription, or executor.
enum class MissionPhase : std::uint8_t {
  kStartup = 0,
  kPatrol = 1,
  kConfirmTarget = 2,
  kApproachTarget = 3,
  kVerifyIdentity = 4,
  kTrackIntruder = 5,
};

enum class MissionBlockCause : std::uint8_t {
  kNone = 0,
  kTargetLost = 1,
  kExecutionError = 2,
};

struct MissionSnapshot {
  std::uint32_t state_seq{0};
  MissionPhase phase{MissionPhase::kStartup};
  int target_id{0};
  bool blocked{false};
  MissionBlockCause block_cause{MissionBlockCause::kNone};
};

enum class PerceptionMissionEvent {
  kTargetConfirmed,
  kTargetLost,
  kTargetReacquired,
};

struct MissionEventAction {
  PerceptionMissionEvent event{PerceptionMissionEvent::kTargetLost};
  int target_id{0};
  std::uint32_t observed_state_seq{0};
};

struct FreshTargetBoxAction {
  int target_id{0};
  std::uint32_t observed_state_seq{0};
  std::chrono::steady_clock::time_point source_time{};
  cv::Rect2f bbox;
  float confidence{0.0F};
};

class MissionCoordinator {
 public:
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  using TimePoint = Clock::time_point;

  struct Config {
    Duration lost_event_timeout{std::chrono::milliseconds{500}};
    Duration reacquire_retention{std::chrono::seconds{6}};
  };

  struct FrameInput {
    MissionSnapshot mission;
    // A view of the current processed frame; it must outlive Update().
    const std::vector<IdentityObservation> &identities;
    PrimaryTargetResult primary;
    TimePoint source_time{};
  };

  struct Output {
    std::optional<FreshTargetBoxAction> target_box;
    std::vector<MissionEventAction> events;
  };

  MissionCoordinator();
  explicit MissionCoordinator(Config config);

  // Consumes one processed source frame. A fresh box action is constructed
  // only from this frame's identity observation, never from cached geometry.
  Output Update(const FrameInput &input);

  static bool AcceptsFreshTargetBox(MissionPhase phase);

 private:
  struct LossCycle {
    int target_id{0};
    std::uint32_t loss_event_state_seq{0};
    std::optional<std::uint32_t> reacquire_event_state_seq;
    bool retention_expired{false};
    bool automatic_recovery_disallowed{false};
  };

  bool IsCurrentMissionState(const MissionSnapshot &mission);
  bool IsCurrentSourceTime(TimePoint source_time) const;
  bool IsTrustedCurrentObservation(const FrameInput &input,
                                   const IdentityObservation **observation) const;
  bool IsTargetLifecycleActive(const MissionSnapshot &mission) const;
  bool IsCompatibleLostBlock(const MissionSnapshot &mission, const LossCycle &cycle) const;
  bool CanPublishForSourceTime(TimePoint source_time) const;
  void ResetMissionTarget();

  Config config_;
  MissionStateSequenceCursor state_sequence_;
  std::optional<TimePoint> latest_source_time_;
  std::optional<TimePoint> last_published_source_time_;
  std::optional<int> tracked_target_id_;
  std::optional<TimePoint> last_fresh_observation_at_;
  std::optional<LossCycle> loss_cycle_;
};

}  // namespace vision_demo_host
