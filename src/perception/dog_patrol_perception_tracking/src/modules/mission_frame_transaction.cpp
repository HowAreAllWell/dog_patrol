#include "dog_patrol_perception_tracking/modules/mission_frame_transaction.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace dog_patrol_perception_tracking {
namespace {

bool IsFinite(const float value) { return std::isfinite(value); }

constexpr auto kTargetConfirmationRetryInterval = std::chrono::milliseconds{100};

}  // namespace

MissionFrameTransaction::MissionFrameTransaction() : MissionFrameTransaction(Config{}) {}

MissionFrameTransaction::MissionFrameTransaction(Config config)
    : primary_manager_(config.primary), coordinator_(config.coordinator) {}

MissionFrameTransaction::MissionFrameTransaction(
    PrimaryTargetManager::Config primary_config,
    MissionCoordinator::Config coordinator_config)
    : MissionFrameTransaction(Config{std::move(primary_config), std::move(coordinator_config)}) {}

bool MissionFrameTransaction::IsTrustedPrimary(const PrimaryTargetResult &primary,
                                               const int semantic_id) {
  return semantic_id > 0 && primary.primary_target_id == semantic_id &&
         IsTrustedCurrentPrimary(primary);
}

bool MissionFrameTransaction::CanRepresentTargetBox(
    const FreshTargetBoxAction &action, const SourceFrameMetadata &metadata) {
  if (action.target_id <= 0 || metadata.source_timestamp_ns == 0U ||
      metadata.image_width <= 0 || metadata.image_height <= 0 ||
      metadata.optical_frame_id.empty() || action.bbox.width <= 0.0F ||
      action.bbox.height <= 0.0F || !IsFinite(action.bbox.x) ||
      !IsFinite(action.bbox.y) || !IsFinite(action.bbox.width) ||
      !IsFinite(action.bbox.height) || !IsFinite(action.confidence)) {
    return false;
  }

  const float x_end = action.bbox.x + action.bbox.width;
  const float y_end = action.bbox.y + action.bbox.height;
  if (!IsFinite(x_end) || !IsFinite(y_end)) {
    return false;
  }
  const auto clamp_x = [width = metadata.image_width](const double value) {
    return std::clamp(value, 0.0, static_cast<double>(width));
  };
  const auto clamp_y = [height = metadata.image_height](const double value) {
    return std::clamp(value, 0.0, static_cast<double>(height));
  };
  const double x_min = clamp_x(std::floor(static_cast<double>(action.bbox.x)));
  const double y_min = clamp_y(std::floor(static_cast<double>(action.bbox.y)));
  const double x_max = clamp_x(std::ceil(static_cast<double>(x_end)));
  const double y_max = clamp_y(std::ceil(static_cast<double>(y_end)));
  return x_max > x_min && y_max > y_min;
}

MissionFrameTransaction::Output MissionFrameTransaction::Update(const FrameInput &input) {
  Output output;
  output.primary = primary_manager_.UpdateForMission(
      input.identities, input.mission, input.previous_mission, input.source_time);
  output.primary_decision_reason = primary_manager_.LastDecisionReason();
  output.primary_reject_reason = primary_manager_.LastRejectReason();
  if (!input.mission.has_value()) {
    return output;
  }

  const MissionSnapshot &mission = input.mission.value();
  if (mission.phase == MissionPhase::kPatrol && !mission.blocked &&
      mission.block_cause == MissionBlockCause::kNone && mission.target_id == 0 &&
      input.metadata.source_timestamp_ns != 0U &&
      IsTrustedPrimary(output.primary, output.primary.primary_target_id) &&
      (confirmation_attempt_patrol_state_seq_ != mission.state_seq ||
       !last_confirmation_attempt_source_time_.has_value() ||
       (input.source_time >= last_confirmation_attempt_source_time_.value() &&
        input.source_time - last_confirmation_attempt_source_time_.value() >=
            kTargetConfirmationRetryInterval))) {
    confirmation_attempt_patrol_state_seq_ = mission.state_seq;
    last_confirmation_attempt_source_time_ = input.source_time;
    output.events.push_back({PerceptionMissionEvent::kTargetConfirmed,
                             output.primary.primary_target_id, mission.state_seq});
  }

  std::vector<IdentityObservation> projectable_identities;
  projectable_identities.reserve(input.identities.size());
  for (const auto &identity : input.identities) {
    FreshTargetBoxAction candidate;
    candidate.target_id = identity.semantic_id;
    candidate.bbox = identity.bbox;
    candidate.confidence = identity.confidence;
    if (CanRepresentTargetBox(candidate, input.metadata)) {
      projectable_identities.push_back(identity);
    }
  }

  const MissionCoordinator::Output coordinator_output = coordinator_.Update(
      {mission, projectable_identities, output.primary, input.source_time});
  output.target_box = coordinator_output.target_box;
  output.events.insert(output.events.end(), coordinator_output.events.begin(),
                       coordinator_output.events.end());
  return output;
}

PrimaryTargetResult MissionFrameTransaction::CurrentPrimary() const {
  return primary_manager_.GetState();
}

}  // namespace dog_patrol_perception_tracking
