#include "dog_patrol_perception_tracking/modules/perception_readiness.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace dog_patrol_perception_tracking {
namespace {

void ValidateCapabilityName(const std::string &capability) {
  if (capability.empty()) {
    throw std::invalid_argument("readiness contributor capability must not be empty");
  }
}

void ValidatePlaceholderMetadata(const std::string &owner, const std::string &replacement_seam) {
  if (owner.empty()) {
    throw std::invalid_argument("placeholder readiness contributor owner must not be empty");
  }
  if (replacement_seam.empty()) {
    throw std::invalid_argument("placeholder readiness contributor replacement seam must not be empty");
  }
}

PerceptionReadinessContribution ValidateContributor(
    const std::unique_ptr<PerceptionReadinessContributor> &contributor) {
  if (contributor == nullptr) {
    throw std::invalid_argument("readiness contributor must not be null");
  }
  PerceptionReadinessContribution contribution = contributor->Contribution();
  ValidateCapabilityName(contribution.capability);
  return contribution;
}

}  // namespace

void DetectionTrackingReadinessContributor::ReportRuntimeStatus(RuntimeStatus status) {
  status_ = std::move(status);
}

PerceptionReadinessContribution DetectionTrackingReadinessContributor::Contribution() const {
  if (!status_.failure_detail.empty()) {
    return {kCapability, PerceptionReadiness::kFailure, status_.failure_detail};
  }
  if (!status_.detector_initialized) {
    return {kCapability, PerceptionReadiness::kNotReady, "detector initialization pending"};
  }
  if (!status_.tracker_initialized) {
    return {kCapability, PerceptionReadiness::kNotReady, "tracker initialization pending"};
  }
  return {kCapability, PerceptionReadiness::kReady, "detector and tracker runtime ready"};
}

PlaceholderReadinessContributor::PlaceholderReadinessContributor(
    std::string capability, std::string owner, std::string replacement_seam,
    const PerceptionReadiness readiness, std::string detail)
    : capability_(std::move(capability)),
      owner_(std::move(owner)),
      replacement_seam_(std::move(replacement_seam)),
      readiness_(readiness),
      detail_(std::move(detail)) {
  ValidateCapabilityName(capability_);
  ValidatePlaceholderMetadata(owner_, replacement_seam_);
}

PerceptionReadinessContribution PlaceholderReadinessContributor::Contribution() const {
  return {capability_, readiness_, detail_};
}

MutableReadinessContributor::MutableReadinessContributor(std::string capability,
                                                         const PerceptionReadiness readiness,
                                                         std::string detail)
    : capability_(std::move(capability)), readiness_(readiness), detail_(std::move(detail)) {
  ValidateCapabilityName(capability_);
}

void MutableReadinessContributor::Report(const PerceptionReadiness readiness, std::string detail) {
  readiness_ = readiness;
  detail_ = std::move(detail);
}

PerceptionReadinessContribution MutableReadinessContributor::Contribution() const {
  return {capability_, readiness_, detail_};
}

void PerceptionReadinessAggregator::AddRequiredContributor(
    std::unique_ptr<PerceptionReadinessContributor> contributor) {
  const PerceptionReadinessContribution contribution = ValidateContributor(contributor);
  const auto existing = std::find_if(required_contributors_.begin(), required_contributors_.end(),
                                     [&contribution](const auto &registered) {
                                       return registered->Contribution().capability == contribution.capability;
                                     });
  if (existing != required_contributors_.end()) {
    throw std::invalid_argument("readiness contributor capability is already required: " +
                                contribution.capability);
  }
  required_contributors_.push_back(std::move(contributor));
}

bool PerceptionReadinessAggregator::ReplaceRequiredContributor(
    std::string capability, std::unique_ptr<PerceptionReadinessContributor> contributor) {
  ValidateCapabilityName(capability);
  const PerceptionReadinessContribution replacement = ValidateContributor(contributor);
  if (replacement.capability != capability) {
    throw std::invalid_argument("readiness replacement must preserve the capability name");
  }

  const auto existing = std::find_if(required_contributors_.begin(), required_contributors_.end(),
                                     [&capability](const auto &registered) {
                                       return registered->Contribution().capability == capability;
                                     });
  if (existing == required_contributors_.end()) {
    return false;
  }
  *existing = std::move(contributor);
  return true;
}

bool PerceptionReadinessAggregator::AllRequiredContributorsReady() const {
  return !required_contributors_.empty() &&
         std::all_of(required_contributors_.begin(), required_contributors_.end(),
                     [](const auto &contributor) {
                       return contributor->Contribution().readiness == PerceptionReadiness::kReady;
                     });
}

PerceptionReadinessAggregator::Output PerceptionReadinessAggregator::Update(const MissionSnapshot &mission) {
  Output output;
  if (!state_sequence_.AcceptsCurrentOrNewer(mission.state_seq) || mission.phase != MissionPhase::kStartup ||
      emitted_startup_state_seq_ == mission.state_seq || !AllRequiredContributorsReady()) {
    return output;
  }

  emitted_startup_state_seq_ = mission.state_seq;
  output.ready = PerceptionReadyAction{mission.state_seq};
  return output;
}

std::vector<PerceptionReadinessContribution> PerceptionReadinessAggregator::RequiredContributions() const {
  std::vector<PerceptionReadinessContribution> contributions;
  contributions.reserve(required_contributors_.size());
  for (const auto &contributor : required_contributors_) {
    contributions.push_back(contributor->Contribution());
  }
  return contributions;
}

}  // namespace dog_patrol_perception_tracking
