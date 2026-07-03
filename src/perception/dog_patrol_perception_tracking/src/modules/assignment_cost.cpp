#include "assignment_cost.hpp"

#include <algorithm>

#include "vision_demo_host/modules/feature_bank_cost.hpp"
#include "vision_demo_host/modules/reliable_geometry_cost.hpp"

namespace vision_demo_host {
namespace {

ReliableGeometryCost::State ReliableGeometryState(const LegacyIdentityRecord &identity) {
  ReliableGeometryCost::State state;
  state.latest_bbox = identity.last_bbox;
  state.latest_center = identity.last_center;
  state.reliable_bbox = identity.feature_geometry.reliable_bbox;
  state.reliable_center = identity.feature_geometry.reliable_center;
  state.reliable_velocity = identity.feature_geometry.reliable_velocity;
  state.missing_frames = identity.missing_frames;
  state.has_reliable_geometry = identity.feature_geometry.has_reliable_geometry;
  return state;
}

}  // namespace

AssignmentCost::Result AssignmentCost::Compute(const Track &track,
                                               const LegacyIdentityRecord &identity,
                                               const std::vector<float> &feature,
                                               const Config &config) {
  Result result;
  result.appearance = FeatureBankCost::AppearanceCost(feature, identity.feature_geometry.feature_bank);
  result.geometry = ReliableGeometryCost::GeometryCost(track.bbox, ReliableGeometryState(identity));
  result.time = std::min(1.0F, static_cast<float>(identity.missing_frames) /
                                   static_cast<float>(std::max(1, config.max_missing_frames)));
  const float weight_sum =
      std::max(1e-6F, config.app_weight + config.geometry_weight + config.time_weight);
  result.final = (config.app_weight * result.appearance + config.geometry_weight * result.geometry +
                  config.time_weight * result.time) /
                 weight_sum;
  return result;
}

}  // namespace vision_demo_host
