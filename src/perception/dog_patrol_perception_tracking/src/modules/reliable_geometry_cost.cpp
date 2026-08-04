#include "dog_patrol_perception_tracking/modules/reliable_geometry_cost.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>

#include "dog_patrol_perception_tracking/modules/association_utils.hpp"

namespace dog_patrol_perception_tracking {

cv::Rect2f ReliableGeometryCost::ReferenceBBox(const State &state) {
  return state.has_reliable_geometry ? state.reliable_bbox : state.latest_bbox;
}

cv::Point2f ReliableGeometryCost::PredictedCenter(const State &state) {
  const cv::Point2f base_center = state.has_reliable_geometry ? state.reliable_center : state.latest_center;
  if (!state.has_reliable_geometry || state.missing_frames <= 0) {
    return base_center;
  }

  const cv::Rect2f ref_bbox = ReferenceBBox(state);
  const float ref_diag =
      std::max(1.0F, std::sqrt(ref_bbox.width * ref_bbox.width + ref_bbox.height * ref_bbox.height));
  const float raw_steps = static_cast<float>(std::min(state.missing_frames, 45));
  cv::Point2f displacement = state.reliable_velocity * raw_steps;
  const float disp_norm = static_cast<float>(cv::norm(displacement));
  const float max_disp = 0.75F * ref_diag;
  if (disp_norm > max_disp && disp_norm > 1e-3F) {
    displacement *= max_disp / disp_norm;
  }
  return base_center + displacement;
}

float ReliableGeometryCost::GeometryCost(const cv::Rect2f &bbox, const State &state) {
  const cv::Rect2f ref_bbox = ReferenceBBox(state);
  const cv::Point2f ref_center = PredictedCenter(state);
  const float iou = association::BBoxIoU(bbox, ref_bbox);
  const float ref_diag =
      std::max(1.0F, std::sqrt(ref_bbox.width * ref_bbox.width + ref_bbox.height * ref_bbox.height));
  const float center_norm = static_cast<float>(cv::norm(association::BBoxCenter(bbox) - ref_center)) / ref_diag;
  const float center_cost = std::min(1.0F, center_norm);
  const float iou_cost = 1.0F - std::clamp(iou, 0.0F, 1.0F);
  return 0.7F * iou_cost + 0.3F * center_cost;
}

bool ReliableGeometryCost::PassesMissingIdentityGate(const cv::Rect2f &bbox, const State &state,
                                                     const float app_cost, const float geo_cost,
                                                     const MissingGateConfig &config) {
  if (state.missing_frames <= 0) {
    return true;
  }

  const cv::Rect2f ref_bbox = ReferenceBBox(state);
  const cv::Point2f ref_center = PredictedCenter(state);
  const float strict_min_area_ratio = std::max(0.01F, config.min_area_ratio);
  const float max_area_ratio = std::max(strict_min_area_ratio, config.max_area_ratio);
  const float relaxed_min_area_ratio = std::min(strict_min_area_ratio, 0.20F);
  const float strong_app_threshold = std::max(0.0F, std::clamp(config.max_app_cost, 0.0F, 1.0F) - 0.05F);
  const float relaxed_area_max_geo_cost = std::min(1.0F, std::clamp(config.active_max_cost, 0.0F, 1.0F) + 0.25F);
  const bool allow_relaxed_area = app_cost <= strong_app_threshold && geo_cost <= relaxed_area_max_geo_cost;
  const float min_area_ratio = allow_relaxed_area ? relaxed_min_area_ratio : strict_min_area_ratio;

  const float area_ratio = association::AreaRatio(bbox, ref_bbox);
  if (area_ratio < min_area_ratio || area_ratio > max_area_ratio) {
    return false;
  }

  const float ref_diag =
      std::max(1.0F, std::sqrt(ref_bbox.width * ref_bbox.width + ref_bbox.height * ref_bbox.height));
  const float center_norm = static_cast<float>(cv::norm(association::BBoxCenter(bbox) - ref_center)) / ref_diag;
  if (center_norm > std::max(0.1F, config.max_center_dist_norm)) {
    return false;
  }

  return true;
}

bool ReliableGeometryCost::PassesShortMissingAppearanceGate(const State &state, const float app_cost,
                                                            const float geo_cost, const float max_app_cost) {
  if (state.missing_frames <= 0) {
    return true;
  }
  constexpr int kShortMissingGeometryFirstFrames = 45;
  constexpr float kShortMissingMaxGeometryCost = 0.25F;
  constexpr float kShortMissingMaxAppearanceCost = 0.75F;
  if (state.missing_frames <= kShortMissingGeometryFirstFrames && geo_cost <= kShortMissingMaxGeometryCost &&
      app_cost <= kShortMissingMaxAppearanceCost) {
    return true;
  }
  return app_cost <= std::clamp(max_app_cost, 0.0F, 1.0F);
}

}  // namespace dog_patrol_perception_tracking
