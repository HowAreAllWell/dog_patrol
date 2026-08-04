#include "dog_patrol_perception_tracking/modules/association_utils.hpp"

#include <algorithm>
#include <cmath>

namespace dog_patrol_perception_tracking {
namespace association {

cv::Point2f BBoxCenter(const cv::Rect2f &bbox) {
  return cv::Point2f(bbox.x + bbox.width * 0.5F, bbox.y + bbox.height * 0.5F);
}

float BBoxIoU(const cv::Rect2f &a, const cv::Rect2f &b) {
  const float intersection = (a & b).area();
  const float union_area = a.area() + b.area() - intersection;
  if (union_area <= 0.0F) {
    return 0.0F;
  }
  return intersection / union_area;
}

float CenterDistanceNormByDiag(const cv::Rect2f &reference, const cv::Rect2f &target) {
  const float ref_diag =
      std::max(1.0F, std::sqrt(reference.width * reference.width + reference.height * reference.height));
  return static_cast<float>(cv::norm(BBoxCenter(target) - BBoxCenter(reference))) / ref_diag;
}

float CenterDistanceNormByMaxArea(const cv::Rect2f &a, const cv::Rect2f &b) {
  const float ref_diag = std::max(1.0F, std::sqrt(std::max(a.area(), b.area())));
  return static_cast<float>(cv::norm(BBoxCenter(a) - BBoxCenter(b))) / ref_diag;
}

float AreaRatio(const cv::Rect2f &current, const cv::Rect2f &reference) {
  return std::max(1.0F, current.area()) / std::max(1.0F, reference.area());
}

}  // namespace association
}  // namespace dog_patrol_perception_tracking
