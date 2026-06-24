#pragma once

#include <opencv2/core.hpp>

namespace vision_demo_host {
namespace association {

cv::Point2f BBoxCenter(const cv::Rect2f &bbox);
float BBoxIoU(const cv::Rect2f &a, const cv::Rect2f &b);
float CenterDistanceNormByDiag(const cv::Rect2f &reference, const cv::Rect2f &target);
float CenterDistanceNormByMaxArea(const cv::Rect2f &a, const cv::Rect2f &b);
float AreaRatio(const cv::Rect2f &current, const cv::Rect2f &reference);

}  // namespace association
}  // namespace vision_demo_host
