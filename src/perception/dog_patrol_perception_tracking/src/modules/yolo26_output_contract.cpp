#include "dog_patrol_perception_tracking/modules/yolo26_output_contract.hpp"

#include <sstream>

namespace dog_patrol_perception_tracking {
namespace {

constexpr int64_t kExpectedFieldsPerDetection = 6;

bool SetError(std::string *error, const std::string &message) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

}  // namespace

std::string FormatYolo26OutputShape(const std::vector<int64_t> &dims) {
  std::ostringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < dims.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << dims[i];
  }
  oss << "]";
  return oss.str();
}

bool ValidateYolo26OutputShape(const std::vector<int64_t> &dims, Yolo26OutputContract *contract,
                               std::string *error) {
  if (contract != nullptr) {
    *contract = Yolo26OutputContract{};
  }

  if (dims.size() != 3U) {
    return SetError(error, "Unexpected YOLO26 TensorRT output rank: got " + std::to_string(dims.size()) +
                              " dims " + FormatYolo26OutputShape(dims) +
                              ", expected [1, 300, 6] = x1,y1,x2,y2,conf,cls.");
  }

  if (dims[2] != kExpectedFieldsPerDetection) {
    return SetError(error, "Unexpected YOLO26 TensorRT output last dimension: got " + std::to_string(dims[2]) +
                              " in shape " + FormatYolo26OutputShape(dims) +
                              ", expected 6 fields per detection: x1,y1,x2,y2,conf,cls.");
  }

  if (dims[0] != 1) {
    return SetError(error, "Unexpected YOLO26 TensorRT output batch dimension: got " + std::to_string(dims[0]) +
                              " in shape " + FormatYolo26OutputShape(dims) +
                              ", expected batch 1 for [1, 300, 6] final detections.");
  }

  if (dims[0] <= 0 || dims[1] <= 0) {
    return SetError(error, "Unexpected YOLO26 TensorRT output shape " + FormatYolo26OutputShape(dims) +
                              ": batch and detection count must be concrete positive dimensions.");
  }

  if (contract != nullptr) {
    contract->batch = static_cast<std::size_t>(dims[0]);
    contract->detection_count = static_cast<std::size_t>(dims[1]);
    contract->fields_per_detection = static_cast<std::size_t>(dims[2]);
  }

  return true;
}

}  // namespace dog_patrol_perception_tracking
