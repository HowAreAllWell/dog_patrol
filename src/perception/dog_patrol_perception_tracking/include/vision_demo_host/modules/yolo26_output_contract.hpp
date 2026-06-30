#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace vision_demo_host {

struct Yolo26OutputContract {
  std::size_t batch{0};
  std::size_t detection_count{0};
  std::size_t fields_per_detection{0};
};

// YOLO26 one-to-one end-to-end final detections.
// Expected TensorRT output: [1, 300, 6] = x1,y1,x2,y2,conf,cls.
// This path is NMS-free; do not add traditional detector NMS here.
bool ValidateYolo26OutputShape(const std::vector<int64_t> &dims, Yolo26OutputContract *contract,
                               std::string *error);

std::string FormatYolo26OutputShape(const std::vector<int64_t> &dims);

}  // namespace vision_demo_host
