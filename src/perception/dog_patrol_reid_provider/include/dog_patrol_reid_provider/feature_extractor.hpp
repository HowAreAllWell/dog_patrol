// -*- coding: utf-8 -*-
// 特征提取器接口 + 工厂。
// device 取值：
//   "tensorrt"  model_path 指向 trtexec 在目标机构建的 .engine（FP16），
//               需构建时启用 TensorRT 后端（CMake 选项 REID_BUILD_TENSORRT）
//   "cuda"/"cpu" model_path 指向 ONNX，走 OpenCV DNN 后端
// 两个后端输入输出约定一致：BGR uint8 任意尺寸裁剪进，L2 归一化特征出；
// 图内已焙入 /255、mean/std、瓶颈层与 L2（通道翻转由预处理完成，模型输入为 RGB）。
#pragma once

#include <memory>

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace dog_patrol_reid_provider {

class FeatureExtractor {
 public:
  struct Config {
    std::string model_path;        // onnx；device=tensorrt 时为 .engine
    std::string device = "cuda";   // "tensorrt" | "cuda" | "cpu"
    int input_height = 256;
    int input_width = 128;
  };

  virtual ~FeatureExtractor() = default;

  // 加载模型/engine（幂等）。成功返回 true；失败打印原因并保持未加载状态。
  virtual bool load() = 0;
  // 卸载释放（显存让位），幂等。
  virtual void unload() = 0;
  virtual bool loaded() const = 0;

  // bgr_crop: 任意尺寸 BGR uint8。返回 L2 归一化特征；未加载/失败返回空。
  virtual std::vector<float> extract(const cv::Mat& bgr_crop) = 0;

  // 按配置创建后端。失败（如 device=tensorrt 但未编 TRT 后端、路径为空）
  // 返回 nullptr，原因已打印。
  static std::unique_ptr<FeatureExtractor> Create(const Config& cfg);
};

}  // namespace dog_patrol_reid_provider
