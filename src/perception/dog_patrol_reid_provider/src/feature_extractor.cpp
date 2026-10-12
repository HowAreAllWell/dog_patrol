// -*- coding: utf-8 -*-
#include "dog_patrol_reid_provider/feature_extractor.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>
#include <utility>

#if defined(REID_HAS_TENSORRT)
#include "dog_patrol_reid_provider/feature_extractor_trt.hpp"
#endif

namespace dog_patrol_reid_provider {
namespace {

// OpenCV DNN ONNX 后端（原实现，接口化后作为默认后端保留）。
class OpenCvDnnFeatureExtractor final : public FeatureExtractor {
 public:
  explicit OpenCvDnnFeatureExtractor(const Config& cfg) : cfg_(cfg) {}

  bool load() override {
    if (loaded_) {
      return true;
    }
    if (cfg_.model_path.empty()) {
      std::fprintf(stderr, "[reid_extractor] model_path 未配置，不加载模型\n");
      return false;
    }
    try {
      net_ = cv::dnn::readNetFromONNX(cfg_.model_path);
    } catch (const cv::Exception& e) {
      std::fprintf(stderr, "[reid_extractor] 读取 ONNX 失败(%s): %s\n",
                   cfg_.model_path.c_str(), e.what());
      return false;
    }
    if (cfg_.device == "cuda") {
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    } else {
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    // 预热一次：摊平首次推理的初始化开销，同时尽早暴露后端不可用等问题
    cv::Mat dummy = cv::Mat::zeros(cfg_.input_height, cfg_.input_width, CV_8UC3);
    const std::vector<float> f = extract(dummy);
    if (f.empty()) {
      std::fprintf(stderr,
                   "[reid_extractor] 预热推理失败，device=%s（可尝试 device:=cpu）\n",
                   cfg_.device.c_str());
      net_ = cv::dnn::Net();
      return false;
    }
    loaded_ = true;
    std::fprintf(stderr,
                 "[reid_extractor] 模型就绪：%s（device=%s, 输入 %dx%d, dim=%zu）\n",
                 cfg_.model_path.c_str(), cfg_.device.c_str(), cfg_.input_width,
                 cfg_.input_height, f.size());
    return true;
  }

  void unload() override {
    if (!loaded_ && net_.empty()) {
      return;
    }
    net_ = cv::dnn::Net();  // 释放网络与其显存/内存占用
    loaded_ = false;
    std::fprintf(stderr, "[reid_extractor] 模型已卸载（显存让位）\n");
  }

  bool loaded() const override { return loaded_; }

  std::vector<float> extract(const cv::Mat& bgr_crop) override {
    if (net_.empty() || bgr_crop.empty()) {
      return {};
    }
    // scale=1.0、无 mean、swapRB=true：blob 为 NCHW float32、RGB、值域 0~255，
    // /255 与 mean/std 在 ONNX 图内完成
    const cv::Mat blob = cv::dnn::blobFromImage(
        bgr_crop, 1.0, cv::Size(cfg_.input_width, cfg_.input_height),
        cv::Scalar(), /*swapRB=*/true, /*crop=*/false, CV_32F);
    net_.setInput(blob);
    cv::Mat out;
    net_.forward(out);  // [1, D]
    if (out.empty() || out.rows < 1) {
      return {};
    }
    const float* row = out.ptr<float>(0);
    std::vector<float> feat(row, row + out.cols);
    // 图内已 L2 归一化，这里防御性再归一化一次（幂等）
    double s = 0.0;
    for (float v : feat) s += static_cast<double>(v) * v;
    if (s <= 1e-12) {
      return {};
    }
    const float inv = static_cast<float>(1.0 / std::sqrt(s));
    for (float& v : feat) v *= inv;
    return feat;
  }

 private:
  Config cfg_;
  cv::dnn::Net net_;
  bool loaded_ = false;
};

}  // namespace

std::unique_ptr<FeatureExtractor> FeatureExtractor::Create(const Config& cfg) {
  if (cfg.device == "tensorrt") {
#if defined(REID_HAS_TENSORRT)
    return CreateTrtFeatureExtractor(cfg);
#else
    std::fprintf(stderr,
                 "[reid_extractor] device=tensorrt 但本构建未启用 TensorRT 后端"
                 "（CMake 选项 REID_BUILD_TENSORRT 且需找到 libnvinfer）\n");
    return nullptr;
#endif
  }
  return std::make_unique<OpenCvDnnFeatureExtractor>(cfg);
}

}  // namespace dog_patrol_reid_provider
