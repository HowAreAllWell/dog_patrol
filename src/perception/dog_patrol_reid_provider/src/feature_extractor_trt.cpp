// -*- coding: utf-8 -*-
// TensorRT 特征提取后端。engine 由 trtexec 在目标机上构建（FP16）：
//   scripts/build_trt_engine.sh 生成 weights/reid_clipreid_vitb16_after.fp16.engine
// 输入输出与 OpenCV DNN 后端完全一致（RGB 0~255 NCHW float32 进、L2 特征出），
// TRT engine 的 IO 保持 fp32，精度由 engine 内部的 fp16 kernel 决定。
// API 面向 TensorRT >= 8.5（enqueueV3/setTensorAddress），JetPack 6 的 TRT 10 适用。
#include "dog_patrol_reid_provider/feature_extractor_trt.hpp"

#if defined(REID_HAS_TENSORRT)

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <opencv2/dnn.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <utility>
#include <vector>

namespace dog_patrol_reid_provider {
namespace {

class TrtLogger final : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::fprintf(stderr, "[reid_trt][nvinfer] %s\n", msg);
    }
  }
};

template <typename T>
void TrtDelete(T* p) {
  delete p;
}
template <typename T>
using TrtPtr = std::unique_ptr<T, void (*)(T*)>;

std::vector<char> ReadFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) {
    return {};
  }
  const std::streamsize size = f.tellg();
  f.seekg(0, std::ios::beg);
  std::vector<char> data(static_cast<size_t>(size));
  f.read(data.data(), size);
  if (f.gcount() != size) {
    return {};
  }
  return data;
}

int64_t Volume(const nvinfer1::Dims& dims) {
  int64_t v = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    v *= dims.d[i];
  }
  return v;
}

}  // namespace

class TrtFeatureExtractor final : public FeatureExtractor {
 public:
  explicit TrtFeatureExtractor(const Config& cfg) : cfg_(cfg) {}
  ~TrtFeatureExtractor() override { unload(); }

  TrtFeatureExtractor(const TrtFeatureExtractor&) = delete;
  TrtFeatureExtractor& operator=(const TrtFeatureExtractor&) = delete;

  bool load() override {
    if (loaded_) {
      return true;
    }
    if (cfg_.model_path.empty()) {
      std::fprintf(stderr, "[reid_trt] model_path 未配置（应指向 .engine）\n");
      return false;
    }
    const std::vector<char> engine_data = ReadFile(cfg_.model_path);
    if (engine_data.empty()) {
      std::fprintf(stderr, "[reid_trt] engine 读取失败: %s\n",
                   cfg_.model_path.c_str());
      return false;
    }
    runtime_ = TrtPtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_),
                                          &TrtDelete<nvinfer1::IRuntime>);
    if (!runtime_) {
      std::fprintf(stderr, "[reid_trt] createInferRuntime 失败\n");
      return false;
    }
    engine_ = TrtPtr<nvinfer1::ICudaEngine>(
        runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size()),
        &TrtDelete<nvinfer1::ICudaEngine>);
    if (!engine_) {
      std::fprintf(stderr, "[reid_trt] engine 反序列化失败: %s\n",
                   cfg_.model_path.c_str());
      return false;
    }
    context_ = TrtPtr<nvinfer1::IExecutionContext>(engine_->createExecutionContext(),
                                                   &TrtDelete<nvinfer1::IExecutionContext>);
    if (!context_) {
      std::fprintf(stderr, "[reid_trt] 创建执行上下文失败\n");
      return false;
    }

    // IO 张量名按模式自查（静态图与导出一致：image_rgb_f32 / feature）
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char* name = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        input_name_ = name;
      } else {
        output_name_ = name;
      }
    }
    const nvinfer1::Dims in_dims = context_->getTensorShape(input_name_.c_str());
    const nvinfer1::Dims out_dims = context_->getTensorShape(output_name_.c_str());
    if (in_dims.nbDims != 4 || out_dims.nbDims != 2) {
      std::fprintf(stderr, "[reid_trt] IO 形状异常（in %d 维 / out %d 维）\n",
                   in_dims.nbDims, out_dims.nbDims);
      unload();
      return false;
    }
    in_elems_ = static_cast<size_t>(Volume(in_dims));
    out_elems_ = static_cast<size_t>(Volume(out_dims));
    if (in_dims.d[2] != cfg_.input_height || in_dims.d[3] != cfg_.input_width) {
      std::fprintf(stderr,
                   "[reid_trt] engine 输入尺寸 %ldx%ld 与配置 %dx%d 不符\n",
                   static_cast<long>(in_dims.d[3]), static_cast<long>(in_dims.d[2]),
                   cfg_.input_width, cfg_.input_height);
      unload();
      return false;
    }

    if (cudaMalloc(&dev_in_, in_elems_ * sizeof(float)) != cudaSuccess ||
        cudaMalloc(&dev_out_, out_elems_ * sizeof(float)) != cudaSuccess ||
        cudaStreamCreate(&stream_) != cudaSuccess) {
      std::fprintf(stderr, "[reid_trt] CUDA 显存/流分配失败\n");
      unload();
      return false;
    }
    context_->setTensorAddress(input_name_.c_str(), dev_in_);
    context_->setTensorAddress(output_name_.c_str(), dev_out_);

    // 预热一次（含首次 kernel 编译/调度开销）
    const std::vector<float> f = extract(
        cv::Mat::zeros(cfg_.input_height, cfg_.input_width, CV_8UC3));
    if (f.empty()) {
      std::fprintf(stderr, "[reid_trt] 预热推理失败\n");
      unload();
      return false;
    }
    loaded_ = true;
    std::fprintf(stderr,
                 "[reid_trt] engine 就绪：%s（输入 %s [%ld,%ld,%ld,%ld], "
                 "输出 %s dim=%zu）\n",
                 cfg_.model_path.c_str(), input_name_.c_str(),
                 static_cast<long>(in_dims.d[0]), static_cast<long>(in_dims.d[1]),
                 static_cast<long>(in_dims.d[2]), static_cast<long>(in_dims.d[3]),
                 output_name_.c_str(), out_elems_);
    return true;
  }

  void unload() override {
    if (stream_ != nullptr) {
      cudaStreamSynchronize(stream_);
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
    if (dev_in_ != nullptr) {
      cudaFree(dev_in_);
      dev_in_ = nullptr;
    }
    if (dev_out_ != nullptr) {
      cudaFree(dev_out_);
      dev_out_ = nullptr;
    }
    context_.reset();
    engine_.reset();
    runtime_.reset();
    if (loaded_) {
      loaded_ = false;
      std::fprintf(stderr, "[reid_trt] engine 已卸载（显存让位）\n");
    }
  }

  bool loaded() const override { return loaded_; }

  std::vector<float> extract(const cv::Mat& bgr_crop) override {
    if (!context_ || dev_in_ == nullptr || bgr_crop.empty()) {
      return {};
    }
    // 与 OpenCV 后端完全一致的预处理：BGR 0-255 -> NCHW float32 RGB
    const cv::Mat blob = cv::dnn::blobFromImage(
        bgr_crop, 1.0, cv::Size(cfg_.input_width, cfg_.input_height),
        cv::Scalar(), /*swapRB=*/true, /*crop=*/false, CV_32F);
    if (blob.total() * blob.elemSize() != in_elems_ * sizeof(float)) {
      return {};
    }
    if (cudaMemcpyAsync(dev_in_, blob.data, in_elems_ * sizeof(float),
                        cudaMemcpyHostToDevice, stream_) != cudaSuccess) {
      return {};
    }
    if (!context_->enqueueV3(stream_)) {
      return {};
    }
    if (cudaStreamSynchronize(stream_) != cudaSuccess) {
      return {};
    }
    std::vector<float> feat(out_elems_, 0.0F);
    if (cudaMemcpy(feat.data(), dev_out_, out_elems_ * sizeof(float),
                   cudaMemcpyDeviceToHost) != cudaSuccess) {
      return {};
    }
    // 图内已 L2 归一化，防御性再归一化一次（幂等）
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
  TrtLogger logger_;
  bool loaded_ = false;

  TrtPtr<nvinfer1::IRuntime> runtime_{nullptr, &TrtDelete<nvinfer1::IRuntime>};
  TrtPtr<nvinfer1::ICudaEngine> engine_{nullptr, &TrtDelete<nvinfer1::ICudaEngine>};
  TrtPtr<nvinfer1::IExecutionContext> context_{
      nullptr, &TrtDelete<nvinfer1::IExecutionContext>};
  std::string input_name_{"image_rgb_f32"};
  std::string output_name_{"feature"};
  size_t in_elems_ = 0;
  size_t out_elems_ = 0;
  void* dev_in_ = nullptr;
  void* dev_out_ = nullptr;
  cudaStream_t stream_ = nullptr;
};

std::unique_ptr<FeatureExtractor> CreateTrtFeatureExtractor(
    const FeatureExtractor::Config& cfg) {
  return std::make_unique<TrtFeatureExtractor>(cfg);
}

}  // namespace dog_patrol_reid_provider

#else  // !REID_HAS_TENSORRT

#include <cstdio>

namespace dog_patrol_reid_provider {

// 未编入 TRT 时提供占位实现，保证符号存在（正常路径下 Create 不会走到这里）
std::unique_ptr<FeatureExtractor> CreateTrtFeatureExtractor(
    const FeatureExtractor::Config&) {
  std::fprintf(stderr,
               "[reid_trt] 本构建未启用 TensorRT 后端（REID_HAS_TENSORRT 未定义）\n");
  return nullptr;
}

}  // namespace dog_patrol_reid_provider

#endif  // REID_HAS_TENSORRT
