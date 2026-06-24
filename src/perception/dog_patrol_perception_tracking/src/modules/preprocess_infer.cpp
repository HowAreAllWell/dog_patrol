#include "vision_demo_host/modules/preprocess_infer.hpp"

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace vision_demo_host {
namespace {

class TrtLogger : public nvinfer1::ILogger {
 public:
  void log(const Severity severity, const char *msg) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

inline std::size_t ElementCount(const nvinfer1::Dims &dims) {
  std::size_t count = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    count *= static_cast<std::size_t>(dims.d[i]);
  }
  return count;
}

inline float ClampFloat(const float v, const float low, const float high) {
  return std::max(low, std::min(v, high));
}

struct LetterboxTransform {
  float scale{1.0F};
  float pad_x{0.0F};
  float pad_y{0.0F};
  int resized_w{0};
  int resized_h{0};
};

LetterboxTransform ApplyLetterbox(const cv::Mat &frame, const int input_w, const int input_h, cv::Mat *out) {
  LetterboxTransform tf;
  if (out == nullptr || frame.empty() || input_w <= 0 || input_h <= 0) {
    return tf;
  }

  const float scale_w = static_cast<float>(input_w) / static_cast<float>(frame.cols);
  const float scale_h = static_cast<float>(input_h) / static_cast<float>(frame.rows);
  tf.scale = std::min(scale_w, scale_h);
  tf.resized_w = std::max(1, static_cast<int>(std::round(static_cast<float>(frame.cols) * tf.scale)));
  tf.resized_h = std::max(1, static_cast<int>(std::round(static_cast<float>(frame.rows) * tf.scale)));

  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(tf.resized_w, tf.resized_h));

  tf.pad_x = static_cast<float>(input_w - tf.resized_w) * 0.5F;
  tf.pad_y = static_cast<float>(input_h - tf.resized_h) * 0.5F;

  const int left = std::max(0, static_cast<int>(std::floor(tf.pad_x)));
  const int right = std::max(0, input_w - tf.resized_w - left);
  const int top = std::max(0, static_cast<int>(std::floor(tf.pad_y)));
  const int bottom = std::max(0, input_h - tf.resized_h - top);
  cv::copyMakeBorder(resized, *out, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  tf.pad_x = static_cast<float>(left);
  tf.pad_y = static_cast<float>(top);
  return tf;
}

}  // namespace

struct PreprocessInfer::Impl {
  Config config;
  TrtLogger logger;

  std::unique_ptr<nvinfer1::IRuntime> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> engine;
  std::unique_ptr<nvinfer1::IExecutionContext> context;

  std::string input_name;
  std::string output_name;
  nvinfer1::Dims input_dims{};
  nvinfer1::Dims output_dims{};

  void *device_input{nullptr};
  void *device_output{nullptr};
  std::size_t input_bytes{0};
  std::size_t output_bytes{0};
  cudaStream_t stream{nullptr};

  bool initialized{false};

  explicit Impl(Config cfg) : config(std::move(cfg)) {}

  ~Impl() {
    if (device_input != nullptr) {
      cudaFree(device_input);
      device_input = nullptr;
    }
    if (device_output != nullptr) {
      cudaFree(device_output);
      device_output = nullptr;
    }
    if (stream != nullptr) {
      cudaStreamDestroy(stream);
      stream = nullptr;
    }
  }

  bool LoadEngineWithUltralyticsHeader(const std::vector<std::uint8_t> &blob) {
    if (runtime == nullptr) {
      return false;
    }

    auto deserialize_at = [&](const std::size_t offset) -> bool {
      if (offset >= blob.size()) {
        return false;
      }
      const std::size_t size = blob.size() - offset;
      nvinfer1::ICudaEngine *raw = runtime->deserializeCudaEngine(blob.data() + offset, size);
      if (raw == nullptr) {
        return false;
      }
      engine.reset(raw);
      return true;
    };

    // Ultralytics TensorRT engine files may store: [4-byte metadata length][JSON metadata][TRT plan].
    if (blob.size() > 4) {
      std::uint32_t meta_len = 0;
      std::memcpy(&meta_len, blob.data(), sizeof(std::uint32_t));
      const std::size_t offset = static_cast<std::size_t>(4) + static_cast<std::size_t>(meta_len);
      if (offset < blob.size() && blob[4] == '{' && deserialize_at(offset)) {
        return true;
      }
    }

    return deserialize_at(0);
  }

  bool InitBindings(std::string *error) {
    if (engine == nullptr) {
      if (error != nullptr) {
        *error = "TensorRT engine is null after deserialization.";
      }
      return false;
    }

    context.reset(engine->createExecutionContext());
    if (context == nullptr) {
      if (error != nullptr) {
        *error = "Failed to create TensorRT execution context.";
      }
      return false;
    }

    const int io_count = engine->getNbIOTensors();
    for (int i = 0; i < io_count; ++i) {
      const char *name = engine->getIOTensorName(i);
      if (name == nullptr) {
        continue;
      }

      if (engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        input_name = name;
      } else {
        output_name = name;
      }
    }

    if (input_name.empty() || output_name.empty()) {
      if (error != nullptr) {
        *error = "Failed to find TensorRT input/output tensor names.";
      }
      return false;
    }

    input_dims = engine->getTensorShape(input_name.c_str());
    if (input_dims.nbDims != 4) {
      if (error != nullptr) {
        *error = "Unexpected input dims. Expected 4D BCHW.";
      }
      return false;
    }

    if (input_dims.d[0] == -1 || input_dims.d[2] == -1 || input_dims.d[3] == -1) {
      nvinfer1::Dims fixed = input_dims;
      fixed.d[0] = 1;
      fixed.d[2] = config.input_height;
      fixed.d[3] = config.input_width;
      if (!context->setInputShape(input_name.c_str(), fixed)) {
        if (error != nullptr) {
          *error = "Failed to set dynamic TensorRT input shape.";
        }
        return false;
      }
      input_dims = fixed;
    }

    output_dims = context->getTensorShape(output_name.c_str());
    if (output_dims.nbDims < 2) {
      if (error != nullptr) {
        *error = "Unexpected output dims from TensorRT engine.";
      }
      return false;
    }

    input_bytes = ElementCount(input_dims) * sizeof(float);
    output_bytes = ElementCount(output_dims) * sizeof(float);

    if (cudaMalloc(&device_input, input_bytes) != cudaSuccess) {
      if (error != nullptr) {
        *error = "cudaMalloc failed for input buffer.";
      }
      return false;
    }

    if (cudaMalloc(&device_output, output_bytes) != cudaSuccess) {
      if (error != nullptr) {
        *error = "cudaMalloc failed for output buffer.";
      }
      return false;
    }

    if (cudaStreamCreate(&stream) != cudaSuccess) {
      if (error != nullptr) {
        *error = "cudaStreamCreate failed.";
      }
      return false;
    }

    if (!context->setTensorAddress(input_name.c_str(), device_input) ||
        !context->setTensorAddress(output_name.c_str(), device_output)) {
      if (error != nullptr) {
        *error = "Failed to bind TensorRT tensor addresses.";
      }
      return false;
    }

    initialized = true;
    return true;
  }
};

PreprocessInfer::PreprocessInfer(Config config)
    : config_(std::move(config)), impl_(std::make_unique<Impl>(config_)) {}

PreprocessInfer::~PreprocessInfer() = default;

PreprocessInfer::PreprocessInfer(PreprocessInfer &&) noexcept = default;
PreprocessInfer &PreprocessInfer::operator=(PreprocessInfer &&) noexcept = default;

bool PreprocessInfer::Initialize(std::string *error) {
  if (config_.detector_runtime_path.empty()) {
    if (error != nullptr) {
      *error = "detector.runtime_path is empty.";
    }
    return false;
  }

  const std::filesystem::path runtime_path(config_.detector_runtime_path);
  if (!std::filesystem::exists(runtime_path)) {
    if (error != nullptr) {
      *error = "Detector runtime path does not exist: " + config_.detector_runtime_path;
    }
    return false;
  }

  if (config_.enable_fake_detection) {
    impl_->initialized = true;
    return true;
  }

  std::ifstream ifs(config_.detector_runtime_path, std::ios::binary);
  if (!ifs.good()) {
    if (error != nullptr) {
      *error = "Failed to open detector engine file: " + config_.detector_runtime_path;
    }
    return false;
  }

  std::vector<std::uint8_t> blob((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  if (blob.empty()) {
    if (error != nullptr) {
      *error = "Detector engine file is empty: " + config_.detector_runtime_path;
    }
    return false;
  }

  impl_->runtime.reset(nvinfer1::createInferRuntime(impl_->logger));
  if (impl_->runtime == nullptr) {
    if (error != nullptr) {
      *error = "Failed to create TensorRT runtime.";
    }
    return false;
  }

  if (!impl_->LoadEngineWithUltralyticsHeader(blob)) {
    if (error != nullptr) {
      *error = "Failed to deserialize TensorRT engine (including Ultralytics header handling).";
    }
    return false;
  }

  if (!impl_->InitBindings(error)) {
    return false;
  }

  return true;
}

std::vector<Detection> PreprocessInfer::Infer(const cv::Mat &frame) {
  std::vector<Detection> detections;

  if (frame.empty() || !impl_->initialized) {
    return detections;
  }

  if (config_.enable_fake_detection) {
    const float width = static_cast<float>(frame.cols);
    const float height = static_cast<float>(frame.rows);

    Detection person;
    person.class_id = ClassId::kPerson;
    person.confidence = 0.9F;
    person.bbox = cv::Rect2f(width * 0.35F, height * 0.2F, width * 0.25F, height * 0.6F);
    detections.push_back(person);

    Detection car;
    car.class_id = ClassId::kCar;
    car.confidence = 0.8F;
    car.bbox = cv::Rect2f(width * 0.05F, height * 0.65F, width * 0.3F, height * 0.25F);
    detections.push_back(car);
    return detections;
  }

  const int input_h = impl_->input_dims.d[2];
  const int input_w = impl_->input_dims.d[3];

  cv::Mat resized;
  const LetterboxTransform tf = ApplyLetterbox(frame, input_w, input_h, &resized);
  if (resized.empty()) {
    return detections;
  }

  cv::Mat rgb;
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

  cv::Mat float_img;
  rgb.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

  std::vector<float> chw(static_cast<std::size_t>(3) * static_cast<std::size_t>(input_h) *
                         static_cast<std::size_t>(input_w));
  std::vector<cv::Mat> channels(3);
  for (int i = 0; i < 3; ++i) {
    channels[i] = cv::Mat(input_h, input_w, CV_32FC1, chw.data() +
                                                        static_cast<std::size_t>(i) * input_h * input_w);
  }
  cv::split(float_img, channels);

  if (cudaMemcpyAsync(impl_->device_input, chw.data(), impl_->input_bytes, cudaMemcpyHostToDevice,
                      impl_->stream) != cudaSuccess) {
    return detections;
  }

  if (!impl_->context->enqueueV3(impl_->stream)) {
    return detections;
  }

  std::vector<float> output(impl_->output_bytes / sizeof(float));
  if (cudaMemcpyAsync(output.data(), impl_->device_output, impl_->output_bytes, cudaMemcpyDeviceToHost,
                      impl_->stream) != cudaSuccess) {
    return detections;
  }

  if (cudaStreamSynchronize(impl_->stream) != cudaSuccess) {
    return detections;
  }

  const std::size_t rows = output.size() / 6;
  detections.reserve(rows);

  for (std::size_t i = 0; i < rows; ++i) {
    const float x1 = (output[i * 6 + 0] - tf.pad_x) / tf.scale;
    const float y1 = (output[i * 6 + 1] - tf.pad_y) / tf.scale;
    const float x2 = (output[i * 6 + 2] - tf.pad_x) / tf.scale;
    const float y2 = (output[i * 6 + 3] - tf.pad_y) / tf.scale;
    const float conf = output[i * 6 + 4];
    const int cls = static_cast<int>(output[i * 6 + 5]);

    if (conf < config_.raw_conf_threshold) {
      continue;
    }

    ClassId class_id = ClassId::kUnknown;
    if (cls == 0) {
      class_id = ClassId::kPerson;
    } else if (cls == 2) {
      class_id = ClassId::kCar;
    } else {
      continue;
    }

    const float bx = ClampFloat(std::min(x1, x2), 0.0F, static_cast<float>(frame.cols - 1));
    const float by = ClampFloat(std::min(y1, y2), 0.0F, static_cast<float>(frame.rows - 1));
    const float bw = ClampFloat(std::abs(x2 - x1), 0.0F, static_cast<float>(frame.cols) - bx);
    const float bh = ClampFloat(std::abs(y2 - y1), 0.0F, static_cast<float>(frame.rows) - by);

    if (bw <= 1.0F || bh <= 1.0F) {
      continue;
    }

    Detection det;
    det.class_id = class_id;
    det.confidence = conf;
    det.bbox = cv::Rect2f(bx, by, bw, bh);
    detections.push_back(det);
  }

  return detections;
}

}  // namespace vision_demo_host
