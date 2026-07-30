#include <MvCameraControl.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "vision_demo_host/modules/ffv1_capture_artifact_writer.hpp"
#include "vision_demo_host/modules/preprocess_infer.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;

constexpr unsigned int kSupportedTransportLayers =
    MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE |
    MV_GENTL_XOF_DEVICE;

struct Options {
  std::filesystem::path output_directory;
  std::string detector_engine;
  std::filesystem::path input_video;
  int width{1280};
  int height{1024};
  double fps{30.0};
  unsigned int warmup_frames{30U};
  unsigned int frames{90U};
  int timeout_ms{1000};
  float detector_raw_confidence{0.10F};
  std::string mvs_model{"MV-CU013-A0UC"};
  std::string mvs_serial;
};

struct RawFrame {
  std::vector<unsigned char> bayer;
  vision_demo_host::CameraIngest::AcquiredFrame metadata;
};

struct PixelDifferenceSummary {
  std::uint64_t samples{0};
  double mean_absolute_difference{0.0};
  unsigned int p50_absolute_difference{0};
  unsigned int p95_absolute_difference{0};
  unsigned int p99_absolute_difference{0};
  unsigned int max_absolute_difference{0};
};

struct DetectionFrame {
  std::vector<vision_demo_host::Detection> detections;
};

struct DetectionDifferenceSummary {
  std::size_t different_frames{0};
  std::size_t different_detection_counts{0};
  std::size_t comparable_detections{0};
  double mean_confidence_delta{0.0};
  double mean_bbox_l1_delta{0.0};
};

struct ModeReport {
  unsigned int quality{0};
  std::string name;
  bool supported{false};
  int set_quality_code{MV_OK};
  int conversion_failure_code{MV_OK};
  vision_demo_host::PreprocessInfer::PercentileSummary conversion;
  vision_demo_host::PreprocessInfer::MetricsSnapshot preprocess;
  double detector_throughput_fps{0.0};
  std::size_t detector_positive_frames{0};
  std::size_t detector_total_detections{0};
  std::uint64_t output_fnv1a{0};
  PixelDifferenceSummary pixel_difference;
  DetectionDifferenceSummary detection_difference;
  std::filesystem::path take_directory;
};

double ElapsedMilliseconds(const SteadyClock::time_point start) {
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}

std::uint64_t UnixNowNs() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}

std::string FormatSdkError(const int code) {
  char buffer[32]{};
  std::snprintf(buffer, sizeof(buffer), "0x%x", code);
  return buffer;
}

bool Fail(std::string *error, const std::string &message) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

bool ParseUnsigned(const std::string &value, unsigned int *out, std::string *error) {
  try {
    const unsigned long parsed = std::stoul(value);
    if (parsed > std::numeric_limits<unsigned int>::max()) {
      return Fail(error, "unsigned value out of range: " + value);
    }
    *out = static_cast<unsigned int>(parsed);
    return true;
  } catch (const std::exception &) {
    return Fail(error, "invalid unsigned value: " + value);
  }
}

bool ParseInt(const std::string &value, int *out, std::string *error) {
  try {
    *out = std::stoi(value);
    return true;
  } catch (const std::exception &) {
    return Fail(error, "invalid integer value: " + value);
  }
}

bool ParseDouble(const std::string &value, double *out, std::string *error) {
  try {
    *out = std::stod(value);
    return std::isfinite(*out) || Fail(error, "non-finite numeric value: " + value);
  } catch (const std::exception &) {
    return Fail(error, "invalid numeric value: " + value);
  }
}

bool ParseFloat(const std::string &value, float *out, std::string *error) {
  double parsed = 0.0;
  if (!ParseDouble(value, &parsed, error)) {
    return false;
  }
  *out = static_cast<float>(parsed);
  return true;
}

void PrintUsage() {
  std::cout
      << "Usage: benchmark_bayer_input --output-dir <path> --detector-engine <path> [options]\n"
      << "  --input-video <path>         synthesize BayerGB8 from this BGR video instead of camera capture\n"
      << "  --width <n>                (default: 1280)\n"
      << "  --height <n>               (default: 1024)\n"
      << "  --fps <n>                  (default: 30)\n"
      << "  --warmup-frames <n>        (default: 30)\n"
      << "  --frames <n>               (default: 90)\n"
      << "  --timeout-ms <n>           (default: 1000)\n"
      << "  --detector-raw-conf <n>    (default: 0.10)\n"
      << "  --mvs-model <name>         (default: MV-CU013-A0UC)\n"
      << "  --mvs-serial <serial>      (required if model selection is ambiguous)\n";
}

bool ParseOptions(const int argc, char **argv, Options *options, std::string *error) {
  if (options == nullptr) {
    return Fail(error, "options pointer is null");
  }
  const auto value = [&](const int &index, const std::string &argument) -> std::optional<std::string> {
    if (index + 1 >= argc) {
      Fail(error, "missing value for " + argument);
      return std::nullopt;
    }
    return std::string(argv[index + 1]);
  };

  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help" || argument == "-h") {
      PrintUsage();
      std::exit(0);
    }
    const std::optional<std::string> option_value = value(index, argument);
    if (!option_value.has_value()) {
      return false;
    }
    ++index;
    if (argument == "--output-dir") {
      options->output_directory = option_value.value();
    } else if (argument == "--detector-engine") {
      options->detector_engine = option_value.value();
    } else if (argument == "--input-video") {
      options->input_video = option_value.value();
    } else if (argument == "--width") {
      if (!ParseInt(option_value.value(), &options->width, error)) {
        return false;
      }
    } else if (argument == "--height") {
      if (!ParseInt(option_value.value(), &options->height, error)) {
        return false;
      }
    } else if (argument == "--fps") {
      if (!ParseDouble(option_value.value(), &options->fps, error)) {
        return false;
      }
    } else if (argument == "--warmup-frames") {
      if (!ParseUnsigned(option_value.value(), &options->warmup_frames, error)) {
        return false;
      }
    } else if (argument == "--frames") {
      if (!ParseUnsigned(option_value.value(), &options->frames, error)) {
        return false;
      }
    } else if (argument == "--timeout-ms") {
      if (!ParseInt(option_value.value(), &options->timeout_ms, error)) {
        return false;
      }
    } else if (argument == "--detector-raw-conf") {
      if (!ParseFloat(option_value.value(), &options->detector_raw_confidence, error)) {
        return false;
      }
    } else if (argument == "--mvs-model") {
      options->mvs_model = option_value.value();
    } else if (argument == "--mvs-serial") {
      options->mvs_serial = option_value.value();
    } else {
      return Fail(error, "unknown option: " + argument);
    }
  }

  if (options->output_directory.empty() || options->detector_engine.empty()) {
    return Fail(error, "--output-dir and --detector-engine are required");
  }
  if (options->width <= 0 || options->height <= 0 || options->fps <= 0.0 ||
      options->frames == 0U || options->timeout_ms <= 0 ||
      !std::isfinite(options->detector_raw_confidence) || options->detector_raw_confidence < 0.0F) {
    return Fail(error, "invalid benchmark dimensions, timing, frame count, or detector threshold");
  }
  return true;
}

std::string BayerQualityName(const unsigned int quality) {
  switch (quality) {
    case 0U:
      return "fast";
    case 1U:
      return "balanced";
    case 2U:
      return "optimal";
    case 3U:
      return "optimal_plus";
    default:
      return "unknown_" + std::to_string(quality);
  }
}

std::string DecodeCharArray(const char *source, const std::size_t size) {
  if (source == nullptr) {
    return "";
  }
  std::size_t length = 0;
  while (length < size && source[length] != '\0') {
    ++length;
  }
  return std::string(source, length);
}

std::string DeviceModelName(const MV_CC_DEVICE_INFO &device) {
  if (device.nTLayerType == MV_USB_DEVICE) {
    return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stUsb3VInfo.chModelName),
                           sizeof(device.SpecialInfo.stUsb3VInfo.chModelName));
  }
  if (device.nTLayerType == MV_GIGE_DEVICE || device.nTLayerType == MV_GENTL_GIGE_DEVICE) {
    return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stGigEInfo.chModelName),
                           sizeof(device.SpecialInfo.stGigEInfo.chModelName));
  }
  return "";
}

std::string DeviceSerialNumber(const MV_CC_DEVICE_INFO &device) {
  if (device.nTLayerType == MV_USB_DEVICE) {
    return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stUsb3VInfo.chSerialNumber),
                           sizeof(device.SpecialInfo.stUsb3VInfo.chSerialNumber));
  }
  if (device.nTLayerType == MV_GIGE_DEVICE || device.nTLayerType == MV_GENTL_GIGE_DEVICE) {
    return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stGigEInfo.chSerialNumber),
                           sizeof(device.SpecialInfo.stGigEInfo.chSerialNumber));
  }
  return "";
}

bool SelectDevice(const MV_CC_DEVICE_INFO_LIST &devices,
                  const Options &options,
                  MV_CC_DEVICE_INFO **device,
                  std::string *model,
                  std::string *serial,
                  std::string *error) {
  if (device == nullptr || model == nullptr || serial == nullptr) {
    return Fail(error, "device selection output pointers are null");
  }
  MV_CC_DEVICE_INFO *selected = nullptr;
  std::string selected_model;
  std::string selected_serial;
  std::ostringstream available;
  std::size_t matches = 0U;
  for (unsigned int index = 0; index < devices.nDeviceNum; ++index) {
    MV_CC_DEVICE_INFO *candidate = devices.pDeviceInfo[index];
    if (candidate == nullptr) {
      continue;
    }
    const std::string candidate_model = DeviceModelName(*candidate);
    const std::string candidate_serial = DeviceSerialNumber(*candidate);
    if (available.tellp() > 0) {
      available << ", ";
    }
    available << '[' << index << "] " << candidate_model << '(' << candidate_serial << ')';
    if ((!options.mvs_model.empty() && candidate_model != options.mvs_model) ||
        (!options.mvs_serial.empty() && candidate_serial != options.mvs_serial)) {
      continue;
    }
    ++matches;
    selected = candidate;
    selected_model = candidate_model;
    selected_serial = candidate_serial;
  }
  if (matches == 0U) {
    return Fail(error, "no Hik camera matched model/serial filter; available=" + available.str());
  }
  if (matches > 1U) {
    return Fail(error, "Hik camera selection is ambiguous; pass --mvs-serial; matches=" + available.str());
  }
  *device = selected;
  *model = std::move(selected_model);
  *serial = std::move(selected_serial);
  return true;
}

void WriteNodeFacts(std::ostream &out, void *handle) {
  const auto write_bool = [&](const char *key) {
    bool value = false;
    const int code = MV_CC_GetBoolValue(handle, key, &value);
    out << "  " << key << ": {kind: bool, supported: " << (code == MV_OK ? "true" : "false")
        << ", code: " << FormatSdkError(code);
    if (code == MV_OK) {
      out << ", value: " << (value ? "true" : "false");
    }
    out << "}\n";
  };
  const auto write_float = [&](const char *key) {
    MVCC_FLOATVALUE value{};
    const int code = MV_CC_GetFloatValue(handle, key, &value);
    out << "  " << key << ": {kind: float, supported: " << (code == MV_OK ? "true" : "false")
        << ", code: " << FormatSdkError(code);
    if (code == MV_OK) {
      out << ", value: " << value.fCurValue << ", range: [" << value.fMin << ", " << value.fMax
          << "]";
    }
    out << "}\n";
  };
  const auto write_enum = [&](const char *key) {
    MVCC_ENUMVALUE value{};
    const int code = MV_CC_GetEnumValue(handle, key, &value);
    out << "  " << key << ": {kind: enum, supported: " << (code == MV_OK ? "true" : "false")
        << ", code: " << FormatSdkError(code);
    if (code == MV_OK) {
      out << ", value: " << value.nCurValue;
    }
    out << "}\n";
  };

  out << "camera_image_control_nodes:\n";
  write_bool("GammaEnable");
  write_enum("GammaSelector");
  write_float("Gamma");
  write_bool("ColorTransformationEnable");
  write_enum("ColorTransformationSelector");
  write_float("ColorTransformationValue");
  write_bool("CCMEnable");
  write_bool("ColorCorrectionEnable");
}

PixelDifferenceSummary SummarizePixelDifferences(const std::array<std::uint64_t, 256U> &histogram,
                                                 const std::uint64_t total,
                                                 const std::uint64_t absolute_sum,
                                                 const unsigned int maximum) {
  PixelDifferenceSummary summary;
  summary.samples = total;
  if (total == 0U) {
    return summary;
  }
  summary.mean_absolute_difference = static_cast<double>(absolute_sum) / static_cast<double>(total);
  summary.max_absolute_difference = maximum;
  const auto percentile = [&histogram, total](const double fraction) {
    const std::uint64_t required_rank = static_cast<std::uint64_t>(std::ceil(fraction * total));
    std::uint64_t observed = 0;
    for (unsigned int difference = 0; difference < histogram.size(); ++difference) {
      observed += histogram[difference];
      if (observed >= required_rank) {
        return difference;
      }
    }
    return static_cast<unsigned int>(histogram.size() - 1U);
  };
  summary.p50_absolute_difference = percentile(0.50);
  summary.p95_absolute_difference = percentile(0.95);
  summary.p99_absolute_difference = percentile(0.99);
  return summary;
}

DetectionDifferenceSummary CompareDetections(const std::vector<DetectionFrame> &reference,
                                             const std::vector<DetectionFrame> &candidate) {
  DetectionDifferenceSummary summary;
  const std::size_t frame_count = std::min(reference.size(), candidate.size());
  double confidence_total = 0.0;
  double bbox_total = 0.0;
  for (std::size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
    const auto &reference_detections = reference[frame_index].detections;
    const auto &candidate_detections = candidate[frame_index].detections;
    if (reference_detections.size() != candidate_detections.size()) {
      ++summary.different_frames;
      ++summary.different_detection_counts;
      continue;
    }
    bool frame_differs = false;
    for (std::size_t detection_index = 0; detection_index < reference_detections.size(); ++detection_index) {
      const auto &left = reference_detections[detection_index];
      const auto &right = candidate_detections[detection_index];
      const double confidence_delta = std::abs(static_cast<double>(left.confidence - right.confidence));
      const double bbox_delta =
          std::abs(static_cast<double>(left.bbox.x - right.bbox.x)) +
          std::abs(static_cast<double>(left.bbox.y - right.bbox.y)) +
          std::abs(static_cast<double>(left.bbox.width - right.bbox.width)) +
          std::abs(static_cast<double>(left.bbox.height - right.bbox.height));
      confidence_total += confidence_delta;
      bbox_total += bbox_delta;
      ++summary.comparable_detections;
      frame_differs = frame_differs || left.class_id != right.class_id || confidence_delta > 1e-6 || bbox_delta > 1e-6;
    }
    if (frame_differs) {
      ++summary.different_frames;
    }
  }
  if (summary.comparable_detections > 0U) {
    summary.mean_confidence_delta = confidence_total / static_cast<double>(summary.comparable_detections);
    summary.mean_bbox_l1_delta = bbox_total / static_cast<double>(summary.comparable_detections);
  }
  return summary;
}

std::uint64_t Fnv1a(const std::vector<unsigned char> &bytes, std::uint64_t seed) {
  std::uint64_t hash = seed;
  for (const unsigned char value : bytes) {
    hash ^= value;
    hash *= 1099511628211ULL;
  }
  return hash;
}

bool ConfigureCamera(void *handle, const Options &options, std::string *error) {
  const auto require = [&](const int code, const char *operation) {
    return code == MV_OK || Fail(error, std::string(operation) + " failed: " + FormatSdkError(code));
  };
  return require(MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF), "TriggerMode") &&
         require(MV_CC_SetIntValueEx(handle, "Width", options.width), "Width") &&
         require(MV_CC_SetIntValueEx(handle, "Height", options.height), "Height") &&
         require(MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true),
                 "AcquisitionFrameRateEnable") &&
         require(MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", static_cast<float>(options.fps)),
                 "AcquisitionFrameRate") &&
         require(MV_CC_SetBayerFilterEnable(handle, false), "MV_CC_SetBayerFilterEnable(false)");
}

bool CaptureRawFrames(void *handle,
                      const Options &options,
                      std::vector<RawFrame> *frames,
                      vision_demo_host::PreprocessInfer::StageTiming *acquisition_timing,
                      vision_demo_host::PreprocessInfer::StageTiming *raw_copy_timing,
                      std::string *error) {
  if (frames == nullptr || acquisition_timing == nullptr || raw_copy_timing == nullptr) {
    return Fail(error, "benchmark output pointers are null");
  }
  const unsigned int requested = options.warmup_frames + options.frames;
  for (unsigned int index = 0; index < requested; ++index) {
    MV_FRAME_OUT source{};
    const auto acquisition_start = SteadyClock::now();
    const int get_code = MV_CC_GetImageBuffer(handle, &source, static_cast<unsigned int>(options.timeout_ms));
    const double acquisition_ms = ElapsedMilliseconds(acquisition_start);
    if (get_code != MV_OK || source.pBufAddr == nullptr) {
      return Fail(error, "MV_CC_GetImageBuffer failed: " + FormatSdkError(get_code));
    }
    const unsigned int source_length = source.stFrameInfo.nFrameLen;
    if (source_length == 0U) {
      MV_CC_FreeImageBuffer(handle, &source);
      return Fail(error, "camera returned an empty Bayer frame");
    }

    if (index >= options.warmup_frames) {
      RawFrame captured;
      captured.metadata.source_timestamp_ns = UnixNowNs();
      captured.metadata.sdk_host_timestamp = source.stFrameInfo.nHostTimeStamp;
      captured.metadata.camera_frame_number = source.stFrameInfo.nFrameNum;
      captured.metadata.camera_frame_number_available = true;
      captured.metadata.device_timestamp_ticks =
          (static_cast<std::uint64_t>(source.stFrameInfo.nDevTimeStampHigh) << 32U) |
          source.stFrameInfo.nDevTimeStampLow;
      captured.metadata.source_pixel_type = static_cast<std::uint32_t>(source.stFrameInfo.enPixelType);
      captured.metadata.source_pixel_type_name = vision_demo_host::CameraIngest::PixelTypeName(
          captured.metadata.source_pixel_type);
      captured.metadata.width = static_cast<int>(source.stFrameInfo.nWidth);
      captured.metadata.height = static_cast<int>(source.stFrameInfo.nHeight);
      captured.metadata.source_payload_bytes = source.stFrameInfo.nFrameLenEx > 0U
                                                   ? source.stFrameInfo.nFrameLenEx
                                                   : source.stFrameInfo.nFrameLen;
      captured.metadata.camera_lost_packets = source.stFrameInfo.nLostPacket;
      const auto copy_start = SteadyClock::now();
      captured.bayer.assign(source.pBufAddr, source.pBufAddr + source_length);
      raw_copy_timing->ObserveMilliseconds(ElapsedMilliseconds(copy_start));
      acquisition_timing->ObserveMilliseconds(acquisition_ms);
      frames->push_back(std::move(captured));
    }

    const int free_code = MV_CC_FreeImageBuffer(handle, &source);
    if (free_code != MV_OK) {
      return Fail(error, "MV_CC_FreeImageBuffer failed: " + FormatSdkError(free_code));
    }
  }
  return true;
}

bool SynthesizeBayerFrames(const Options &options,
                           std::vector<RawFrame> *frames,
                           vision_demo_host::PreprocessInfer::StageTiming *derive_timing,
                           std::string *error) {
  if (frames == nullptr || derive_timing == nullptr) {
    return Fail(error, "synthetic Bayer output pointers are null");
  }
  cv::VideoCapture capture(options.input_video.string());
  if (!capture.isOpened()) {
    return Fail(error, "failed to open input video: " + options.input_video.string());
  }
  cv::Mat bgr;
  for (unsigned int index = 0; index < options.warmup_frames; ++index) {
    if (!capture.read(bgr) || bgr.empty()) {
      return Fail(error, "input video ended during requested warmup");
    }
  }
  frames->reserve(options.frames);
  for (unsigned int index = 0; index < options.frames; ++index) {
    if (!capture.read(bgr) || bgr.empty()) {
      return Fail(error, "input video ended before requested Bayer sample count");
    }
    if (bgr.type() != CV_8UC3) {
      return Fail(error, "input video frame is not BGR8");
    }
    RawFrame synthetic;
    synthetic.metadata.source_timestamp_ns = 1'000'000'000ULL + static_cast<std::uint64_t>(std::llround(
        static_cast<double>(index) * 1'000'000'000.0 / options.fps));
    synthetic.metadata.camera_frame_number = index;
    synthetic.metadata.camera_frame_number_available = true;
    synthetic.metadata.source_pixel_type = static_cast<std::uint32_t>(PixelType_Gvsp_BayerGB8);
    synthetic.metadata.source_pixel_type_name = "BayerGB8_synthetic_from_bgr";
    synthetic.metadata.width = bgr.cols;
    synthetic.metadata.height = bgr.rows;
    synthetic.metadata.source_payload_bytes = bgr.total();
    synthetic.bayer.resize(bgr.total());
    const auto derive_start = SteadyClock::now();
    for (int row = 0; row < bgr.rows; ++row) {
      const auto *source = bgr.ptr<cv::Vec3b>(row);
      auto *destination = synthetic.bayer.data() + static_cast<std::size_t>(row) * bgr.cols;
      for (int column = 0; column < bgr.cols; ++column) {
        const cv::Vec3b &pixel = source[column];
        const bool even_row = (row % 2) == 0;
        const bool even_column = (column % 2) == 0;
        destination[column] = even_row
                                  ? (even_column ? pixel[1] : pixel[0])
                                  : (even_column ? pixel[2] : pixel[1]);
      }
    }
    derive_timing->ObserveMilliseconds(ElapsedMilliseconds(derive_start));
    frames->push_back(std::move(synthetic));
  }
  return true;
}

bool WriteDetectionCsv(const std::filesystem::path &path, const std::vector<DetectionFrame> &frames) {
  std::ofstream output(path);
  if (!output) {
    return false;
  }
  output << "frame_idx,det_idx,class_id,confidence,x,y,width,height\n";
  for (std::size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
    for (std::size_t detection_index = 0; detection_index < frames[frame_index].detections.size(); ++detection_index) {
      const auto &detection = frames[frame_index].detections[detection_index];
      output << frame_index << ',' << detection_index << ',' << static_cast<int>(detection.class_id) << ','
             << std::fixed << std::setprecision(6) << detection.confidence << ',' << detection.bbox.x << ','
             << detection.bbox.y << ',' << detection.bbox.width << ',' << detection.bbox.height << '\n';
    }
  }
  return static_cast<bool>(output);
}

void WritePercentiles(std::ostream &out,
                      const std::string &name,
                      const vision_demo_host::PreprocessInfer::PercentileSummary &summary) {
  out << "  " << name << ": {samples: " << summary.samples << ", p50_ms: " << summary.p50_ms
      << ", p95_ms: " << summary.p95_ms << ", p99_ms: " << summary.p99_ms << "}\n";
}

bool ConvertAndMeasureMode(void *handle,
                           const Options &options,
                           const std::vector<RawFrame> &raw_frames,
                           const unsigned int quality,
                           vision_demo_host::PreprocessInfer *infer,
                           const std::filesystem::path &output_directory,
                           const std::vector<std::vector<unsigned char>> *optimal_bgr,
                           std::vector<std::vector<unsigned char>> *record_optimal_bgr,
                           std::vector<DetectionFrame> *record_detections,
                           ModeReport *report,
                           std::string *error) {
  if (infer == nullptr || record_detections == nullptr || report == nullptr || raw_frames.empty()) {
    return Fail(error, "invalid Bayer conversion benchmark inputs");
  }
  report->quality = quality;
  report->name = BayerQualityName(quality);
  report->set_quality_code = MV_CC_SetBayerCvtQuality(handle, quality);
  if (report->set_quality_code != MV_OK) {
    return true;
  }

  const RawFrame &first = raw_frames.front();
  const int width = first.metadata.width;
  const int height = first.metadata.height;
  const std::size_t output_bytes = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3U;
  std::vector<unsigned char> converted(output_bytes);
  vision_demo_host::PreprocessInfer::StageTiming conversion_timing;
  std::array<std::uint64_t, 256U> pixel_difference_histogram{};
  std::uint64_t pixel_difference_count = 0U;
  std::uint64_t pixel_difference_sum = 0U;
  unsigned int pixel_difference_maximum = 0U;

  vision_demo_host::Ffv1CaptureArtifactWriterFactory::Config writer_config;
  writer_config.session_directory = output_directory;
  writer_config.requested_fps = options.fps;
  writer_config.mvs_model = options.mvs_model;
  writer_config.mvs_serial = options.mvs_serial;
  writer_config.requested_width = width;
  writer_config.requested_height = height;
  writer_config.timeout_ms = options.timeout_ms;
  auto writer = vision_demo_host::Ffv1CaptureArtifactWriterFactory(writer_config).Create();
  vision_demo_host::CaptureTakeDescriptor descriptor;
  descriptor.sequence = quality + 1U;
  descriptor.name = "bayer_" + report->name;
  descriptor.started_wall_time_ns = UnixNowNs();
  vision_demo_host::CaptureFrameContract contract;
  contract.source_pixel_type = first.metadata.source_pixel_type;
  contract.source_pixel_type_name = first.metadata.source_pixel_type_name;
  contract.width = width;
  contract.height = height;
  contract.source_payload_bytes = first.metadata.source_payload_bytes;
  contract.bayer_interpolation = report->name;
  contract.bayer_smoothing = false;
  if (!writer->Begin(descriptor, contract, error)) {
    return false;
  }

  for (std::size_t warmup_index = 0; warmup_index < std::min<std::size_t>(5U, raw_frames.size()); ++warmup_index) {
    MV_CC_PIXEL_CONVERT_PARAM_EX convert{};
    convert.nWidth = static_cast<unsigned int>(width);
    convert.nHeight = static_cast<unsigned int>(height);
    convert.enSrcPixelType = static_cast<MvGvspPixelType>(raw_frames[warmup_index].metadata.source_pixel_type);
    convert.pSrcData = const_cast<unsigned char *>(raw_frames[warmup_index].bayer.data());
    convert.nSrcDataLen = static_cast<unsigned int>(raw_frames[warmup_index].bayer.size());
    convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    convert.pDstBuffer = converted.data();
    convert.nDstBufferSize = static_cast<unsigned int>(converted.size());
    if (MV_CC_ConvertPixelTypeEx(handle, &convert) != MV_OK) {
      return Fail(error, "Bayer conversion warmup failed for " + report->name);
    }
    (void)infer->Infer(cv::Mat(height, width, CV_8UC3, converted.data()));
  }
  infer->ResetMetrics();
  double detector_elapsed_ms = 0.0;

  for (std::size_t frame_index = 0; frame_index < raw_frames.size(); ++frame_index) {
    const RawFrame &raw = raw_frames[frame_index];
    if (raw.metadata.width != width || raw.metadata.height != height ||
        raw.metadata.source_pixel_type != first.metadata.source_pixel_type) {
      return Fail(error, "raw Bayer source contract changed within one benchmark capture");
    }
    MV_CC_PIXEL_CONVERT_PARAM_EX convert{};
    convert.nWidth = static_cast<unsigned int>(width);
    convert.nHeight = static_cast<unsigned int>(height);
    convert.enSrcPixelType = static_cast<MvGvspPixelType>(raw.metadata.source_pixel_type);
    convert.pSrcData = const_cast<unsigned char *>(raw.bayer.data());
    convert.nSrcDataLen = static_cast<unsigned int>(raw.bayer.size());
    convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    convert.pDstBuffer = converted.data();
    convert.nDstBufferSize = static_cast<unsigned int>(converted.size());
    const auto conversion_start = SteadyClock::now();
    const int conversion_code = MV_CC_ConvertPixelTypeEx(handle, &convert);
    conversion_timing.ObserveMilliseconds(ElapsedMilliseconds(conversion_start));
    if (conversion_code != MV_OK) {
      report->conversion_failure_code = conversion_code;
      return true;
    }

    report->output_fnv1a = Fnv1a(converted, report->output_fnv1a == 0U ? 1469598103934665603ULL : report->output_fnv1a);
    if (record_optimal_bgr != nullptr) {
      record_optimal_bgr->push_back(converted);
    }
    if (optimal_bgr != nullptr) {
      if (optimal_bgr->size() != raw_frames.size() || (*optimal_bgr)[frame_index].size() != converted.size()) {
        return Fail(error, "optimal Bayer reference does not match candidate output shape");
      }
      const auto &reference = (*optimal_bgr)[frame_index];
      for (std::size_t pixel_index = 0; pixel_index < converted.size(); ++pixel_index) {
        const unsigned int difference = static_cast<unsigned int>(std::abs(
            static_cast<int>(reference[pixel_index]) - static_cast<int>(converted[pixel_index])));
        ++pixel_difference_histogram[difference];
        ++pixel_difference_count;
        pixel_difference_sum += difference;
        pixel_difference_maximum = std::max(pixel_difference_maximum, difference);
      }
    }

    cv::Mat bgr(height, width, CV_8UC3, converted.data());
    if (frame_index == 0U && !cv::imwrite((output_directory / (report->name + "_first.png")).string(), bgr)) {
      return Fail(error, "failed to write first-frame PNG for " + report->name);
    }
    DetectionFrame detection_frame;
    const auto detector_frame_start = SteadyClock::now();
    detection_frame.detections = infer->Infer(bgr);
    detector_elapsed_ms += ElapsedMilliseconds(detector_frame_start);
    if (!detection_frame.detections.empty()) {
      ++report->detector_positive_frames;
    }
    report->detector_total_detections += detection_frame.detections.size();
    record_detections->push_back(std::move(detection_frame));

    vision_demo_host::CaptureFrame artifact_frame;
    artifact_frame.capture_index = frame_index;
    artifact_frame.source = raw.metadata;
    artifact_frame.source.bgr8 = bgr;
    if (!writer->Write(artifact_frame, error)) {
      return false;
    }
  }

  report->detector_throughput_fps = detector_elapsed_ms > 0.0
                                        ? 1000.0 * static_cast<double>(raw_frames.size()) / detector_elapsed_ms
                                        : 0.0;
  report->conversion = conversion_timing.Summary();
  report->preprocess = infer->Metrics();
  report->pixel_difference = SummarizePixelDifferences(
      pixel_difference_histogram, pixel_difference_count, pixel_difference_sum, pixel_difference_maximum);
  report->supported = true;
  report->take_directory = output_directory / descriptor.name;

  vision_demo_host::CaptureTakeSummary summary;
  summary.descriptor = descriptor;
  summary.frame_contract = contract;
  summary.complete = true;
  summary.writer_opened = true;
  summary.finished_wall_time_ns = UnixNowNs();
  summary.captured_frames = raw_frames.size();
  summary.written_frames = raw_frames.size();
  if (!writer->Finish(summary, error)) {
    return false;
  }
  if (!WriteDetectionCsv(output_directory / (report->name + "_detections.csv"), *record_detections)) {
    return Fail(error, "failed to write detector result CSV for " + report->name);
  }
  return true;
}

void WriteModeReport(std::ostream &out, const ModeReport &report) {
  out << "mode " << report.name << ":\n";
  out << "  quality_code: " << report.quality << "\n";
  out << "  set_quality_code: " << FormatSdkError(report.set_quality_code) << "\n";
  out << "  supported: " << (report.supported ? "true" : "false") << "\n";
  out << "  conversion_failure_code: " << FormatSdkError(report.conversion_failure_code) << "\n";
  if (!report.supported) {
    return;
  }
  WritePercentiles(out, "conversion", report.conversion);
  out << "  detector_throughput_fps: " << report.detector_throughput_fps << "\n";
  out << "  detector_positive_frames: " << report.detector_positive_frames << "\n";
  out << "  detector_total_detections: " << report.detector_total_detections << "\n";
  out << "  output_fnv1a: " << report.output_fnv1a << "\n";
  out << "  variant_take_directory: " << report.take_directory.string() << "\n";
  out << "  pixel_difference_from_optimal: {samples: " << report.pixel_difference.samples
      << ", mean_abs: " << report.pixel_difference.mean_absolute_difference
      << ", p50_abs: " << report.pixel_difference.p50_absolute_difference
      << ", p95_abs: " << report.pixel_difference.p95_absolute_difference
      << ", p99_abs: " << report.pixel_difference.p99_absolute_difference
      << ", max_abs: " << report.pixel_difference.max_absolute_difference << "}\n";
  out << "  detector_difference_from_optimal: {different_frames: "
      << report.detection_difference.different_frames << ", different_detection_counts: "
      << report.detection_difference.different_detection_counts << ", comparable_detections: "
      << report.detection_difference.comparable_detections << ", mean_confidence_delta: "
      << report.detection_difference.mean_confidence_delta << ", mean_bbox_l1_delta: "
      << report.detection_difference.mean_bbox_l1_delta << "}\n";
  out << "  detector_stages:\n";
  WritePercentiles(out, "resize", report.preprocess.resize);
  WritePercentiles(out, "border", report.preprocess.border);
  WritePercentiles(out, "channel_swap", report.preprocess.channel_swap);
  WritePercentiles(out, "normalize", report.preprocess.normalize);
  WritePercentiles(out, "layout", report.preprocess.layout);
  WritePercentiles(out, "h2d", report.preprocess.h2d);
  WritePercentiles(out, "tensor_rt", report.preprocess.tensor_rt);
  WritePercentiles(out, "d2h", report.preprocess.d2h);
  WritePercentiles(out, "parser", report.preprocess.parser);
  WritePercentiles(out, "total", report.preprocess.total);
}

}  // namespace

int main(int argc, char **argv) {
  Options options;
  std::string error;
  if (!ParseOptions(argc, argv, &options, &error)) {
    std::cerr << "benchmark_bayer_input: " << error << std::endl;
    PrintUsage();
    return 2;
  }
  std::error_code filesystem_error;
  if (!std::filesystem::create_directories(options.output_directory, filesystem_error) && filesystem_error) {
    std::cerr << "benchmark_bayer_input: failed to create output directory: " << filesystem_error.message()
              << std::endl;
    return 2;
  }
  const std::filesystem::path report_path = options.output_directory / "report.txt";
  std::ofstream report_file(report_path);
  if (!report_file) {
    std::cerr << "benchmark_bayer_input: failed to open report: " << report_path << std::endl;
    return 2;
  }

  const int initialize_code = MV_CC_Initialize();
  if (initialize_code != MV_OK) {
    std::cerr << "benchmark_bayer_input: MV_CC_Initialize failed: " << FormatSdkError(initialize_code) << std::endl;
    return 1;
  }
  void *handle = nullptr;
  const auto cleanup = [&]() {
    if (handle != nullptr) {
      MV_CC_StopGrabbing(handle);
      MV_CC_CloseDevice(handle);
      MV_CC_DestroyHandle(handle);
      handle = nullptr;
    }
    MV_CC_Finalize();
  };

  MV_CC_DEVICE_INFO_LIST devices{};
  const int enumerate_code = MV_CC_EnumDevices(kSupportedTransportLayers, &devices);
  if (enumerate_code != MV_OK || devices.nDeviceNum == 0U) {
    std::cerr << "benchmark_bayer_input: no Hik MVS device: " << FormatSdkError(enumerate_code) << std::endl;
    cleanup();
    return 1;
  }
  MV_CC_DEVICE_INFO *device = nullptr;
  std::string model;
  std::string serial;
  if (!SelectDevice(devices, options, &device, &model, &serial, &error)) {
    std::cerr << "benchmark_bayer_input: " << error << std::endl;
    cleanup();
    return 1;
  }
  const int create_code = MV_CC_CreateHandle(&handle, device);
  const int open_code = create_code == MV_OK ? MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0) : create_code;
  if (create_code != MV_OK || open_code != MV_OK) {
    std::cerr << "benchmark_bayer_input: failed to open camera: "
              << FormatSdkError(create_code != MV_OK ? create_code : open_code) << std::endl;
    cleanup();
    return 1;
  }
  if (!ConfigureCamera(handle, options, &error)) {
    std::cerr << "benchmark_bayer_input: " << error << std::endl;
    cleanup();
    return 1;
  }
  const int start_code = MV_CC_StartGrabbing(handle);
  if (start_code != MV_OK) {
    std::cerr << "benchmark_bayer_input: MV_CC_StartGrabbing failed: " << FormatSdkError(start_code) << std::endl;
    cleanup();
    return 1;
  }

  report_file << std::fixed << std::setprecision(6);
  report_file << "issue: 85\n";
  report_file << "sdk_version: 0x" << std::hex << MV_CC_GetSDKVersion() << std::dec << "\n";
  report_file << "camera_model: " << model << "\n";
  report_file << "camera_serial: " << serial << "\n";
  report_file << "request: " << options.width << 'x' << options.height << '@' << options.fps << "\n";
  report_file << "explicit_bayer_smoothing: false (MV_CC_SetBayerFilterEnable=false)\n";
  WriteNodeFacts(report_file, handle);

  std::vector<RawFrame> raw_frames;
  vision_demo_host::PreprocessInfer::StageTiming acquisition_timing;
  vision_demo_host::PreprocessInfer::StageTiming raw_copy_timing;
  const bool native_camera_source = options.input_video.empty();
  const bool captured = native_camera_source
                            ? CaptureRawFrames(handle, options, &raw_frames, &acquisition_timing,
                                               &raw_copy_timing, &error)
                            : SynthesizeBayerFrames(options, &raw_frames, &raw_copy_timing, &error);
  if (!captured) {
    std::cerr << "benchmark_bayer_input: " << error << std::endl;
    cleanup();
    return 1;
  }
  if (raw_frames.empty()) {
    std::cerr << "benchmark_bayer_input: no raw frames captured" << std::endl;
    cleanup();
    return 1;
  }
  const RawFrame &first = raw_frames.front();
  report_file << "source_pixel_type: " << first.metadata.source_pixel_type_name << " (0x" << std::hex
              << first.metadata.source_pixel_type << std::dec << ")\n";
  report_file << "source_origin: "
              << (native_camera_source ? "native_camera_Bayer_buffer" : "synthetic_BayerGB8_from_BGR_video")
              << "\n";
  if (!native_camera_source) {
    report_file << "synthetic_input_video: " << options.input_video.string() << "\n";
  }
  report_file << "source_payload_bytes: " << first.metadata.source_payload_bytes << "\n";
  report_file << "captured_identical_source_frames: " << raw_frames.size() << "\n";
  if (native_camera_source) {
    WritePercentiles(report_file, "acquisition", acquisition_timing.Summary());
    WritePercentiles(report_file, "raw_source_copy", raw_copy_timing.Summary());
  } else {
    WritePercentiles(report_file, "synthetic_bayer_derivation", raw_copy_timing.Summary());
  }

  vision_demo_host::PreprocessInfer::Config detector_config;
  detector_config.detector_runtime_path = options.detector_engine;
  detector_config.raw_conf_threshold = options.detector_raw_confidence;
  detector_config.enable_timing_metrics = true;
  vision_demo_host::PreprocessInfer infer(detector_config);
  if (!infer.Initialize(&error)) {
    std::cerr << "benchmark_bayer_input: detector initialization failed: " << error << std::endl;
    cleanup();
    return 1;
  }

  std::vector<std::vector<unsigned char>> optimal_bgr;
  std::vector<DetectionFrame> optimal_detections;
  std::vector<ModeReport> reports;
  const std::array<unsigned int, 4U> qualities{{2U, 0U, 1U, 3U}};
  for (const unsigned int quality : qualities) {
    ModeReport mode_report;
    std::vector<DetectionFrame> detections;
    const std::vector<std::vector<unsigned char>> *reference = quality == 2U ? nullptr : &optimal_bgr;
    std::vector<std::vector<unsigned char>> *record_reference = quality == 2U ? &optimal_bgr : nullptr;
    if (!ConvertAndMeasureMode(handle, options, raw_frames, quality, &infer, options.output_directory,
                               reference, record_reference, &detections, &mode_report, &error)) {
      std::cerr << "benchmark_bayer_input: " << error << std::endl;
      cleanup();
      return 1;
    }
    if (quality == 2U && mode_report.supported) {
      optimal_detections = detections;
    } else if (mode_report.supported) {
      mode_report.detection_difference = CompareDetections(optimal_detections, detections);
    }
    reports.push_back(std::move(mode_report));
  }
  cleanup();

  for (const ModeReport &mode_report : reports) {
    WriteModeReport(report_file, mode_report);
  }
  report_file.close();
  if (!report_file) {
    std::cerr << "benchmark_bayer_input: failed writing report" << std::endl;
    return 1;
  }
  std::cout << "benchmark_bayer_input report=" << report_path << " frames=" << raw_frames.size() << std::endl;
  return 0;
}
