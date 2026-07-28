#include "vision_demo_host/modules/camera_ingest.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#ifdef VISION_DEMO_HOST_ENABLE_HIK_MVS
#include <MvCameraControl.h>
#endif

namespace vision_demo_host {
namespace {

using SteadyClock = std::chrono::steady_clock;

double ElapsedMilliseconds(const SteadyClock::time_point start) {
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}

bool Fail(std::string *error, const std::string &message) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

#ifdef VISION_DEMO_HOST_ENABLE_HIK_MVS

constexpr unsigned int kSupportedTransportLayers =
    MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE |
    MV_GENTL_XOF_DEVICE;

std::string DecodeCharArray(const char *src, const std::size_t size) {
  if (src == nullptr) {
    return "";
  }
  std::size_t len = 0;
  while (len < size && src[len] != '\0') {
    ++len;
  }
  return std::string(src, len);
}

std::string FormatSdkError(const int code) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "0x%x", code);
  return buf;
}

bool RequireSdkSuccess(const int code, const char *operation, std::string *error) {
  if (code == MV_OK) {
    return true;
  }
  return Fail(error, std::string(operation) + " failed: " + FormatSdkError(code));
}

std::string DeviceModelName(const MV_CC_DEVICE_INFO &device) {
  switch (device.nTLayerType) {
    case MV_USB_DEVICE:
      return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stUsb3VInfo.chModelName),
                             sizeof(device.SpecialInfo.stUsb3VInfo.chModelName));
    case MV_GIGE_DEVICE:
    case MV_GENTL_GIGE_DEVICE:
      return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stGigEInfo.chModelName),
                             sizeof(device.SpecialInfo.stGigEInfo.chModelName));
    case MV_GENTL_CAMERALINK_DEVICE:
      return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stCMLInfo.chModelName),
                             sizeof(device.SpecialInfo.stCMLInfo.chModelName));
    case MV_GENTL_CXP_DEVICE:
      return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stCXPInfo.chModelName),
                             sizeof(device.SpecialInfo.stCXPInfo.chModelName));
    case MV_GENTL_XOF_DEVICE:
      return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stXoFInfo.chModelName),
                             sizeof(device.SpecialInfo.stXoFInfo.chModelName));
    default:
      return "";
  }
}

std::string DeviceSerialNumber(const MV_CC_DEVICE_INFO &device) {
  switch (device.nTLayerType) {
    case MV_USB_DEVICE:
      return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stUsb3VInfo.chSerialNumber),
                             sizeof(device.SpecialInfo.stUsb3VInfo.chSerialNumber));
    case MV_GIGE_DEVICE:
    case MV_GENTL_GIGE_DEVICE:
      return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stGigEInfo.chSerialNumber),
                             sizeof(device.SpecialInfo.stGigEInfo.chSerialNumber));
    case MV_GENTL_CAMERALINK_DEVICE:
      return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stCMLInfo.chSerialNumber),
                             sizeof(device.SpecialInfo.stCMLInfo.chSerialNumber));
    case MV_GENTL_CXP_DEVICE:
      return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stCXPInfo.chSerialNumber),
                             sizeof(device.SpecialInfo.stCXPInfo.chSerialNumber));
    case MV_GENTL_XOF_DEVICE:
      return DecodeCharArray(reinterpret_cast<const char *>(device.SpecialInfo.stXoFInfo.chSerialNumber),
                             sizeof(device.SpecialInfo.stXoFInfo.chSerialNumber));
    default:
      return "";
  }
}

#endif

}  // namespace

struct CameraIngest::Impl {
  StageTiming acquisition_timing;
  StageTiming conversion_timing;
  StageTiming copy_timing;
  FrameContinuity continuity;
  std::uint64_t acquired_frames{0};
  std::uint64_t acquisition_failures{0};
  std::uint64_t camera_lost_packets{0};

  void ResetMetrics() {
    acquisition_timing.Clear();
    conversion_timing.Clear();
    copy_timing.Clear();
    continuity.Reset();
    acquired_frames = 0;
    acquisition_failures = 0;
    camera_lost_packets = 0;
  }

#ifdef VISION_DEMO_HOST_ENABLE_HIK_MVS
  void *mvs_handle{nullptr};
  bool mvs_sdk_initialized{false};
  bool mvs_grabbing{false};
  std::vector<unsigned char> mvs_convert_buffer;

  void ResetMvs() {
    if (mvs_handle != nullptr && mvs_grabbing) {
      MV_CC_StopGrabbing(mvs_handle);
      mvs_grabbing = false;
    }
    if (mvs_handle != nullptr) {
      MV_CC_CloseDevice(mvs_handle);
      MV_CC_DestroyHandle(mvs_handle);
      mvs_handle = nullptr;
    }
    if (mvs_sdk_initialized) {
      MV_CC_Finalize();
      mvs_sdk_initialized = false;
    }
    mvs_convert_buffer.clear();
  }
#endif
};

void CameraIngest::StageTiming::ObserveMilliseconds(const double milliseconds) {
  if (!std::isfinite(milliseconds) || milliseconds < 0.0) {
    return;
  }
  if (samples_.size() == kMaxSamples) {
    samples_.erase(samples_.begin());
  }
  samples_.push_back(milliseconds);
}

CameraIngest::PercentileSummary CameraIngest::StageTiming::Summary() const {
  PercentileSummary result;
  result.samples = samples_.size();
  if (samples_.empty()) {
    return result;
  }

  std::vector<double> sorted = samples_;
  std::sort(sorted.begin(), sorted.end());
  const auto percentile = [&sorted](const double fraction) {
    const auto rank = static_cast<std::size_t>(
        std::ceil(fraction * static_cast<double>(sorted.size())));
    return sorted[std::max<std::size_t>(1, rank) - 1];
  };
  result.p50_ms = percentile(0.50);
  result.p95_ms = percentile(0.95);
  result.p99_ms = percentile(0.99);
  return result;
}

void CameraIngest::StageTiming::Clear() { samples_.clear(); }

std::uint64_t CameraIngest::FrameContinuity::Observe(const std::uint32_t frame_number) {
  if (!has_previous_) {
    has_previous_ = true;
    previous_ = frame_number;
    return 0;
  }

  const std::uint32_t expected = previous_ + 1U;
  previous_ = frame_number;
  if (frame_number == expected) {
    return 0;
  }

  ++non_contiguous_frames_;
  if (frame_number > expected) {
    const auto dropped = static_cast<std::uint64_t>(frame_number - expected);
    dropped_frames_ += dropped;
    return dropped;
  }
  return 0;
}

std::uint64_t CameraIngest::FrameContinuity::DroppedFrames() const {
  return dropped_frames_;
}

std::uint64_t CameraIngest::FrameContinuity::NonContiguousFrames() const {
  return non_contiguous_frames_;
}

void CameraIngest::FrameContinuity::Reset() {
  has_previous_ = false;
  previous_ = 0;
  dropped_frames_ = 0;
  non_contiguous_frames_ = 0;
}

CameraIngest::CameraIngest() : impl_(std::make_unique<Impl>()) {}

CameraIngest::~CameraIngest() { Close(); }

void CameraIngest::Close() {
#ifdef VISION_DEMO_HOST_ENABLE_HIK_MVS
  impl_->ResetMvs();
#endif
}

bool CameraIngest::ValidateConfig(const Config &config, std::string *error) {
  if (error != nullptr) {
    error->clear();
  }
  if (config.width <= 0 || config.height <= 0) {
    return Fail(error, "camera width and height must be positive");
  }
  if (!std::isfinite(config.fps) || config.fps <= 0.0) {
    return Fail(error, "camera fps must be finite and positive");
  }
  if (config.timeout_ms <= 0) {
    return Fail(error, "camera timeout_ms must be positive");
  }
  switch (config.bayer_interpolation) {
    case BayerInterpolation::kFast:
    case BayerInterpolation::kBalanced:
    case BayerInterpolation::kOptimal:
    case BayerInterpolation::kOptimalPlus:
      return true;
  }
  return Fail(error, "camera bayer_interpolation is unsupported");
}

bool CameraIngest::ParseBayerInterpolation(const std::string &value,
                                           BayerInterpolation *interpolation,
                                           std::string *error) {
  if (interpolation == nullptr) {
    return Fail(error, "Bayer interpolation output pointer is null");
  }
  if (value == "fast") {
    *interpolation = BayerInterpolation::kFast;
  } else if (value == "balanced") {
    *interpolation = BayerInterpolation::kBalanced;
  } else if (value == "optimal") {
    *interpolation = BayerInterpolation::kOptimal;
  } else if (value == "optimal_plus") {
    *interpolation = BayerInterpolation::kOptimalPlus;
  } else {
    return Fail(error, "Unsupported Bayer interpolation mode: " + value);
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

std::string CameraIngest::BayerInterpolationName(const BayerInterpolation interpolation) {
  switch (interpolation) {
    case BayerInterpolation::kFast:
      return "fast";
    case BayerInterpolation::kBalanced:
      return "balanced";
    case BayerInterpolation::kOptimal:
      return "optimal";
    case BayerInterpolation::kOptimalPlus:
      return "optimal_plus";
  }
  return "unknown";
}

std::string CameraIngest::PixelTypeName(const std::uint32_t pixel_type) {
  switch (pixel_type) {
    case 0x01080001U:
      return "Mono8";
    case 0x01100003U:
      return "Mono10";
    case 0x010C0004U:
      return "Mono10_Packed";
    case 0x01100005U:
      return "Mono12";
    case 0x010C0006U:
      return "Mono12_Packed";
    case 0x01100007U:
      return "Mono16";
    case 0x01080008U:
      return "BayerGR8";
    case 0x01080009U:
      return "BayerRG8";
    case 0x0108000AU:
      return "BayerGB8";
    case 0x0108000BU:
      return "BayerBG8";
    case 0x0110000CU:
      return "BayerGR10";
    case 0x0110000DU:
      return "BayerRG10";
    case 0x0110000EU:
      return "BayerGB10";
    case 0x0110000FU:
      return "BayerBG10";
    case 0x01100010U:
      return "BayerGR12";
    case 0x01100011U:
      return "BayerRG12";
    case 0x01100012U:
      return "BayerGB12";
    case 0x01100013U:
      return "BayerBG12";
    case 0x010C0026U:
      return "BayerGR10_Packed";
    case 0x010C0027U:
      return "BayerRG10_Packed";
    case 0x010C0028U:
      return "BayerGB10_Packed";
    case 0x010C0029U:
      return "BayerBG10_Packed";
    case 0x010C002AU:
      return "BayerGR12_Packed";
    case 0x010C002BU:
      return "BayerRG12_Packed";
    case 0x010C002CU:
      return "BayerGB12_Packed";
    case 0x010C002DU:
      return "BayerBG12_Packed";
    case 0x02180014U:
      return "RGB8_Packed";
    case 0x02180015U:
      return "BGR8_Packed";
    default:
      std::ostringstream value;
      value << "Unknown(0x" << std::hex << std::nouppercase << pixel_type << ")";
      return value.str();
  }
}

void CameraIngest::ApplySourceFrameMetadata(const SourceFrameMetadata &metadata,
                                            AcquiredFrame *frame) {
  if (frame == nullptr) {
    return;
  }
  frame->source_timestamp_ns = metadata.source_timestamp_ns;
  frame->sdk_host_timestamp = metadata.sdk_host_timestamp;
  frame->camera_frame_number = metadata.camera_frame_number;
  frame->camera_frame_number_available = true;
  frame->device_timestamp_ticks = metadata.device_timestamp_ticks;
  frame->source_pixel_type = metadata.source_pixel_type;
  frame->source_pixel_type_name = PixelTypeName(metadata.source_pixel_type);
  frame->width = metadata.width;
  frame->height = metadata.height;
  frame->source_payload_bytes = metadata.source_payload_bytes;
  frame->camera_lost_packets = metadata.camera_lost_packets;
}

bool CameraIngest::Open(const Config &config, std::string *error) {
  Close();
  if (!ValidateConfig(config, error)) {
    return false;
  }
  config_ = config;
  impl_->ResetMetrics();

#ifndef VISION_DEMO_HOST_ENABLE_HIK_MVS
  return Fail(error, "This build does not include the Hik MVS SDK.");
#else
  const int init_ret = MV_CC_Initialize();
  if (init_ret != MV_OK) {
    if (error != nullptr) {
      *error = "MV_CC_Initialize failed: " + FormatSdkError(init_ret);
    }
    return false;
  }
  impl_->mvs_sdk_initialized = true;

  MV_CC_DEVICE_INFO_LIST device_list{};
  const int enum_ret = MV_CC_EnumDevices(kSupportedTransportLayers, &device_list);
  if (enum_ret != MV_OK) {
    if (error != nullptr) {
      *error = "MV_CC_EnumDevices failed: " + FormatSdkError(enum_ret);
    }
    Close();
    return false;
  }
  if (device_list.nDeviceNum == 0) {
    if (error != nullptr) {
      *error = "No Hik-compatible cameras were discovered by the MVS SDK.";
    }
    Close();
    return false;
  }

  MV_CC_DEVICE_INFO *selected_device = nullptr;
  std::string available_devices;
  for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
    MV_CC_DEVICE_INFO *device = device_list.pDeviceInfo[i];
    if (device == nullptr) {
      continue;
    }

    const std::string model = DeviceModelName(*device);
    const std::string serial = DeviceSerialNumber(*device);
    if (!available_devices.empty()) {
      available_devices += ", ";
    }
    available_devices += "[" + std::to_string(i) + "] " + model + "(" + serial + ")";

    const bool model_match = config_.hik_mvs_model.empty() || model == config_.hik_mvs_model;
    const bool serial_match = config_.hik_mvs_serial.empty() || serial == config_.hik_mvs_serial;
    if (selected_device == nullptr && model_match && serial_match) {
      selected_device = device;
    }
  }

  if (selected_device == nullptr) {
    if (error != nullptr) {
      *error = "No Hik camera matched model/serial filter. available=" + available_devices;
    }
    Close();
    return false;
  }

  const int create_ret = MV_CC_CreateHandle(&impl_->mvs_handle, selected_device);
  if (create_ret != MV_OK) {
    if (error != nullptr) {
      *error = "MV_CC_CreateHandle failed: " + FormatSdkError(create_ret);
    }
    Close();
    return false;
  }

  const int open_ret = MV_CC_OpenDevice(impl_->mvs_handle, MV_ACCESS_Exclusive, 0);
  if (open_ret != MV_OK) {
    if (error != nullptr) {
      *error = "MV_CC_OpenDevice failed: " + FormatSdkError(open_ret);
    }
    Close();
    return false;
  }

  if (selected_device->nTLayerType == MV_GIGE_DEVICE || selected_device->nTLayerType == MV_GENTL_GIGE_DEVICE) {
    const int packet_size = MV_CC_GetOptimalPacketSize(impl_->mvs_handle);
    if (packet_size > 0) {
      if (!RequireSdkSuccess(
              MV_CC_SetIntValueEx(impl_->mvs_handle, "GevSCPSPacketSize", packet_size),
              "MV_CC_SetIntValueEx(GevSCPSPacketSize)", error)) {
        Close();
        return false;
      }
    }
  }

  if (!RequireSdkSuccess(
          MV_CC_SetEnumValue(impl_->mvs_handle, "TriggerMode", MV_TRIGGER_MODE_OFF),
          "MV_CC_SetEnumValue(TriggerMode)", error) ||
      !RequireSdkSuccess(
          MV_CC_SetIntValueEx(impl_->mvs_handle, "Width", config_.width),
          "MV_CC_SetIntValueEx(Width)", error) ||
      !RequireSdkSuccess(
          MV_CC_SetIntValueEx(impl_->mvs_handle, "Height", config_.height),
          "MV_CC_SetIntValueEx(Height)", error) ||
      !RequireSdkSuccess(
          MV_CC_SetBoolValue(impl_->mvs_handle, "AcquisitionFrameRateEnable", true),
          "MV_CC_SetBoolValue(AcquisitionFrameRateEnable)", error) ||
      !RequireSdkSuccess(
          MV_CC_SetFloatValue(
              impl_->mvs_handle, "AcquisitionFrameRate", static_cast<float>(config_.fps)),
          "MV_CC_SetFloatValue(AcquisitionFrameRate)", error) ||
      !RequireSdkSuccess(
          MV_CC_SetBayerCvtQuality(
              impl_->mvs_handle, static_cast<unsigned int>(config_.bayer_interpolation)),
          "MV_CC_SetBayerCvtQuality", error) ||
      !RequireSdkSuccess(
          MV_CC_SetBayerFilterEnable(impl_->mvs_handle, config_.bayer_smoothing),
          "MV_CC_SetBayerFilterEnable", error)) {
    Close();
    return false;
  }

  const int start_ret = MV_CC_StartGrabbing(impl_->mvs_handle);
  if (start_ret != MV_OK) {
    if (error != nullptr) {
      *error = "MV_CC_StartGrabbing failed: " + FormatSdkError(start_ret);
    }
    Close();
    return false;
  }

  impl_->mvs_grabbing = true;
  return true;
#endif
}

bool CameraIngest::Read(AcquiredFrame *frame, std::string *error) {
  if (frame == nullptr) {
    return Fail(error, "Output acquired-frame pointer is null.");
  }
  *frame = AcquiredFrame{};

#ifndef VISION_DEMO_HOST_ENABLE_HIK_MVS
  return Fail(error, "This build does not include the Hik MVS SDK.");
#else
  if (impl_->mvs_handle == nullptr || !impl_->mvs_grabbing) {
    if (error != nullptr) {
      *error = "Hik MVS camera is not opened.";
    }
    return false;
  }

  MV_FRAME_OUT out_frame{};
  const unsigned int timeout_ms = static_cast<unsigned int>(std::max(1, config_.timeout_ms));
  const auto acquisition_start = SteadyClock::now();
  const int get_ret = MV_CC_GetImageBuffer(impl_->mvs_handle, &out_frame, timeout_ms);
  frame->acquisition_ms = ElapsedMilliseconds(acquisition_start);
  if (get_ret != MV_OK || out_frame.pBufAddr == nullptr) {
    ++impl_->acquisition_failures;
    if (error != nullptr) {
      *error = "MV_CC_GetImageBuffer failed: " + FormatSdkError(get_ret);
    }
    return false;
  }

  bool ok = false;
  do {
    const unsigned int width = out_frame.stFrameInfo.nWidth;
    const unsigned int height = out_frame.stFrameInfo.nHeight;
    const auto src_pixel_type = out_frame.stFrameInfo.enPixelType;

    SourceFrameMetadata metadata;
    metadata.source_timestamp_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    metadata.sdk_host_timestamp = out_frame.stFrameInfo.nHostTimeStamp;
    metadata.camera_frame_number = out_frame.stFrameInfo.nFrameNum;
    metadata.device_timestamp_ticks =
        (static_cast<std::uint64_t>(out_frame.stFrameInfo.nDevTimeStampHigh) << 32U) |
        out_frame.stFrameInfo.nDevTimeStampLow;
    metadata.source_pixel_type = static_cast<std::uint32_t>(src_pixel_type);
    metadata.width = static_cast<int>(width);
    metadata.height = static_cast<int>(height);
    metadata.source_payload_bytes =
        out_frame.stFrameInfo.nFrameLenEx > 0 ? out_frame.stFrameInfo.nFrameLenEx
                                             : out_frame.stFrameInfo.nFrameLen;
    metadata.camera_lost_packets = out_frame.stFrameInfo.nLostPacket;
    ApplySourceFrameMetadata(metadata, frame);

    MV_CC_PIXEL_CONVERT_PARAM_EX convert_param{};
    convert_param.nWidth = width;
    convert_param.nHeight = height;
    convert_param.pSrcData = out_frame.pBufAddr;
    convert_param.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
    convert_param.enSrcPixelType = src_pixel_type;

    unsigned int channel_count = 3;
    MvGvspPixelType dst_pixel_type = PixelType_Gvsp_BGR8_Packed;
    if (src_pixel_type == PixelType_Gvsp_Mono8) {
      channel_count = 1;
      dst_pixel_type = PixelType_Gvsp_Mono8;
    }

    const std::size_t dst_size =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * channel_count;
    impl_->mvs_convert_buffer.resize(dst_size);
    convert_param.enDstPixelType = dst_pixel_type;
    convert_param.pDstBuffer = impl_->mvs_convert_buffer.data();
    convert_param.nDstBufferSize = static_cast<unsigned int>(impl_->mvs_convert_buffer.size());

    const auto conversion_start = SteadyClock::now();
    const int convert_ret = MV_CC_ConvertPixelTypeEx(impl_->mvs_handle, &convert_param);
    if (convert_ret != MV_OK) {
      if (error != nullptr) {
        *error = "MV_CC_ConvertPixelTypeEx failed: " + FormatSdkError(convert_ret);
      }
      break;
    }

    if (channel_count == 1) {
      cv::Mat mono(static_cast<int>(height), static_cast<int>(width), CV_8UC1, impl_->mvs_convert_buffer.data());
      cv::cvtColor(mono, frame->bgr8, cv::COLOR_GRAY2BGR);
    } else {
      frame->conversion_ms = ElapsedMilliseconds(conversion_start);
      cv::Mat bgr(static_cast<int>(height), static_cast<int>(width), CV_8UC3, impl_->mvs_convert_buffer.data());
      const auto copy_start = SteadyClock::now();
      frame->bgr8 = bgr.clone();
      frame->copy_ms = ElapsedMilliseconds(copy_start);
    }
    if (channel_count == 1) {
      frame->conversion_ms = ElapsedMilliseconds(conversion_start);
    }
    ok = !frame->bgr8.empty() && frame->bgr8.type() == CV_8UC3;
    if (!ok && error != nullptr) {
      *error = "Converted MVS frame is not a non-empty BGR8 image.";
    }
  } while (false);

  const int free_ret = MV_CC_FreeImageBuffer(impl_->mvs_handle, &out_frame);
  if (free_ret != MV_OK && error != nullptr && ok) {
    *error = "MV_CC_FreeImageBuffer failed: " + FormatSdkError(free_ret);
    ok = false;
  }

  if (ok) {
    ++impl_->acquired_frames;
    impl_->camera_lost_packets += frame->camera_lost_packets;
    impl_->continuity.Observe(frame->camera_frame_number);
    impl_->acquisition_timing.ObserveMilliseconds(frame->acquisition_ms);
    impl_->conversion_timing.ObserveMilliseconds(frame->conversion_ms);
    impl_->copy_timing.ObserveMilliseconds(frame->copy_ms);
  }
  return ok;
#endif
}

CameraIngest::MetricsSnapshot CameraIngest::Metrics() const {
  MetricsSnapshot result;
  result.acquired_frames = impl_->acquired_frames;
  result.acquisition_failures = impl_->acquisition_failures;
  result.dropped_frames = impl_->continuity.DroppedFrames();
  result.non_contiguous_frames = impl_->continuity.NonContiguousFrames();
  result.camera_lost_packets = impl_->camera_lost_packets;
  result.acquisition = impl_->acquisition_timing.Summary();
  result.conversion = impl_->conversion_timing.Summary();
  result.copy = impl_->copy_timing.Summary();
  return result;
}

}  // namespace vision_demo_host
