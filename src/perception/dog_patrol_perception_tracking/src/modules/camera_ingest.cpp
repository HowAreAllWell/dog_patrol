#include "vision_demo_host/modules/camera_ingest.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#ifdef VISION_DEMO_HOST_ENABLE_HIK_MVS
#include <MvCameraControl.h>
#endif

namespace vision_demo_host {
namespace {

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

CameraIngest::CameraIngest() : impl_(std::make_unique<Impl>()) {}

CameraIngest::~CameraIngest() { Close(); }

void CameraIngest::Close() {
  if (capture_.isOpened()) {
    capture_.release();
  }
#ifdef VISION_DEMO_HOST_ENABLE_HIK_MVS
  impl_->ResetMvs();
#endif
}

bool CameraIngest::Open(const Config &config, std::string *error) {
  Close();
  config_ = config;

  if (config_.backend == Backend::kGstreamer) {
    if (config_.gstreamer_pipeline.empty()) {
      if (error != nullptr) {
        *error = "GStreamer pipeline is empty.";
      }
      return false;
    }

    if (!capture_.open(config_.gstreamer_pipeline, cv::CAP_GSTREAMER)) {
      if (error != nullptr) {
        *error = "Failed to open GStreamer source. Pipeline: " + config_.gstreamer_pipeline;
      }
      return false;
    }
    return true;
  }

#ifndef VISION_DEMO_HOST_ENABLE_HIK_MVS
  if (error != nullptr) {
    *error = "Hik MVS backend requested, but this build does not include the MVS SDK.";
  }
  return false;
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
      (void)MV_CC_SetIntValueEx(impl_->mvs_handle, "GevSCPSPacketSize", static_cast<unsigned int>(packet_size));
    }
  }

  (void)MV_CC_SetEnumValue(impl_->mvs_handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
  if (config_.width > 0) {
    (void)MV_CC_SetIntValueEx(impl_->mvs_handle, "Width", config_.width);
  }
  if (config_.height > 0) {
    (void)MV_CC_SetIntValueEx(impl_->mvs_handle, "Height", config_.height);
  }
  if (config_.fps > 0.0) {
    (void)MV_CC_SetBoolValue(impl_->mvs_handle, "AcquisitionFrameRateEnable", true);
    (void)MV_CC_SetFloatValue(impl_->mvs_handle, "AcquisitionFrameRate", static_cast<float>(config_.fps));
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

bool CameraIngest::Read(cv::Mat *frame, std::string *error) {
  if (frame == nullptr) {
    if (error != nullptr) {
      *error = "Output frame pointer is null.";
    }
    return false;
  }

  if (config_.backend == Backend::kGstreamer) {
    if (!capture_.isOpened()) {
      if (error != nullptr) {
        *error = "Camera source is not opened.";
      }
      return false;
    }

    if (!capture_.read(*frame) || frame->empty()) {
      if (error != nullptr) {
        *error = "Failed to read frame from GStreamer source.";
      }
      return false;
    }
    return true;
  }

#ifndef VISION_DEMO_HOST_ENABLE_HIK_MVS
  if (error != nullptr) {
    *error = "Hik MVS backend requested, but this build does not include the MVS SDK.";
  }
  return false;
#else
  if (impl_->mvs_handle == nullptr || !impl_->mvs_grabbing) {
    if (error != nullptr) {
      *error = "Hik MVS camera is not opened.";
    }
    return false;
  }

  MV_FRAME_OUT out_frame{};
  const unsigned int timeout_ms = static_cast<unsigned int>(std::max(1, config_.timeout_ms));
  const int get_ret = MV_CC_GetImageBuffer(impl_->mvs_handle, &out_frame, timeout_ms);
  if (get_ret != MV_OK || out_frame.pBufAddr == nullptr) {
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

    const int convert_ret = MV_CC_ConvertPixelTypeEx(impl_->mvs_handle, &convert_param);
    if (convert_ret != MV_OK) {
      if (error != nullptr) {
        *error = "MV_CC_ConvertPixelTypeEx failed: " + FormatSdkError(convert_ret);
      }
      break;
    }

    if (channel_count == 1) {
      cv::Mat mono(static_cast<int>(height), static_cast<int>(width), CV_8UC1, impl_->mvs_convert_buffer.data());
      cv::cvtColor(mono, *frame, cv::COLOR_GRAY2BGR);
    } else {
      cv::Mat bgr(static_cast<int>(height), static_cast<int>(width), CV_8UC3, impl_->mvs_convert_buffer.data());
      *frame = bgr.clone();
    }
    ok = !frame->empty();
    if (!ok && error != nullptr) {
      *error = "Converted MVS frame is empty.";
    }
  } while (false);

  const int free_ret = MV_CC_FreeImageBuffer(impl_->mvs_handle, &out_frame);
  if (free_ret != MV_OK && error != nullptr && ok) {
    *error = "MV_CC_FreeImageBuffer failed: " + FormatSdkError(free_ret);
    ok = false;
  }

  return ok;
#endif
}

}  // namespace vision_demo_host
