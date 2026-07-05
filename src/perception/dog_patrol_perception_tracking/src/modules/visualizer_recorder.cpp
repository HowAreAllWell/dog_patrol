#include "vision_demo_host/modules/visualizer_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iomanip>
#include <sstream>
#include <utility>

#include "vision_demo_host/modules/primary_recovery_debug.hpp"

namespace vision_demo_host {
namespace {

std::string EscapeGstString(const std::string &in) {
  std::string out;
  out.reserve(in.size());
  for (const char c : in) {
    if (c == ' ') {
      out += "\\ ";
    } else {
      out.push_back(c);
    }
  }
  return out;
}

std::string BuildNvencMp4WriterPipeline(const std::string &output_path, const int width, const int height,
                                        const double fps) {
  const int fps_i = std::max(1, static_cast<int>(std::lround(fps)));
  std::ostringstream oss;
  oss << "appsrc ! video/x-raw,format=BGR,width=" << width << ",height=" << height << ",framerate=" << fps_i
      << "/1 ! "
      << "videoconvert ! video/x-raw,format=BGRx ! "
      << "nvvidconv ! video/x-raw(memory:NVMM),format=NV12,width=" << width << ",height=" << height
      << ",framerate=" << fps_i << "/1 ! "
      << "nvv4l2h264enc insert-sps-pps=true iframeinterval=" << fps_i << " idrinterval=" << fps_i
      << " bitrate=12000000 preset-level=1 control-rate=1 ! "
      << "h264parse ! qtmux ! filesink location=" << EscapeGstString(output_path) << " sync=false";
  return oss.str();
}

const IdentityObservation *FindIdentityByRawTrack(const IdentityManagerResult &result, const int raw_track_id) {
  for (const auto &identity : result.identities) {
    if (identity.supporting_raw_track_id.has_value() && *identity.supporting_raw_track_id == raw_track_id) {
      return &identity;
    }
  }
  return nullptr;
}

}  // namespace

VisualizerRecorder::VisualizerRecorder(Config config) : config_(std::move(config)) {}

bool VisualizerRecorder::Initialize(const cv::Size &frame_size, std::string *error) {
  if (config_.enable_recording) {
    const std::string gst_writer = BuildNvencMp4WriterPipeline(config_.recording_path, frame_size.width,
                                                               frame_size.height, config_.recording_fps);
    if (!writer_.open(gst_writer, cv::CAP_GSTREAMER, 0, config_.recording_fps, frame_size, true)) {
      const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
      if (!writer_.open(config_.recording_path, fourcc, config_.recording_fps, frame_size, true)) {
        if (error != nullptr) {
          *error = "Failed to open recording writer (HW+fallback) at: " + config_.recording_path;
        }
        return false;
      }
    }
  }
  return true;
}

void VisualizerRecorder::Render(const cv::Mat &frame, const std::vector<Track> &tracks,
                                const PrimaryTargetResult &primary,
                                const IdentityManagerResult *identity_result,
                                const std::string &primary_decision_reason,
                                const std::string &primary_reject_reason) {
  if (!config_.enable_visualization && !config_.enable_recording) {
    return;
  }
  if (identity_result == nullptr) {
    return;
  }

  cv::Mat canvas = frame.clone();
  const int primary_semantic_id = primary.primary_target_id;

  for (const auto &track : tracks) {
    const auto *identity = FindIdentityByRawTrack(*identity_result, track.id);
    if (identity == nullptr || identity->semantic_id < 0) {
      continue;
    }
    const int semantic_id = identity->semantic_id;
    const bool is_primary = (primary_semantic_id > 0 && semantic_id == primary_semantic_id);
    const cv::Scalar color = is_primary ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
    cv::rectangle(canvas, track.bbox, color, 2);
    std::ostringstream label;
    label << "id=" << semantic_id << " " << IdentityStateToString(identity->state) << " raw=" << track.id;
    cv::putText(canvas, label.str(), CompactOverlayTrackLabelPoint(canvas.size(), track.bbox),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
  }

  const std::string primary_line = BuildPrimaryOverlayLine(
      primary, *identity_result, primary_decision_reason, primary_reject_reason);
  cv::putText(canvas, primary_line, cv::Point(20, 28), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(255, 255, 255), 2);

  if (config_.enable_visualization) {
    cv::imshow("vision_demo_host", canvas);
    cv::waitKey(1);
  }

  if (config_.enable_recording && writer_.isOpened()) {
    writer_.write(canvas);
  }
}

}  // namespace vision_demo_host
