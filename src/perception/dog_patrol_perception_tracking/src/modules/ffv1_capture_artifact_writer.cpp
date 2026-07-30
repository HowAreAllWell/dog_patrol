#include "vision_demo_host/modules/ffv1_capture_artifact_writer.hpp"

#include <opencv2/videoio.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace vision_demo_host {
namespace {

bool Fail(std::string *error, const std::string &message) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

std::string EscapeJson(const std::string &value) {
  std::ostringstream escaped;
  for (const char c : value) {
    switch (c) {
      case '"':
        escaped << "\\\"";
        break;
      case '\\':
        escaped << "\\\\";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        escaped << c;
        break;
    }
  }
  return escaped.str();
}

class Ffv1CaptureArtifactWriter final : public CaptureArtifactWriter {
 public:
  explicit Ffv1CaptureArtifactWriter(Ffv1CaptureArtifactWriterFactory::Config config)
      : config_(std::move(config)) {}

  bool Begin(const CaptureTakeDescriptor &descriptor,
             const CaptureFrameContract &frame_contract,
             std::string *error) override {
    if (started_) {
      return Fail(error, "FFV1 capture artifact writer has already started a take");
    }
    if (frame_contract.output_pixel_format != "BGR8") {
      return Fail(error, "FFV1 capture artifact writer requires BGR8 input frames");
    }
    if (frame_contract.width <= 0 || frame_contract.height <= 0) {
      return Fail(error, "FFV1 capture frame contract has invalid dimensions");
    }
    if (!std::isfinite(config_.requested_fps) || config_.requested_fps <= 0.0) {
      return Fail(error, "FFV1 capture requested fps must be finite and positive");
    }

    descriptor_ = descriptor;
    frame_contract_ = frame_contract;
    take_directory_ = config_.session_directory / descriptor.name;
    std::error_code filesystem_error;
    std::filesystem::create_directories(take_directory_, filesystem_error);
    if (filesystem_error) {
      return Fail(error, "Failed to create FFV1 take directory: " + filesystem_error.message());
    }
    frame_timestamps_path_ = take_directory_ / "frame_timestamps.csv";
    frame_timestamps_.open(frame_timestamps_path_);
    if (!frame_timestamps_) {
      return Fail(error, "Failed to open FFV1 frame timestamp artifact: " +
                             frame_timestamps_path_.string());
    }
    frame_timestamps_
        << "capture_index,source_timestamp_ns,sdk_host_timestamp,camera_frame_number,"
           "camera_frame_number_available,device_timestamp_ticks,source_pixel_type,"
           "source_pixel_type_name,width,height,source_payload_bytes,camera_lost_packets\n";

    video_path_ = take_directory_ / "video.mkv";
    const int fourcc = cv::VideoWriter::fourcc('F', 'F', 'V', '1');
    const cv::Size frame_size(frame_contract.width, frame_contract.height);
    if (!video_writer_.open(video_path_.string(), cv::CAP_FFMPEG, fourcc,
                            config_.requested_fps, frame_size, true) &&
        !video_writer_.open(video_path_.string(), fourcc, config_.requested_fps,
                            frame_size, true)) {
      frame_timestamps_.close();
      return Fail(error, "Failed to open mandatory FFV1/MKV writer: " + video_path_.string() +
                             ". Check that OpenCV was built with FFmpeg and FFV1 support.");
    }
    started_ = true;
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Write(const CaptureFrame &frame, std::string *error) override {
    if (!started_ || finished_) {
      return Fail(error, "FFV1 capture artifact writer is not accepting frames");
    }
    const cv::Mat &bgr8 = frame.source.bgr8;
    if (bgr8.empty() || bgr8.type() != CV_8UC3 || bgr8.cols != frame_contract_.width ||
        bgr8.rows != frame_contract_.height) {
      return Fail(error, "FFV1 capture frame violates the BGR8 frame contract");
    }
    try {
      video_writer_.write(bgr8);
    } catch (const cv::Exception &exception) {
      return Fail(error, "FFV1 frame write failed: " + std::string(exception.what()));
    }
    frame_timestamps_ << frame.capture_index << ',' << frame.source.source_timestamp_ns << ','
                      << frame.source.sdk_host_timestamp << ','
                      << frame.source.camera_frame_number << ','
                      << (frame.source.camera_frame_number_available ? "true" : "false") << ','
                      << frame.source.device_timestamp_ticks << ','
                      << frame.source.source_pixel_type << ','
                      << '"' << EscapeJson(frame.source.source_pixel_type_name) << '"' << ','
                      << frame.source.width << ',' << frame.source.height << ','
                      << frame.source.source_payload_bytes << ','
                      << frame.source.camera_lost_packets << '\n';
    if (!frame_timestamps_) {
      return Fail(error, "Failed to write FFV1 frame timestamp artifact");
    }
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Finish(const CaptureTakeSummary &summary, std::string *error) override {
    if (!started_ || finished_) {
      return Fail(error, "FFV1 capture artifact writer has no active take to finalize");
    }
    video_writer_.release();
    frame_timestamps_.close();
    const std::filesystem::path markers_path = take_directory_ / "markers.csv";
    std::ofstream markers(markers_path);
    if (!markers) {
      return Fail(error, "Failed to open FFV1 marker artifact: " + markers_path.string());
    }
    markers << "marker_index,wall_time_unix_ns,elapsed_seconds,note\n";
    for (const CaptureMarker &marker : summary.markers) {
      markers << marker.index << ',' << marker.wall_time_ns << ',' << std::fixed
              << std::setprecision(9) << marker.elapsed_seconds << ',' << '"'
              << EscapeJson(marker.note) << '"' << '\n';
    }
    markers.close();
    if (!markers) {
      return Fail(error, "Failed to write FFV1 marker artifact");
    }

    const std::filesystem::path metadata_path = take_directory_ / "metadata.json";
    std::ofstream metadata(metadata_path);
    if (!metadata) {
      return Fail(error, "Failed to open FFV1 metadata artifact: " + metadata_path.string());
    }
    metadata << "{\n"
             << "  \"take_name\": \"" << EscapeJson(summary.descriptor.name) << "\",\n"
             << "  \"take_sequence\": " << summary.descriptor.sequence << ",\n"
             << "  \"state\": \"" << (summary.complete ? "complete" : "incomplete")
             << "\",\n"
             << "  \"codec\": \"FFV1\",\n"
             << "  \"container\": \"MKV\",\n"
             << "  \"video_path\": \"" << EscapeJson(video_path_.string()) << "\",\n"
             << "  \"started_wall_time_ns\": " << summary.descriptor.started_wall_time_ns
             << ",\n"
             << "  \"finished_wall_time_ns\": " << summary.finished_wall_time_ns << ",\n"
             << "  \"camera\": {\n"
             << "    \"backend\": \"hik_mvs\",\n"
             << "    \"model\": \"" << EscapeJson(config_.mvs_model) << "\",\n"
             << "    \"serial\": \"" << EscapeJson(config_.mvs_serial) << "\",\n"
             << "    \"requested_width\": " << config_.requested_width << ",\n"
             << "    \"requested_height\": " << config_.requested_height << ",\n"
             << "    \"requested_fps\": " << config_.requested_fps << ",\n"
             << "    \"timeout_ms\": " << config_.timeout_ms << "\n"
             << "  },\n"
             << "  \"frame_contract\": {\n"
             << "    \"output_pixel_format\": \""
             << EscapeJson(summary.frame_contract.output_pixel_format) << "\",\n"
             << "    \"source_pixel_type\": " << summary.frame_contract.source_pixel_type
             << ",\n"
             << "    \"source_pixel_type_name\": \""
             << EscapeJson(summary.frame_contract.source_pixel_type_name) << "\",\n"
             << "    \"width\": " << summary.frame_contract.width << ",\n"
             << "    \"height\": " << summary.frame_contract.height << ",\n"
             << "    \"source_payload_bytes\": "
             << summary.frame_contract.source_payload_bytes << ",\n"
             << "    \"bayer_interpolation\": \""
             << EscapeJson(summary.frame_contract.bayer_interpolation) << "\",\n"
             << "    \"bayer_smoothing\": "
             << (summary.frame_contract.bayer_smoothing ? "true" : "false") << "\n"
             << "  },\n"
             << "  \"counts\": {\n"
             << "    \"captured_frames\": " << summary.captured_frames << ",\n"
             << "    \"written_frames\": " << summary.written_frames << ",\n"
             << "    \"dropped_frames\": " << summary.dropped_frames << ",\n"
             << "    \"write_errors\": " << summary.write_errors << ",\n"
             << "    \"camera_frame_gaps\": " << summary.camera_frame_gaps << "\n"
             << "  },\n"
             << "  \"markers_path\": \"" << EscapeJson(markers_path.string()) << "\",\n"
             << "  \"frame_timestamps_path\": \""
             << EscapeJson(frame_timestamps_path_.string()) << "\"\n"
             << "}\n";
    metadata.close();
    if (!metadata) {
      return Fail(error, "Failed to write FFV1 metadata artifact");
    }
    finished_ = true;
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

 private:
  Ffv1CaptureArtifactWriterFactory::Config config_;
  CaptureTakeDescriptor descriptor_;
  CaptureFrameContract frame_contract_;
  std::filesystem::path take_directory_;
  std::filesystem::path video_path_;
  std::filesystem::path frame_timestamps_path_;
  cv::VideoWriter video_writer_;
  std::ofstream frame_timestamps_;
  bool started_{false};
  bool finished_{false};
};

}  // namespace

Ffv1CaptureArtifactWriterFactory::Ffv1CaptureArtifactWriterFactory(Config config)
    : config_(std::move(config)) {}

std::unique_ptr<CaptureArtifactWriter> Ffv1CaptureArtifactWriterFactory::Create() {
  return std::make_unique<Ffv1CaptureArtifactWriter>(config_);
}

}  // namespace vision_demo_host
