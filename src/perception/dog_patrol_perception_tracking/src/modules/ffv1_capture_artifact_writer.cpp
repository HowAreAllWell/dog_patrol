#include "vision_demo_host/modules/ffv1_capture_artifact_writer.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/rational.h>
}

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

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

std::string FfmpegError(const int code) {
  char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(code, buffer, sizeof(buffer));
  return buffer;
}

class Ffv1CaptureArtifactWriter final : public CaptureArtifactWriter {
 public:
  explicit Ffv1CaptureArtifactWriter(Ffv1CaptureArtifactWriterFactory::Config config)
      : config_(std::move(config)) {}

  ~Ffv1CaptureArtifactWriter() override { CloseVideo(); }

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
    if (!PrepareTakeDirectory(descriptor, error)) {
      return false;
    }
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
    if (!OpenVideo(error)) {
      frame_timestamps_.close();
      CloseVideo();
      return false;
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

    int result = av_frame_make_writable(video_frame_);
    if (result < 0) {
      return Fail(error, "FFV1 frame buffer is not writable: " + FfmpegError(result));
    }
    for (int row = 0; row < bgr8.rows; ++row) {
      const std::uint8_t *source = bgr8.ptr<std::uint8_t>(row);
      std::uint8_t *destination = video_frame_->data[0] + row * video_frame_->linesize[0];
      for (int column = 0; column < bgr8.cols; ++column) {
        const int source_offset = column * 3;
        const int destination_offset = column * 4;
        destination[destination_offset] = source[source_offset];
        destination[destination_offset + 1] = source[source_offset + 1];
        destination[destination_offset + 2] = source[source_offset + 2];
        destination[destination_offset + 3] = 255U;
      }
    }
    video_frame_->pts = next_frame_pts_++;
    result = avcodec_send_frame(codec_context_, video_frame_);
    if (result < 0) {
      return Fail(error, "FFV1 frame encode submission failed: " + FfmpegError(result));
    }
    if (!WriteAvailablePackets(error)) {
      return false;
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
    if (finished_) {
      return Fail(error, "FFV1 capture artifact writer has already finalized its take");
    }
    descriptor_ = summary.descriptor;
    frame_contract_ = summary.frame_contract;
    if (!PrepareTakeDirectory(summary.descriptor, error)) {
      return false;
    }

    std::string video_finalize_error;
    if (started_) {
      if (!FinalizeVideo(&video_finalize_error)) {
        // The sidecar artifacts still describe a failed take even when FFmpeg cannot finalize.
      }
      frame_timestamps_.close();
      if (!frame_timestamps_ && video_finalize_error.empty()) {
        video_finalize_error = "Failed to finalize FFV1 frame timestamp artifact";
      }
      CloseVideo();
    } else if (!EnsureFrameTimestampArtifact(error)) {
      return false;
    }

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
    const std::uint64_t elapsed_ns =
        summary.finished_wall_time_ns >= summary.descriptor.started_wall_time_ns
            ? summary.finished_wall_time_ns - summary.descriptor.started_wall_time_ns
            : 0U;
    const double elapsed_seconds = static_cast<double>(elapsed_ns) / 1'000'000'000.0;
    const double captured_fps = elapsed_seconds > 0.0
                                    ? static_cast<double>(summary.captured_frames) / elapsed_seconds
                                    : 0.0;
    const double written_fps = elapsed_seconds > 0.0
                                   ? static_cast<double>(summary.written_frames) / elapsed_seconds
                                   : 0.0;
    const bool artifact_complete = summary.complete && video_finalize_error.empty();
    const std::string artifact_last_write_error =
        summary.last_write_error.empty() ? video_finalize_error : summary.last_write_error;
    metadata << "{\n"
             << "  \"take_name\": \"" << EscapeJson(summary.descriptor.name) << "\",\n"
             << "  \"take_sequence\": " << summary.descriptor.sequence << ",\n"
             << "  \"state\": \"" << (artifact_complete ? "complete" : "incomplete")
             << "\",\n"
             << "  \"codec\": \"FFV1\",\n"
             << "  \"container\": \"MKV\",\n"
             << "  \"encoder\": {\n"
             << "    \"backend\": \"native_ffmpeg\",\n"
             << "    \"thread_type\": \"slice\",\n"
             << "    \"thread_count\": " << encoder_thread_count_ << ",\n"
             << "    \"slice_count\": " << encoder_slice_count_ << ",\n"
             << "    \"pixel_format\": \"bgr0\"\n"
             << "  },\n"
             << "  \"writer_opened\": " << (summary.writer_opened ? "true" : "false")
             << ",\n"
             << "  \"last_write_error\": \"" << EscapeJson(artifact_last_write_error)
             << "\",\n"
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
             << "  \"timing\": {\n"
             << "    \"nominal_stream_fps\": " << config_.requested_fps << ",\n"
             << "    \"stream_fps_is_nominal\": true,\n"
             << "    \"take_elapsed_seconds\": " << std::fixed << std::setprecision(6)
             << elapsed_seconds << ",\n"
             << "    \"captured_fps\": " << captured_fps << ",\n"
             << "    \"written_fps\": " << written_fps << "\n"
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
    if (!video_finalize_error.empty()) {
      return Fail(error, video_finalize_error);
    }
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

 private:
  bool PrepareTakeDirectory(const CaptureTakeDescriptor &descriptor, std::string *error) {
    if (!take_directory_.empty()) {
      return true;
    }
    take_directory_ = config_.session_directory / descriptor.name;
    std::error_code filesystem_error;
    std::filesystem::create_directories(take_directory_, filesystem_error);
    if (filesystem_error) {
      return Fail(error, "Failed to create FFV1 take directory: " + filesystem_error.message());
    }
    video_path_ = take_directory_ / "video.mkv";
    frame_timestamps_path_ = take_directory_ / "frame_timestamps.csv";
    return true;
  }

  bool EnsureFrameTimestampArtifact(std::string *error) {
    if (std::filesystem::exists(frame_timestamps_path_)) {
      return true;
    }
    std::ofstream timestamps(frame_timestamps_path_);
    if (!timestamps) {
      return Fail(error, "Failed to open FFV1 frame timestamp artifact: " +
                             frame_timestamps_path_.string());
    }
    timestamps
        << "capture_index,source_timestamp_ns,sdk_host_timestamp,camera_frame_number,"
           "camera_frame_number_available,device_timestamp_ticks,source_pixel_type,"
           "source_pixel_type_name,width,height,source_payload_bytes,camera_lost_packets\n";
    timestamps.close();
    return static_cast<bool>(timestamps) ||
           Fail(error, "Failed to write FFV1 frame timestamp artifact");
  }

  std::vector<int> SliceCandidates() const {
    const std::uint64_t pixels = static_cast<std::uint64_t>(frame_contract_.width) *
                                 static_cast<std::uint64_t>(frame_contract_.height);
    if (pixels >= 1'000'000U) {
      return {12, 9, 6, 4};
    }
    return {0};
  }

  bool OpenVideo(std::string *error) {
    int result = avformat_alloc_output_context2(&format_context_, nullptr, "matroska",
                                                 video_path_.c_str());
    if (result < 0 || format_context_ == nullptr) {
      return Fail(error, "Failed to allocate FFV1/MKV output context: " +
                             FfmpegError(result));
    }
    const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_FFV1);
    if (codec == nullptr) {
      return Fail(error, "Native FFmpeg build has no FFV1 encoder");
    }
    if ((codec->capabilities & AV_CODEC_CAP_SLICE_THREADS) == 0) {
      return Fail(error, "Native FFmpeg FFV1 encoder has no slice threading support");
    }
    const AVRational frame_rate = av_d2q(config_.requested_fps, 100'000);
    if (frame_rate.num <= 0 || frame_rate.den <= 0) {
      return Fail(error, "Failed to represent requested FFV1 frame rate");
    }
    encoder_thread_count_ = std::max(
        1, std::min(12, static_cast<int>(std::thread::hardware_concurrency())));
    std::string last_open_error;
    for (const int slice_count : SliceCandidates()) {
      AVCodecContext *candidate = avcodec_alloc_context3(codec);
      if (candidate == nullptr) {
        return Fail(error, "Failed to allocate native FFmpeg FFV1 codec context");
      }
      candidate->codec_id = AV_CODEC_ID_FFV1;
      candidate->codec_type = AVMEDIA_TYPE_VIDEO;
      candidate->width = frame_contract_.width;
      candidate->height = frame_contract_.height;
      candidate->pix_fmt = AV_PIX_FMT_BGR0;
      candidate->time_base = av_inv_q(frame_rate);
      candidate->framerate = frame_rate;
      candidate->thread_count = encoder_thread_count_;
      candidate->thread_type = FF_THREAD_SLICE;
      candidate->slices = slice_count;
      result = avcodec_open2(candidate, codec, nullptr);
      if (result >= 0) {
        codec_context_ = candidate;
        encoder_slice_count_ = slice_count;
        break;
      }
      last_open_error = FfmpegError(result);
      avcodec_free_context(&candidate);
    }
    if (codec_context_ == nullptr) {
      return Fail(error, "Failed to open native FFmpeg FFV1 slice encoder: " +
                             last_open_error);
    }
    video_stream_ = avformat_new_stream(format_context_, nullptr);
    if (video_stream_ == nullptr) {
      return Fail(error, "Failed to create native FFmpeg FFV1 video stream");
    }
    video_stream_->time_base = codec_context_->time_base;
    result = avcodec_parameters_from_context(video_stream_->codecpar, codec_context_);
    if (result < 0) {
      return Fail(error, "Failed to copy FFV1 stream parameters: " + FfmpegError(result));
    }
    video_frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (video_frame_ == nullptr || packet_ == nullptr) {
      return Fail(error, "Failed to allocate native FFmpeg FFV1 frame or packet");
    }
    video_frame_->format = codec_context_->pix_fmt;
    video_frame_->width = codec_context_->width;
    video_frame_->height = codec_context_->height;
    result = av_frame_get_buffer(video_frame_, 32);
    if (result < 0) {
      return Fail(error, "Failed to allocate FFV1 frame buffer: " + FfmpegError(result));
    }
    if ((format_context_->oformat->flags & AVFMT_NOFILE) == 0) {
      result = avio_open(&format_context_->pb, video_path_.c_str(), AVIO_FLAG_WRITE);
      if (result < 0) {
        return Fail(error, "Failed to open FFV1/MKV artifact: " + FfmpegError(result));
      }
      output_io_opened_ = true;
    }
    result = avformat_write_header(format_context_, nullptr);
    if (result < 0) {
      return Fail(error, "Failed to write FFV1/MKV header: " + FfmpegError(result));
    }
    header_written_ = true;
    return true;
  }

  bool WriteAvailablePackets(std::string *error) {
    while (true) {
      const int result = avcodec_receive_packet(codec_context_, packet_);
      if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
        return true;
      }
      if (result < 0) {
        return Fail(error, "FFV1 packet encode failed: " + FfmpegError(result));
      }
      av_packet_rescale_ts(packet_, codec_context_->time_base, video_stream_->time_base);
      packet_->stream_index = video_stream_->index;
      const int write_result = av_interleaved_write_frame(format_context_, packet_);
      av_packet_unref(packet_);
      if (write_result < 0) {
        return Fail(error, "Failed to write FFV1/MKV packet: " + FfmpegError(write_result));
      }
    }
  }

  bool FinalizeVideo(std::string *error) {
    int result = avcodec_send_frame(codec_context_, nullptr);
    if (result < 0 && result != AVERROR_EOF) {
      return Fail(error, "Failed to flush FFV1 encoder: " + FfmpegError(result));
    }
    if (!WriteAvailablePackets(error)) {
      return false;
    }
    if (header_written_) {
      result = av_write_trailer(format_context_);
      if (result < 0) {
        return Fail(error, "Failed to finalize FFV1/MKV artifact: " + FfmpegError(result));
      }
      header_written_ = false;
    }
    return true;
  }

  void CloseVideo() {
    if (packet_ != nullptr) {
      av_packet_free(&packet_);
    }
    if (video_frame_ != nullptr) {
      av_frame_free(&video_frame_);
    }
    if (codec_context_ != nullptr) {
      avcodec_free_context(&codec_context_);
    }
    if (format_context_ != nullptr) {
      if (output_io_opened_) {
        avio_closep(&format_context_->pb);
      }
      avformat_free_context(format_context_);
      format_context_ = nullptr;
    }
    video_stream_ = nullptr;
    output_io_opened_ = false;
    header_written_ = false;
  }

  Ffv1CaptureArtifactWriterFactory::Config config_;
  CaptureTakeDescriptor descriptor_;
  CaptureFrameContract frame_contract_;
  std::filesystem::path take_directory_;
  std::filesystem::path video_path_;
  std::filesystem::path frame_timestamps_path_;
  std::ofstream frame_timestamps_;
  AVFormatContext *format_context_{nullptr};
  AVCodecContext *codec_context_{nullptr};
  AVStream *video_stream_{nullptr};
  AVFrame *video_frame_{nullptr};
  AVPacket *packet_{nullptr};
  std::int64_t next_frame_pts_{0};
  int encoder_thread_count_{0};
  int encoder_slice_count_{0};
  bool output_io_opened_{false};
  bool header_written_{false};
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
