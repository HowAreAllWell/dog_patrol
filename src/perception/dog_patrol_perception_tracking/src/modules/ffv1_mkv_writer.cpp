#include "vision_demo_host/modules/ffv1_mkv_writer.hpp"

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

std::string FfmpegError(const int code) {
  char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(code, buffer, sizeof(buffer));
  return buffer;
}

}  // namespace

class Ffv1MkvWriter::Impl {
 public:
  explicit Impl(Config config) : config_(std::move(config)) {}

  ~Impl() { CloseUnchecked(); }

  bool Open(std::string *error) {
    if (opened_) {
      return Fail(error, "FFV1/MKV writer is already open");
    }
    if (config_.output_path.extension() != ".mkv") {
      return Fail(error, "FFV1 writer output path must end in .mkv");
    }
    if (config_.width <= 0 || config_.height <= 0) {
      return Fail(error, "FFV1 writer requires positive frame dimensions");
    }
    if (!std::isfinite(config_.fps) || config_.fps <= 0.0) {
      return Fail(error, "FFV1 writer requires a finite positive fps");
    }

    int result = avformat_alloc_output_context2(&format_context_, nullptr, "matroska",
                                                 config_.output_path.c_str());
    if (result < 0 || format_context_ == nullptr) {
      return Fail(error, "Failed to allocate FFV1/MKV output context: " + FfmpegError(result));
    }
    const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_FFV1);
    if (codec == nullptr) {
      CloseUnchecked();
      return Fail(error, "Native FFmpeg build has no FFV1 encoder");
    }
    if ((codec->capabilities & AV_CODEC_CAP_SLICE_THREADS) == 0) {
      CloseUnchecked();
      return Fail(error, "Native FFmpeg FFV1 encoder has no slice threading support");
    }
    const AVRational frame_rate = av_d2q(config_.fps, 100'000);
    if (frame_rate.num <= 0 || frame_rate.den <= 0) {
      CloseUnchecked();
      return Fail(error, "Failed to represent requested FFV1 frame rate");
    }

    info_.thread_count = std::max(1, std::min(12, static_cast<int>(std::thread::hardware_concurrency())));
    std::string last_open_error;
    for (const int slice_count : SliceCandidates()) {
      AVCodecContext *candidate = avcodec_alloc_context3(codec);
      if (candidate == nullptr) {
        CloseUnchecked();
        return Fail(error, "Failed to allocate native FFmpeg FFV1 codec context");
      }
      candidate->codec_id = AV_CODEC_ID_FFV1;
      candidate->codec_type = AVMEDIA_TYPE_VIDEO;
      candidate->width = config_.width;
      candidate->height = config_.height;
      candidate->pix_fmt = AV_PIX_FMT_BGR0;
      candidate->time_base = av_inv_q(frame_rate);
      candidate->framerate = frame_rate;
      candidate->thread_count = info_.thread_count;
      candidate->thread_type = FF_THREAD_SLICE;
      candidate->slices = slice_count;
      result = avcodec_open2(candidate, codec, nullptr);
      if (result >= 0) {
        codec_context_ = candidate;
        info_.slice_count = slice_count;
        break;
      }
      last_open_error = FfmpegError(result);
      avcodec_free_context(&candidate);
    }
    if (codec_context_ == nullptr) {
      CloseUnchecked();
      return Fail(error, "Failed to open native FFmpeg FFV1 slice encoder: " + last_open_error);
    }

    video_stream_ = avformat_new_stream(format_context_, nullptr);
    if (video_stream_ == nullptr) {
      CloseUnchecked();
      return Fail(error, "Failed to create native FFmpeg FFV1 video stream");
    }
    video_stream_->time_base = codec_context_->time_base;
    result = avcodec_parameters_from_context(video_stream_->codecpar, codec_context_);
    if (result < 0) {
      CloseUnchecked();
      return Fail(error, "Failed to copy FFV1 stream parameters: " + FfmpegError(result));
    }
    video_frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (video_frame_ == nullptr || packet_ == nullptr) {
      CloseUnchecked();
      return Fail(error, "Failed to allocate native FFmpeg FFV1 frame or packet");
    }
    video_frame_->format = codec_context_->pix_fmt;
    video_frame_->width = codec_context_->width;
    video_frame_->height = codec_context_->height;
    result = av_frame_get_buffer(video_frame_, 32);
    if (result < 0) {
      CloseUnchecked();
      return Fail(error, "Failed to allocate FFV1 frame buffer: " + FfmpegError(result));
    }
    if ((format_context_->oformat->flags & AVFMT_NOFILE) == 0) {
      result = avio_open(&format_context_->pb, config_.output_path.c_str(), AVIO_FLAG_WRITE);
      if (result < 0) {
        CloseUnchecked();
        return Fail(error, "Failed to open FFV1/MKV artifact: " + FfmpegError(result));
      }
      output_io_opened_ = true;
    }
    result = avformat_write_header(format_context_, nullptr);
    if (result < 0) {
      CloseUnchecked();
      return Fail(error, "Failed to write FFV1/MKV header: " + FfmpegError(result));
    }
    opened_ = true;
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Write(const cv::Mat &bgr8, std::string *error) {
    if (!opened_) {
      return Fail(error, "FFV1/MKV writer is not open");
    }
    if (bgr8.empty() || bgr8.type() != CV_8UC3 || bgr8.cols != config_.width ||
        bgr8.rows != config_.height) {
      return Fail(error, "FFV1 writer frame violates the BGR8 frame contract");
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
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Close(std::string *error) {
    if (!opened_) {
      if (error != nullptr) {
        error->clear();
      }
      return true;
    }
    int result = avcodec_send_frame(codec_context_, nullptr);
    if (result < 0) {
      const std::string message = "Failed to flush FFV1 encoder: " + FfmpegError(result);
      CloseUnchecked();
      return Fail(error, message);
    }
    if (!WriteAvailablePackets(error)) {
      CloseUnchecked();
      return false;
    }
    result = av_write_trailer(format_context_);
    if (result < 0) {
      const std::string message = "Failed to finalize FFV1/MKV artifact: " + FfmpegError(result);
      CloseUnchecked();
      return Fail(error, message);
    }
    CloseUnchecked();
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  const EncoderInfo &Info() const { return info_; }

 private:
  std::vector<int> SliceCandidates() const {
    const std::uint64_t pixels = static_cast<std::uint64_t>(config_.width) *
                                 static_cast<std::uint64_t>(config_.height);
    if (pixels >= 1'000'000U) {
      return {12, 9, 6, 4};
    }
    return {0};
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

  void CloseUnchecked() {
    if (output_io_opened_ && format_context_ != nullptr && format_context_->pb != nullptr) {
      avio_closep(&format_context_->pb);
    }
    output_io_opened_ = false;
    av_frame_free(&video_frame_);
    av_packet_free(&packet_);
    avcodec_free_context(&codec_context_);
    avformat_free_context(format_context_);
    format_context_ = nullptr;
    video_stream_ = nullptr;
    opened_ = false;
  }

  Config config_;
  EncoderInfo info_;
  AVFormatContext *format_context_{nullptr};
  AVCodecContext *codec_context_{nullptr};
  AVStream *video_stream_{nullptr};
  AVFrame *video_frame_{nullptr};
  AVPacket *packet_{nullptr};
  bool output_io_opened_{false};
  bool opened_{false};
  std::int64_t next_frame_pts_{0};
};

Ffv1MkvWriter::Ffv1MkvWriter(Config config) : impl_(std::make_unique<Impl>(std::move(config))) {}

Ffv1MkvWriter::~Ffv1MkvWriter() = default;

bool Ffv1MkvWriter::Open(std::string *error) { return impl_->Open(error); }

bool Ffv1MkvWriter::Write(const cv::Mat &bgr8, std::string *error) {
  return impl_->Write(bgr8, error);
}

bool Ffv1MkvWriter::Close(std::string *error) { return impl_->Close(error); }

const Ffv1MkvWriter::EncoderInfo &Ffv1MkvWriter::Info() const { return impl_->Info(); }

}  // namespace vision_demo_host
