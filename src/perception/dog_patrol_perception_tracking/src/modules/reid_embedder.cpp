#include "reid_embedder.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace dog_patrol_perception_tracking {
namespace {

cv::Rect MakeSafeRoi(const cv::Rect2f &bbox, const cv::Mat &frame) {
  const cv::Rect requested(static_cast<int>(std::floor(bbox.x)), static_cast<int>(std::floor(bbox.y)),
                           std::max(1, static_cast<int>(std::ceil(bbox.width))),
                           std::max(1, static_cast<int>(std::ceil(bbox.height))));
  const cv::Rect frame_rect(0, 0, frame.cols, frame.rows);
  return requested & frame_rect;
}

void L2NormalizeRow(cv::Mat *feature) {
  if (feature == nullptr || feature->empty()) {
    return;
  }
  const float norm = static_cast<float>(cv::norm(*feature, cv::NORM_L2));
  if (norm > 1e-6F) {
    *feature /= norm;
  }
}

}  // namespace

ReIdEmbedder::ReIdEmbedder() : ReIdEmbedder(Config{}) {}

ReIdEmbedder::ReIdEmbedder(Config config) : config_(std::move(config)) {
  config_.backend = NormalizeBackend(config_.backend);
  config_.input_width = std::max(16, config_.input_width);
  config_.input_height = std::max(16, config_.input_height);
  config_.light_profile = NormalizeBackend(config_.light_profile);
  config_.light_h_bins = std::max(4, config_.light_h_bins);
  config_.light_s_bins = std::max(4, config_.light_s_bins);
}

std::string ReIdEmbedder::NormalizeBackend(std::string backend) {
  std::transform(backend.begin(), backend.end(), backend.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (backend == "onnx" || backend == "osnet" || backend == "osnet_onnx" || backend == "true_reid") {
    return "osnet_onnx";
  }
  if (backend.empty()) {
    return "light";
  }
  if (backend == "tracker" || backend == "light_tracker") {
    return "light_tracker";
  }
  if (backend == "identity" || backend == "semantic" || backend == "light_identity") {
    return "light_identity";
  }
  return backend;
}

bool ReIdEmbedder::BackendUsesOnnx(const std::string &backend) {
  return NormalizeBackend(backend) == "osnet_onnx";
}

bool ReIdEmbedder::IsEnabled() const {
  return BackendUsesOnnx(config_.backend);
}

bool ReIdEmbedder::UsesOnnx() const {
  return IsEnabled();
}

bool ReIdEmbedder::Initialize(std::string *error) {
  ready_ = false;
  net_ = cv::dnn::Net();

  if (!IsEnabled()) {
    ready_ = true;
    return true;
  }

  if (config_.model_path.empty()) {
    if (error != nullptr) {
      *error = "ReID model path is empty for backend " + config_.backend;
    }
    return false;
  }
  if (!std::filesystem::exists(config_.model_path)) {
    if (error != nullptr) {
      *error = "ReID model not found: " + config_.model_path;
    }
    return false;
  }

  try {
    net_ = cv::dnn::readNetFromONNX(config_.model_path);
    ready_ = !net_.empty();
  } catch (const cv::Exception &e) {
    if (error != nullptr) {
      *error = "Failed to load ReID ONNX: " + std::string(e.what());
    }
    return false;
  }

  if (!ready_) {
    if (error != nullptr) {
      *error = "Failed to create ReID network from: " + config_.model_path;
    }
    return false;
  }

  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  return true;
}

cv::Mat ReIdEmbedder::Extract(const cv::Mat &frame, const cv::Rect2f &bbox) const {
  if (!ready_ || frame.empty()) {
    return cv::Mat();
  }

  const cv::Rect roi = MakeSafeRoi(bbox, frame);
  if (roi.width < 8 || roi.height < 8) {
    return cv::Mat();
  }

  if (!UsesOnnx()) {
    cv::Mat patch = frame(roi).clone();
    cv::Mat hsv;
    cv::cvtColor(patch, hsv, cv::COLOR_BGR2HSV);

    const int h_bins = std::max(4, config_.light_h_bins);
    const int s_bins = std::max(4, config_.light_s_bins);
    const int hist_size[] = {h_bins, s_bins};
    const float h_ranges[] = {0.0F, 180.0F};
    const float s_ranges[] = {0.0F, 256.0F};
    const float *ranges[] = {h_ranges, s_ranges};
    const int channels[] = {0, 1};

    const int half = std::max(1, hsv.rows / 2);
    const cv::Rect top_roi(0, 0, hsv.cols, half);
    const cv::Rect bottom_roi(0, half, hsv.cols, hsv.rows - half);
    cv::Mat top = hsv(top_roi);
    cv::Mat bottom = hsv(bottom_roi);

    cv::Mat hist_top;
    cv::Mat hist_bottom;
    cv::calcHist(&top, 1, channels, cv::Mat(), hist_top, 2, hist_size, ranges, true, false);
    cv::calcHist(&bottom, 1, channels, cv::Mat(), hist_bottom, 2, hist_size, ranges, true, false);

    hist_top = hist_top.reshape(1, 1);
    hist_bottom = hist_bottom.reshape(1, 1);
    hist_top.convertTo(hist_top, CV_32F);
    hist_bottom.convertTo(hist_bottom, CV_32F);

    const float sum_top = static_cast<float>(cv::sum(hist_top)[0]);
    const float sum_bottom = static_cast<float>(cv::sum(hist_bottom)[0]);
    if (sum_top <= 1e-6F || sum_bottom <= 1e-6F) {
      return cv::Mat();
    }
    hist_top /= sum_top;
    hist_bottom /= sum_bottom;

    cv::Mat feat;
    cv::hconcat(hist_top, hist_bottom, feat);
    const float feat_sum = static_cast<float>(cv::sum(feat)[0]);
    if (feat_sum > 1e-6F) {
      feat /= feat_sum;
    }
    return feat;
  }

  cv::Mat patch = frame(roi).clone();
  cv::Mat blob = cv::dnn::blobFromImage(
      patch, 1.0 / 255.0, cv::Size(config_.input_width, config_.input_height),
      cv::Scalar(0.485 * 255.0, 0.456 * 255.0, 0.406 * 255.0), true, false, CV_32F);

  constexpr float kStd[3] = {0.229F, 0.224F, 0.225F};
  const int hw = config_.input_width * config_.input_height;
  float *blob_data = reinterpret_cast<float *>(blob.data);
  for (int c = 0; c < 3; ++c) {
    const float denom = std::max(1e-6F, kStd[c]);
    float *channel = blob_data + c * hw;
    for (int i = 0; i < hw; ++i) {
      channel[i] /= denom;
    }
  }

  cv::Mat embedding;
  try {
    cv::dnn::Net net = net_;
    net.setInput(blob);
    embedding = net.forward();
  } catch (const cv::Exception &) {
    return cv::Mat();
  }

  if (embedding.empty()) {
    return cv::Mat();
  }

  cv::Mat row = embedding.reshape(1, 1);
  row.convertTo(row, CV_32F);
  L2NormalizeRow(&row);
  if (cv::norm(row, cv::NORM_L2) <= 1e-6F) {
    return cv::Mat();
  }
  return row;
}

std::vector<float> ReIdEmbedder::ExtractVector(const cv::Mat &frame, const cv::Rect2f &bbox) const {
  if (!UsesOnnx()) {
    if (!ready_ || frame.empty()) {
      return {};
    }
    const cv::Rect roi = MakeSafeRoi(bbox, frame);
    if (roi.width < 8 || roi.height < 8) {
      return {};
    }
    cv::Mat patch = frame(roi).clone();
    cv::Mat hsv;
    cv::cvtColor(patch, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    std::vector<float> out;
    out.reserve(24);

    auto append_hist = [&](const cv::Mat &channel, const int bins, const float range_min, const float range_max) {
      cv::Mat hist;
      const int hist_size[] = {bins};
      const float ranges[] = {range_min, range_max};
      const float *hist_ranges[] = {ranges};
      const int channel_idx[] = {0};
      cv::calcHist(&channel, 1, channel_idx, cv::Mat(), hist, 1, hist_size, hist_ranges, true, false);
      const float sum = static_cast<float>(cv::sum(hist)[0]);
      const float inv = sum > 1e-6F ? (1.0F / sum) : 1.0F;
      for (int i = 0; i < bins; ++i) {
        out.push_back(hist.at<float>(i) * inv);
      }
    };

    append_hist(channels[0], 12, 0.0F, 180.0F);
    append_hist(channels[1], 6, 0.0F, 256.0F);
    append_hist(channels[2], 6, 0.0F, 256.0F);
    return out;
  }

  const cv::Mat feature = Extract(frame, bbox);
  if (feature.empty()) {
    return {};
  }
  std::vector<float> out(static_cast<std::size_t>(feature.cols));
  for (int i = 0; i < feature.cols; ++i) {
    out[static_cast<std::size_t>(i)] = feature.at<float>(0, i);
  }
  return out;
}

}  // namespace dog_patrol_perception_tracking
