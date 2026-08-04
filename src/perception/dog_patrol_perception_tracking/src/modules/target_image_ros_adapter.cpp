#include "dog_patrol_perception_tracking/modules/target_image_ros_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace dog_patrol_perception_tracking {

TargetImageRosAdapter::TargetImageRosAdapter(rclcpp::Node &node, Config config)
    : config_(std::move(config)) {
  publisher_ = node.create_publisher<Message>(config_.topic, Qos());
  publish_ = [publisher = publisher_](Message::UniquePtr message) {
    publisher->publish(std::move(message));
  };
  convert_ = ToMessage;
  Start();
}

TargetImageRosAdapter::TargetImageRosAdapter(
    Config config, PublishFunction publish)
    : TargetImageRosAdapter(std::move(config), std::move(publish), ToMessage) {}

TargetImageRosAdapter::TargetImageRosAdapter(
    Config config, PublishFunction publish, ConvertFunction convert)
    : config_(std::move(config)), publish_(std::move(publish)),
      convert_(std::move(convert)) {
  Start();
}

void TargetImageRosAdapter::Start() {
  if (config_.topic.empty() || config_.queue_capacity == 0U ||
      !std::isfinite(config_.max_publish_hz) || config_.max_publish_hz <= 0.0 ||
      !publish_ || !convert_) {
    throw std::invalid_argument(
        "target image topic/publisher must be set, queue_capacity must be positive, "
        "and max_publish_hz must be finite and positive");
  }
  worker_ = std::thread(&TargetImageRosAdapter::Run, this);
}

TargetImageRosAdapter::~TargetImageRosAdapter() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    queue_.clear();
  }
  wake_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
}

rclcpp::QoS TargetImageRosAdapter::Qos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

void TargetImageRosAdapter::Consume(
    std::optional<PrimaryTargetObservation> observation) {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++generation_;
    current_observation_ = observation.has_value();
    if (!observation.has_value()) {
      queue_.clear();
      // A publish that already linearized is in flight and may finish, but an
      // invalidation never returns while a worker is between its final
      // generation check and that linearization point. It does not wait for
      // middleware work after publication has begun.
      publication_started_.wait(lock, [this] { return !publish_starting_; });
      return;
    }
    ++metrics_.submitted;
    while (queue_.size() >= config_.queue_capacity) {
      queue_.pop_front();
      ++metrics_.queue_dropped;
    }
    queue_.push_back({std::move(*observation), generation_});
  }
  wake_.notify_one();
}

TargetImageRosAdapter::Metrics TargetImageRosAdapter::GetMetrics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return metrics_;
}

bool TargetImageRosAdapter::HasCurrentObservation() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_observation_;
}

TargetImageRosAdapter::Message::UniquePtr TargetImageRosAdapter::ToMessage(
    const PrimaryTargetObservation &observation) {
  if (observation.target_image.empty() || observation.target_image.type() != CV_8UC3) {
    throw std::invalid_argument("tracked target crop must be a non-empty CV_8UC3 image");
  }
  auto message = std::make_unique<Message>();
  message->source_stamp.sec = static_cast<std::int32_t>(
      observation.source.source_timestamp_ns / 1000000000ULL);
  message->source_stamp.nanosec = static_cast<std::uint32_t>(
      observation.source.source_timestamp_ns % 1000000000ULL);
  message->source_frame_id = observation.source.optical_frame_id;
  message->source_frame_number = observation.source.camera_frame_number;
  message->source_frame_number_available = observation.source.camera_frame_number_available;
  message->target_id = observation.target_id;
  message->confidence = observation.confidence;
  message->source_image_width = static_cast<std::uint32_t>(observation.source.image_width);
  message->source_image_height = static_cast<std::uint32_t>(observation.source.image_height);
  message->bbox_x = observation.bbox.x;
  message->bbox_y = observation.bbox.y;
  message->bbox_width = static_cast<std::uint32_t>(observation.bbox.width);
  message->bbox_height = static_cast<std::uint32_t>(observation.bbox.height);
  message->encoding = "bgr8";
  message->crop_width = static_cast<std::uint32_t>(observation.target_image.cols);
  message->crop_height = static_cast<std::uint32_t>(observation.target_image.rows);
  message->crop_step = static_cast<std::uint32_t>(observation.target_image.cols * 3);
  const std::size_t row_bytes = message->crop_step;
  message->crop_data.resize(row_bytes * message->crop_height);
  for (int row = 0; row < observation.target_image.rows; ++row) {
    std::memcpy(message->crop_data.data() + static_cast<std::size_t>(row) * row_bytes,
                observation.target_image.ptr(row), row_bytes);
  }
  return message;
}

void TargetImageRosAdapter::Run() {
  const auto minimum_period_ns = static_cast<std::uint64_t>(
      std::ceil(1000000000.0 / config_.max_publish_hz));
  for (;;) {
    PendingObservation pending;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
      if (stopping_) {
        return;
      }
      pending = std::move(queue_.back());
      if (queue_.size() > 1U) {
        metrics_.queue_dropped += queue_.size() - 1U;
      }
      queue_.clear();
      if (last_published_source_ns_ != 0U &&
          pending.observation.source.source_timestamp_ns <
              last_published_source_ns_ + minimum_period_ns) {
        ++metrics_.rate_limited;
        continue;
      }
    }
    try {
      auto message = convert_(pending.observation);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_observation_ || pending.generation != generation_) {
          ++metrics_.queue_dropped;
          continue;
        }
        last_published_source_ns_ = pending.observation.source.source_timestamp_ns;
        publish_starting_ = true;
      }
      BeginPublish(std::move(message));
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++metrics_.published;
      }
    } catch (...) {
      // A middleware failure must never unwind onto or stop the tracking
      // pipeline. The next current observation remains eligible for publish.
    }
  }
}

void TargetImageRosAdapter::BeginPublish(Message::UniquePtr message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Linearization point for publication. Invalidation waits only for this
    // transition, never for ROS serialization or DDS/middleware completion.
    publish_starting_ = false;
  }
  publication_started_.notify_all();
  publish_(std::move(message));
}

}  // namespace dog_patrol_perception_tracking
