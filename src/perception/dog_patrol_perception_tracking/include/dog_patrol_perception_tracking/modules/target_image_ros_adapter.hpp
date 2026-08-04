#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "dog_patrol_perception_interfaces/msg/tracked_target_image.hpp"
#include "dog_patrol_perception_tracking/modules/primary_target_observer.hpp"

namespace dog_patrol_perception_tracking {

// A non-blocking boundary between the tracking frame thread and ROS/DDS. New
// observations displace the oldest pending crop when the bounded queue is
// full; ROS serialization and middleware work happen only on the worker.
class TargetImageRosAdapter final : public PrimaryTargetObservationSink {
 public:
  using Message = dog_patrol_perception_interfaces::msg::TrackedTargetImage;
  using PublishFunction = std::function<void(Message::UniquePtr)>;
  using ConvertFunction =
      std::function<Message::UniquePtr(const PrimaryTargetObservation &)>;

  struct Config {
    std::string topic{"/perception/tracked_target_image"};
    std::size_t queue_capacity{2U};
    double max_publish_hz{10.0};
  };

  struct Metrics {
    std::uint64_t submitted{0U};
    std::uint64_t published{0U};
    std::uint64_t queue_dropped{0U};
    std::uint64_t rate_limited{0U};
  };

  TargetImageRosAdapter(rclcpp::Node &node, Config config);
  // Test seam for deliberately slow consumers without a DDS dependency.
  TargetImageRosAdapter(Config config, PublishFunction publish);
  TargetImageRosAdapter(Config config, PublishFunction publish,
                        ConvertFunction convert);
  ~TargetImageRosAdapter() override;

  TargetImageRosAdapter(const TargetImageRosAdapter &) = delete;
  TargetImageRosAdapter &operator=(const TargetImageRosAdapter &) = delete;

  void Consume(std::optional<PrimaryTargetObservation> observation) override;
  Metrics GetMetrics() const;
  bool HasCurrentObservation() const;

  static Message::UniquePtr ToMessage(const PrimaryTargetObservation &observation);
  static rclcpp::QoS Qos();

 private:
  struct PendingObservation {
    PrimaryTargetObservation observation;
    std::uint64_t generation{0U};
  };

  void Start();
  void Run();
  void BeginPublish(Message::UniquePtr message);

  Config config_;
  PublishFunction publish_;
  ConvertFunction convert_;
  rclcpp::Publisher<Message>::SharedPtr publisher_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable publication_started_;
  std::deque<PendingObservation> queue_;
  Metrics metrics_;
  std::uint64_t last_published_source_ns_{0U};
  std::uint64_t generation_{0U};
  bool current_observation_{false};
  bool publish_starting_{false};
  bool stopping_{false};
  std::thread worker_;
};

}  // namespace dog_patrol_perception_tracking
