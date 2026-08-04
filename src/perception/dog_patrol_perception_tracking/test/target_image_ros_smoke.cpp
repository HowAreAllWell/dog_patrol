#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dog_patrol_perception_interfaces/msg/tracked_target_image.hpp"
#include "dog_patrol_perception_tracking/modules/target_image_ros_adapter.hpp"

namespace {

constexpr const char *kTopic = "/test/tracked_target_image";

dog_patrol_perception_tracking::PrimaryTargetObservation Observation(
    std::uint64_t timestamp_ns) {
  dog_patrol_perception_tracking::PrimaryTargetObservation observation;
  observation.target_id = 42;
  observation.confidence = 0.9F;
  observation.source.source_timestamp_ns = timestamp_ns;
  observation.source.camera_frame_number = 9U;
  observation.source.camera_frame_number_available = true;
  observation.source.image_width = 8;
  observation.source.image_height = 6;
  observation.source.optical_frame_id = "hik_camera_optical_frame";
  observation.bbox = cv::Rect{2, 1, 3, 4};
  observation.target_image = cv::Mat(4, 3, CV_8UC3, cv::Scalar{10, 20, 30}).clone();
  return observation;
}

int Subscribe() {
  auto node = std::make_shared<rclcpp::Node>("target_image_smoke_subscriber");
  bool valid = false;
  auto subscription = node->create_subscription<
      dog_patrol_perception_interfaces::msg::TrackedTargetImage>(
      kTopic, dog_patrol_perception_tracking::TargetImageRosAdapter::Qos(),
      [&](const dog_patrol_perception_interfaces::msg::TrackedTargetImage &message) {
        valid = message.target_id == 42 && message.source_frame_id == "hik_camera_optical_frame" &&
                message.source_image_width == 8U && message.source_image_height == 6U &&
                message.bbox_x == 2 && message.bbox_y == 1 &&
                message.bbox_width == 3U && message.bbox_height == 4U &&
                message.encoding == "bgr8" && message.crop_width == 3U &&
                message.crop_height == 4U && message.crop_step == 9U &&
                message.crop_data.size() == 36U && message.crop_data[0] == 10U;
      });
  (void)subscription;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (rclcpp::ok() && !valid && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return valid ? 0 : 1;
}

int Publish() {
  auto node = std::make_shared<rclcpp::Node>("target_image_smoke_publisher");
  dog_patrol_perception_tracking::TargetImageRosAdapter::Config config;
  config.topic = kTopic;
  config.queue_capacity = 2U;
  config.max_publish_hz = 20.0;
  dog_patrol_perception_tracking::TargetImageRosAdapter adapter(*node, config);
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  for (std::uint64_t index = 0U; index < 10U; ++index) {
    adapter.Consume(Observation(1710000000000000000ULL + index * 100000000ULL));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const int result = argc == 2 && std::string(argv[1]) == "subscribe"
                         ? Subscribe()
                         : (argc == 2 && std::string(argv[1]) == "publish" ? Publish() : 2);
  rclcpp::shutdown();
  return result;
}
