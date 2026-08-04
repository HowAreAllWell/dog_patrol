#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dog_patrol_perception_interfaces/msg/tracked_target_image.hpp"
#include "dog_patrol_perception_tracking/modules/primary_target_observer.hpp"
#include "dog_patrol_perception_tracking/modules/target_image_ros_adapter.hpp"

namespace {

constexpr const char *kTopic = "/test/tracked_target_image";

int Subscribe() {
  auto node = std::make_shared<rclcpp::Node>("target_image_smoke_subscriber");
  bool valid = false;
  std::uint32_t latest_frame = 0U;
  std::size_t received = 0U;
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
        latest_frame = message.source_frame_number;
        ++received;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      });
  (void)subscription;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (rclcpp::ok() && latest_frame < 90U &&
         std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return valid && latest_frame >= 90U && received < 100U ? 0 : 1;
}

int Publish() {
  auto node = std::make_shared<rclcpp::Node>("target_image_smoke_publisher");
  dog_patrol_perception_tracking::TargetImageRosAdapter::Config config;
  config.topic = kTopic;
  config.queue_capacity = 2U;
  config.max_publish_hz = 1000.0;
  auto adapter = std::make_shared<
      dog_patrol_perception_tracking::TargetImageRosAdapter>(*node, config);
  dog_patrol_perception_tracking::PrimaryTargetManager::Config primary_config;
  primary_config.min_person_area_px = 1.0F;
  dog_patrol_perception_tracking::PrimaryTargetObserver observer(
      primary_config, adapter);
  dog_patrol_perception_tracking::IdentityObservation identity;
  identity.semantic_id = 42;
  identity.state = dog_patrol_perception_tracking::IdentityState::kActive;
  identity.supporting_raw_track_id = 7;
  identity.class_id = dog_patrol_perception_tracking::ClassId::kPerson;
  identity.confidence = 0.91F;
  identity.bbox = cv::Rect2f{2.0F, 1.0F, 3.0F, 4.0F};
  identity.visible = true;
  cv::Mat source_image(6, 8, CV_8UC3, cv::Scalar{10, 20, 30});
  const auto process_frame = [&](const std::uint32_t frame_number) {
    dog_patrol_perception_tracking::SourceFrameMetadata source;
    source.source_timestamp_ns = 1710000000000000000ULL +
                                 static_cast<std::uint64_t>(frame_number) * 2000000ULL;
    source.camera_frame_number = frame_number;
    source.camera_frame_number_available = true;
    source.image_width = source_image.cols;
    source.image_height = source_image.rows;
    source.optical_frame_id = "hik_camera_optical_frame";
    observer.Update({identity}, source, source_image);
    rclcpp::spin_some(node);
  };
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  const auto frame_loop_start = std::chrono::steady_clock::now();
  for (std::uint32_t index = 0U; index < 100U; ++index) {
    process_frame(index);
  }
  const auto frame_loop_elapsed = std::chrono::steady_clock::now() - frame_loop_start;
  if (frame_loop_elapsed >= std::chrono::milliseconds(200)) {
    RCLCPP_ERROR(node->get_logger(), "observer frame loop was blocked for %lld ms",
                 static_cast<long long>(
                     std::chrono::duration_cast<std::chrono::milliseconds>(frame_loop_elapsed)
                         .count()));
    return 1;
  }
  // Continue a short live cadence after the burst so DDS discovery timing
  // cannot turn the slow-consumer assertion into a race.
  for (std::uint32_t index = 100U; index < 110U; ++index) {
    process_frame(index);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
