#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <mutex>
#include <thread>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "dog_patrol_perception_tracking/modules/target_image_ros_adapter.hpp"

namespace {

using dog_patrol_perception_tracking::TargetImageRosAdapter;

dog_patrol_perception_tracking::PrimaryTargetObservation TargetImageObservation(
    std::uint64_t timestamp_ns, int target_id = 42,
    std::uint32_t frame_number = 7U) {
  dog_patrol_perception_tracking::PrimaryTargetObservation observation;
  observation.target_id = target_id;
  observation.confidence = 0.91F;
  observation.source.source_timestamp_ns = timestamp_ns;
  observation.source.camera_frame_number = frame_number;
  observation.source.camera_frame_number_available = true;
  observation.source.image_width = 8;
  observation.source.image_height = 6;
  observation.source.optical_frame_id = "hik_camera_optical_frame";
  observation.bbox = cv::Rect{2, 1, 3, 4};
  observation.target_image =
      cv::Mat(4, 3, CV_8UC3, cv::Scalar{10, 20, 30}).clone();
  return observation;
}

TEST(TargetImageRosAdapterTest, ConvertsOwnedBgrCropAndSourceCoordinates) {
  auto message = TargetImageRosAdapter::ToMessage(
      TargetImageObservation(1710000000123456789ULL));

  EXPECT_EQ(message->source_stamp.sec, 1710000000);
  EXPECT_EQ(message->source_stamp.nanosec, 123456789U);
  EXPECT_EQ(message->source_frame_id, "hik_camera_optical_frame");
  EXPECT_EQ(message->target_id, 42);
  EXPECT_EQ(message->source_image_width, 8U);
  EXPECT_EQ(message->bbox_x, 2);
  EXPECT_EQ(message->bbox_height, 4U);
  EXPECT_EQ(message->encoding, "bgr8");
  EXPECT_EQ(message->crop_width, 3U);
  EXPECT_EQ(message->crop_height, 4U);
  EXPECT_EQ(message->crop_step, 9U);
  ASSERT_EQ(message->crop_data.size(), 36U);
  EXPECT_EQ(message->crop_data[0], 10U);
  EXPECT_EQ(message->crop_data[1], 20U);
  EXPECT_EQ(message->crop_data[2], 30U);
}

TEST(TargetImageRosAdapterTest, SlowPublisherDropsOldCropsWithoutBlockingProducer) {
  std::mutex mutex;
  std::condition_variable first_publish_started;
  std::condition_variable release_publisher;
  bool started = false;
  bool release = false;
  std::atomic<int> published{0};
  TargetImageRosAdapter::Config config;
  config.queue_capacity = 2U;
  config.max_publish_hz = 1000.0;
  TargetImageRosAdapter adapter(config, [&](TargetImageRosAdapter::Message::UniquePtr) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      started = true;
    }
    first_publish_started.notify_one();
    ++published;
    std::unique_lock<std::mutex> lock(mutex);
    release_publisher.wait(lock, [&] { return release; });
  });

  adapter.Consume(TargetImageObservation(1000000000ULL));
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(first_publish_started.wait_for(
        lock, std::chrono::seconds(1), [&] { return started; }));
  }
  auto producer = std::async(std::launch::async, [&] {
    adapter.Consume(std::nullopt);
    for (int index = 1; index <= 20; ++index) {
      adapter.Consume(TargetImageObservation(
          1000000000ULL + static_cast<std::uint64_t>(index) * 2000000ULL));
    }
  });
  EXPECT_EQ(producer.wait_for(std::chrono::seconds(2)),
            std::future_status::ready);
  {
    std::lock_guard<std::mutex> lock(mutex);
    release = true;
  }
  release_publisher.notify_all();
  producer.get();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const auto metrics = adapter.GetMetrics();
  EXPECT_EQ(metrics.submitted, 21U);
  EXPECT_GT(metrics.queue_dropped, 0U);
  EXPECT_LT(published.load(), 21);
}

TEST(TargetImageRosAdapterTest, InvalidatingTargetClearsPendingHistoricalCrop) {
  std::mutex mutex;
  std::condition_variable started_cv;
  bool started = false;
  TargetImageRosAdapter::Config config;
  config.queue_capacity = 2U;
  config.max_publish_hz = 1000.0;
  TargetImageRosAdapter adapter(config, [&](TargetImageRosAdapter::Message::UniquePtr) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      started = true;
    }
    started_cv.notify_one();
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
  });
  adapter.Consume(TargetImageObservation(1000000000ULL));
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(started_cv.wait_for(lock, std::chrono::seconds(1), [&] { return started; }));
  }
  adapter.Consume(TargetImageObservation(1002000000ULL));
  adapter.Consume(std::nullopt);

  EXPECT_FALSE(adapter.HasCurrentObservation());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(adapter.GetMetrics().published, 1U);
}

TEST(TargetImageRosAdapterTest, InvalidationCancelsCropAlreadyDequeuedForConversion) {
  std::mutex mutex;
  std::condition_variable converting_cv;
  std::condition_variable release_cv;
  bool converting = false;
  bool release = false;
  std::atomic<int> published{0};
  TargetImageRosAdapter::Config config;
  config.max_publish_hz = 1000.0;
  TargetImageRosAdapter adapter(
      config,
      [&](TargetImageRosAdapter::Message::UniquePtr) { ++published; },
      [&](const dog_patrol_perception_tracking::PrimaryTargetObservation &observation) {
        std::unique_lock<std::mutex> lock(mutex);
        converting = true;
        converting_cv.notify_one();
        release_cv.wait(lock, [&] { return release; });
        return TargetImageRosAdapter::ToMessage(observation);
      });
  adapter.Consume(TargetImageObservation(1000000000ULL));
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(converting_cv.wait_for(
        lock, std::chrono::seconds(1), [&] { return converting; }));
  }

  adapter.Consume(std::nullopt);
  {
    std::lock_guard<std::mutex> lock(mutex);
    release = true;
  }
  release_cv.notify_one();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  EXPECT_EQ(published.load(), 0);
  EXPECT_FALSE(adapter.HasCurrentObservation());
  EXPECT_EQ(adapter.GetMetrics().queue_dropped, 1U);
}

}  // namespace
