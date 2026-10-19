#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dog_patrol_interfaces/msg/mission_state.hpp"
#include "dog_patrol_perception_tracking/modules/mission_tracking_runtime.hpp"
#include "dog_patrol_reid_provider/srv/re_id_dedup.hpp"

namespace {

using namespace dog_patrol_perception_tracking;
using namespace std::chrono_literals;
using MissionStateMsg = dog_patrol_interfaces::msg::MissionState;
using DedupSrv = dog_patrol_reid_provider::srv::ReIDDedup;
namespace fs = std::filesystem;

constexpr std::uint64_t kStartStamp = 1710000000000000000ULL;

IdentityObservation Person() {
  IdentityObservation person;
  person.semantic_id = 42;
  person.state = IdentityState::kActive;
  person.visible = true;
  person.class_id = ClassId::kPerson;
  person.supporting_raw_track_id = 7;
  person.bbox = cv::Rect2f{16.0F, 8.0F, 32.0F, 48.0F};
  person.confidence = 0.95F;
  return person;
}

SourceFrameMetadata Metadata(std::uint64_t stamp) {
  SourceFrameMetadata metadata;
  metadata.source_timestamp_ns = stamp;
  metadata.image_width = 96;
  metadata.image_height = 64;
  metadata.optical_frame_id = "camera_optical_frame";
  return metadata;
}

cv::Mat Frame(const cv::Scalar &color) {
  return cv::Mat(64, 96, CV_8UC3, color);
}

class MissionPhotoEvidenceTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    suffix_ = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    root_ = fs::temp_directory_path() /
            ("dog_patrol_mission_photos_" + suffix_ + "_" +
             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    EventPhotoRecorder::Config photo_config;
    photo_config.root = root_.string();
    recorder_ = std::make_shared<EventPhotoRecorder>(photo_config);
    node_ = rclcpp::Node::make_shared("mission_photos_" + suffix_);
    image_adapter_ = std::make_shared<TargetImageRosAdapter>(
        TargetImageRosAdapter::Config{},
        [](dog_patrol_perception_interfaces::msg::TrackedTargetImage::UniquePtr) {});
  }

  void BuildRuntime(ReidConfirmGate *gate = nullptr) {
    MissionRosAdapter::Config config;
    config.mission_state_topic = "/mission_photos/" + suffix_ + "/state";
    config.mission_event_topic = "/mission_photos/" + suffix_ + "/event";
    config.target_bbox_topic = "/mission_photos/" + suffix_ + "/bbox";
    config.capability_status_topic = "/mission_photos/" + suffix_ + "/status";
    config.state_timeout = 30s;
    auto adapter = std::make_unique<MissionRosAdapter>(*node_, config);
    adapter_ = adapter.get();
    runtime_ = std::make_unique<MissionTrackingRuntime>(
        std::move(adapter), image_adapter_, PrimaryTargetCropConfig{}, gate, recorder_);
  }

  void TearDown() override {
    runtime_.reset();
    image_adapter_.reset();
    recorder_->Flush();
    EXPECT_EQ(recorder_->GetMetrics().write_errors, 0U);
    recorder_.reset();
    node_.reset();
    std::error_code error;
    fs::remove_all(root_, error);
    EXPECT_FALSE(error) << error.message();
  }

  void Store(std::uint32_t seq, std::uint8_t state, std::uint32_t target,
             std::uint64_t stamp) {
    MissionStateMsg message;
    message.header.stamp.sec = static_cast<std::int32_t>(stamp / 1000000000ULL);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(stamp % 1000000000ULL);
    message.state_seq = seq;
    message.state = state;
    message.target_id = target;
    ASSERT_TRUE(adapter_->StoreMissionState(message));
  }

  std::vector<fs::path> Photos(const fs::path &relative_directory) {
    recorder_->Flush();
    std::vector<fs::path> result;
    const auto directory = root_ / relative_directory;
    if (!fs::exists(directory)) {
      return result;
    }
    const std::regex filename_pattern(
        R"(^[0-9]{8}_[0-9]{6}_t([1-9][0-9]*)(_[1-9][0-9]*)?\.jpg$)");
    for (const auto &entry : fs::directory_iterator(directory)) {
      if (entry.is_regular_file() && entry.path().extension() == ".jpg") {
        const auto filename = entry.path().filename().string();
        std::smatch match;
        const bool valid_filename = std::regex_match(filename, match, filename_pattern);
        EXPECT_TRUE(valid_filename) << entry.path();
        if (valid_filename) {
          // Both event and lost photos belong to this fixture's semantic ID.
          EXPECT_EQ(match[1].str(), "42") << entry.path();
        }
        result.push_back(entry.path());
      }
    }
    std::sort(result.begin(), result.end());
    return result;
  }

  static void ExpectFullFrame(const fs::path &path, const cv::Scalar &color) {
    const auto decoded = cv::imread(path.string(), cv::IMREAD_COLOR);
    ASSERT_FALSE(decoded.empty());
    ASSERT_EQ(decoded.size(), cv::Size(96, 64));
    // A solid source frame makes JPEG's small color error explicit, and checks
    // pixels outside the person crop as well as the target itself.
    const auto difference = cv::norm(decoded, Frame(color), cv::NORM_INF);
    EXPECT_LE(difference, 3.0);
  }

  std::string suffix_;
  fs::path root_;
  rclcpp::Node::SharedPtr node_;
  MissionRosAdapter *adapter_{nullptr};
  std::shared_ptr<TargetImageRosAdapter> image_adapter_;
  std::shared_ptr<EventPhotoRecorder> recorder_;
  std::unique_ptr<MissionTrackingRuntime> runtime_;
};

TEST_F(MissionPhotoEvidenceTest, ConfirmSavesFullFrameOnceAndNewMissionUsesItsOwnFrame) {
  BuildRuntime();
  const cv::Scalar first_color(30, 60, 180);
  const cv::Scalar next_color(180, 60, 30);
  auto first = Frame(first_color);
  Store(100U, MissionStateMsg::PATROL, 0U, kStartStamp);
  runtime_->ProcessFrame({Person()}, Metadata(kStartStamp), first);
  // The encoder must own its source pixels after ProcessFrame returns.
  first.setTo(cv::Scalar(0, 0, 0));
  auto photos = Photos("tracking/event");
  ASSERT_EQ(photos.size(), 1U);
  const auto first_path = photos.front();
  ExpectFullFrame(first_path, first_color);

  // Confirmation is retried after 100 ms while the supervisor has not moved.
  std::this_thread::sleep_for(120ms);
  runtime_->ProcessFrame({Person()}, Metadata(kStartStamp + 120000000ULL), Frame(next_color));
  ASSERT_EQ(Photos("tracking/event").size(), 1U);

  Store(101U, MissionStateMsg::RECOVER_PATROL, 42U, kStartStamp + 200000000ULL);
  runtime_->ProcessFrame({}, Metadata(kStartStamp + 200000000ULL), Frame(next_color));
  Store(102U, MissionStateMsg::PATROL, 0U, kStartStamp + 300000000ULL);
  runtime_->ProcessFrame({Person()}, Metadata(kStartStamp + 300000000ULL), Frame(next_color));
  photos = Photos("tracking/event");
  ASSERT_EQ(photos.size(), 2U);
  const auto next = std::find_if(photos.begin(), photos.end(), [&](const auto &path) {
    return path != first_path;
  });
  ASSERT_NE(next, photos.end());
  // Separate missions within the same source second must retain both photos.
  EXPECT_EQ(next->filename().string(), first_path.stem().string() + "_1.jpg");
  ExpectFullFrame(*next, next_color);
  ExpectFullFrame(first_path, first_color);
}

TEST_F(MissionPhotoEvidenceTest, SixSecondLossSavesLastVisibleFrameOncePerTargetLifecycle) {
  BuildRuntime();
  const cv::Scalar target_color(30, 60, 180);
  Store(200U, MissionStateMsg::VERIFY_IDENTITY, 42U, kStartStamp);
  runtime_->ProcessFrame({Person()}, Metadata(kStartStamp), Frame(target_color));
  runtime_->ProcessFrame({}, Metadata(kStartStamp + 100000000ULL), Frame(cv::Scalar(0, 0, 0)));
  EXPECT_TRUE(Photos("tracking/lost").empty());

  // Exercise the production runtime clock and its default six-second policy.
  std::this_thread::sleep_for(6050ms);
  runtime_->ProcessFrame({}, Metadata(kStartStamp + 6200000000ULL), Frame(cv::Scalar(0, 0, 0)));
  auto photos = Photos("tracking/lost");
  ASSERT_EQ(photos.size(), 1U);
  const auto first_path = photos.front();
  ExpectFullFrame(first_path, target_color);

  runtime_->ProcessFrame({}, Metadata(kStartStamp + 6300000000ULL), Frame(cv::Scalar(0, 0, 0)));
  EXPECT_EQ(Photos("tracking/lost").size(), 1U);

  const cv::Scalar next_color(180, 60, 30);
  Store(201U, MissionStateMsg::RECOVER_PATROL, 42U, kStartStamp + 6400000000ULL);
  runtime_->ProcessFrame({}, Metadata(kStartStamp + 6400000000ULL), Frame(cv::Scalar(0, 0, 0)));
  Store(202U, MissionStateMsg::VERIFY_IDENTITY, 42U, kStartStamp + 6500000000ULL);
  runtime_->ProcessFrame({Person()}, Metadata(kStartStamp + 6500000000ULL), Frame(next_color));
  std::this_thread::sleep_for(6050ms);
  runtime_->ProcessFrame({}, Metadata(kStartStamp + 12700000000ULL), Frame(cv::Scalar(0, 0, 0)));
  photos = Photos("tracking/lost");
  ASSERT_EQ(photos.size(), 2U);
  const auto next = std::find_if(photos.begin(), photos.end(), [&](const auto &path) {
    return path != first_path;
  });
  ASSERT_NE(next, photos.end());
  ExpectFullFrame(*next, next_color);
  ExpectFullFrame(first_path, target_color);
  EXPECT_TRUE(Photos("tracking/event").empty());
}

TEST_F(MissionPhotoEvidenceTest, ReidSuppressedConfirmationDoesNotCreateDetectionPhoto) {
  const std::string service_name = "/mission_photos/" + suffix_ + "/dedup";
  auto service = node_->create_service<DedupSrv>(
      service_name,
      [](const DedupSrv::Request::SharedPtr, DedupSrv::Response::SharedPtr response) {
        response->status = 0;
        response->is_seen = true;
        response->handled = true;
      });
  ReidConfirmGate::Config gate_config;
  gate_config.service_name = service_name;
  gate_config.handled_topic = "/mission_photos/" + suffix_ + "/handled";
  gate_config.retry_interval_sec = 0.01;
  ReidConfirmGate gate(*node_, gate_config);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  const auto image = Frame(cv::Scalar(30, 60, 180));

  // Prime a real ReID verdict, then consume its selection-layer notification
  // so the runtime must itself pass the selected person through the gate.
  bool suppressed = false;
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!suppressed && std::chrono::steady_clock::now() < deadline) {
    gate.Allow(42, image);
    executor.spin_some();
    gate.DrainExclusions([&](int target) { suppressed = target == 42; });
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(suppressed);
  BuildRuntime(&gate);
  Store(300U, MissionStateMsg::PATROL, 0U, kStartStamp);
  const auto result = runtime_->ProcessFrame({Person()}, Metadata(kStartStamp), image);
  ASSERT_EQ(result.primary.primary_target_id, 42);
  EXPECT_TRUE(Photos("tracking/event").empty());
  EXPECT_EQ(recorder_->GetMetrics().enqueued, 0U);
  runtime_.reset();  // Its non-owning gate pointer must not outlive this test's gate.
}

}  // namespace
