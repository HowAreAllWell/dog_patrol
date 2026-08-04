#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "dog_patrol_perception_tracking/modules/primary_target_observer.hpp"

namespace {

using dog_patrol_perception_tracking::ClassId;
using dog_patrol_perception_tracking::IdentityObservation;
using dog_patrol_perception_tracking::IdentityState;
using dog_patrol_perception_tracking::PrimaryState;
using dog_patrol_perception_tracking::PrimaryTargetManager;
using dog_patrol_perception_tracking::PrimaryTargetObserver;
using dog_patrol_perception_tracking::SourceFrameMetadata;

IdentityObservation TrustedPerson(const cv::Rect2f &bbox = cv::Rect2f{1.0F, 1.0F, 2.0F, 2.0F}) {
  IdentityObservation identity;
  identity.semantic_id = 42;
  identity.state = IdentityState::kActive;
  identity.supporting_raw_track_id = 7;
  identity.class_id = ClassId::kPerson;
  identity.confidence = 0.92F;
  identity.bbox = bbox;
  identity.visible = true;
  return identity;
}

SourceFrameMetadata Metadata(const cv::Mat &frame) {
  SourceFrameMetadata source;
  source.source_timestamp_ns = 1710000000000000000ULL;
  source.camera_frame_number = 123U;
  source.camera_frame_number_available = true;
  source.image_width = frame.cols;
  source.image_height = frame.rows;
  source.optical_frame_id = "hik_camera_optical_frame";
  return source;
}

PrimaryTargetObserver Observer() {
  PrimaryTargetManager::Config config;
  config.min_person_area_px = 1.0F;
  return PrimaryTargetObserver(config);
}

TEST(PrimaryTargetObserverTest, ReturnsCurrentSemanticTargetWithOwnedImageAndSourceMetadata) {
  auto observer = Observer();
  cv::Mat frame(4, 5, CV_8UC3, cv::Scalar{10, 20, 30});
  const auto source = Metadata(frame);

  auto output = observer.Update({TrustedPerson(cv::Rect2f{-1.0F, 1.0F, 4.0F, 3.0F})}, source, frame);

  EXPECT_EQ(output.primary.state, PrimaryState::kLocked);
  ASSERT_TRUE(output.observation.has_value());
  EXPECT_EQ(output.observation->target_id, 42);
  EXPECT_EQ(output.observation->source.source_timestamp_ns, source.source_timestamp_ns);
  EXPECT_EQ(output.observation->source.camera_frame_number, 123U);
  EXPECT_EQ(output.observation->bbox, cv::Rect(0, 1, 3, 3));
  EXPECT_FLOAT_EQ(output.observation->confidence, 0.92F);
  ASSERT_EQ(output.observation->target_image.size(), cv::Size(3, 3));
  EXPECT_EQ(output.observation->target_image.at<cv::Vec3b>(0, 0), cv::Vec3b(10, 20, 30));

  frame.setTo(cv::Scalar{99, 99, 99});
  EXPECT_EQ(output.observation->target_image.at<cv::Vec3b>(0, 0), cv::Vec3b(10, 20, 30));
}

TEST(PrimaryTargetObserverTest, DoesNotReturnHistoricalObservationWhenCurrentTargetIsMissing) {
  auto observer = Observer();
  cv::Mat frame(4, 5, CV_8UC3, cv::Scalar{10, 20, 30});
  const auto source = Metadata(frame);
  ASSERT_TRUE(observer.Update({TrustedPerson()}, source, frame).observation.has_value());

  const auto missing = observer.Update({}, source, frame);

  EXPECT_EQ(missing.primary.state, PrimaryState::kOccluded);
  EXPECT_FALSE(missing.observation.has_value());
}

TEST(PrimaryTargetObserverTest, RejectsUnrepresentableCurrentFrameInsteadOfFabricatingObservation) {
  auto observer = Observer();
  cv::Mat frame(4, 5, CV_8UC3, cv::Scalar{10, 20, 30});
  auto source = Metadata(frame);
  source.image_width = 99;

  const auto mismatched = observer.Update({TrustedPerson()}, source, frame);

  EXPECT_EQ(mismatched.primary.state, PrimaryState::kLocked);
  EXPECT_FALSE(mismatched.observation.has_value());
}

TEST(PrimaryTargetObserverTest, SuspiciousCurrentEvidenceCannotProduceObservation) {
  auto observer = Observer();
  cv::Mat frame(4, 5, CV_8UC3, cv::Scalar{10, 20, 30});
  auto person = TrustedPerson();
  person.low_score_update = true;

  const auto output = observer.Update({person}, Metadata(frame), frame);

  EXPECT_EQ(output.primary.state, PrimaryState::kLocked);
  EXPECT_FALSE(output.observation.has_value());
}

}  // namespace
