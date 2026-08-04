#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "dog_patrol_perception_tracking/modules/yolo26_output_contract.hpp"

TEST(Yolo26OutputContractTest, AcceptsExpectedFinalDetectionShape) {
  dog_patrol_perception_tracking::Yolo26OutputContract contract;
  std::string error;

  ASSERT_TRUE(dog_patrol_perception_tracking::ValidateYolo26OutputShape({1, 300, 6}, &contract, &error)) << error;
  EXPECT_EQ(contract.batch, 1U);
  EXPECT_EQ(contract.detection_count, 300U);
  EXPECT_EQ(contract.fields_per_detection, 6U);
}

TEST(Yolo26OutputContractTest, AcceptsAndRecordsNonStandardDetectionCount) {
  dog_patrol_perception_tracking::Yolo26OutputContract contract;
  std::string error;

  ASSERT_TRUE(dog_patrol_perception_tracking::ValidateYolo26OutputShape({1, 8400, 6}, &contract, &error)) << error;
  EXPECT_EQ(contract.detection_count, 8400U);
}

TEST(Yolo26OutputContractTest, RejectsWrongRankWithContractMessage) {
  std::string error;

  EXPECT_FALSE(dog_patrol_perception_tracking::ValidateYolo26OutputShape({300, 6}, nullptr, &error));
  EXPECT_NE(error.find("Unexpected YOLO26 TensorRT output rank"), std::string::npos);
  EXPECT_NE(error.find("[1, 300, 6]"), std::string::npos);
}

TEST(Yolo26OutputContractTest, RejectsWrongLastDimensionWithFieldMessage) {
  std::string error;

  EXPECT_FALSE(dog_patrol_perception_tracking::ValidateYolo26OutputShape({1, 300, 85}, nullptr, &error));
  EXPECT_NE(error.find("expected 6 fields per detection"), std::string::npos);
  EXPECT_NE(error.find("x1,y1,x2,y2,conf,cls"), std::string::npos);
}

TEST(Yolo26OutputContractTest, RejectsUnexpectedBatchDimension) {
  std::string error;

  EXPECT_FALSE(dog_patrol_perception_tracking::ValidateYolo26OutputShape({2, 300, 6}, nullptr, &error));
  EXPECT_NE(error.find("expected batch 1"), std::string::npos);
}

TEST(Yolo26OutputContractTest, RejectsDynamicOrInvalidDetectionCount) {
  std::string error;

  EXPECT_FALSE(dog_patrol_perception_tracking::ValidateYolo26OutputShape({1, -1, 6}, nullptr, &error));
  EXPECT_NE(error.find("concrete positive dimensions"), std::string::npos);
}
