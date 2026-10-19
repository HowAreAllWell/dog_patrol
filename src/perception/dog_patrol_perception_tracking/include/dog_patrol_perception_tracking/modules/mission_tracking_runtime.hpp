#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "dog_patrol_perception_tracking/modules/mission_ros_adapter.hpp"
#include "dog_patrol_perception_tracking/modules/event_photo_recorder.hpp"
#include "dog_patrol_perception_tracking/modules/reid_confirm_gate.hpp"
#include "dog_patrol_perception_tracking/modules/target_image_ros_adapter.hpp"
#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

// 帧处理运行时的公共抽象（从 perception_tracking_node.cpp 拆出，
// 评审 N06/T01：MissionTrackingRuntime 需要在 OFF 配置下可链接、可回归）。
struct RuntimeFrameOutput {
  PrimaryTargetResult primary;
  std::string primary_decision_reason;
  std::string primary_reject_reason;
};

class TrackingRuntime {
 public:
  virtual ~TrackingRuntime() = default;
  virtual const char *Name() const = 0;
  virtual bool FailFastOnInitializationError() const = 0;
  virtual PrimaryTargetResult CurrentPrimary() const = 0;
  virtual std::optional<MissionSnapshot> CurrentMission() const { return std::nullopt; }
  virtual RuntimeFrameOutput ProcessFrame(
      const std::vector<IdentityObservation> &identities,
      const SourceFrameMetadata &source_metadata,
      const cv::Mat &frame) = 0;
};

class DetectionTrackingStatusSink {
 public:
  virtual ~DetectionTrackingStatusSink() = default;
  virtual void Report(DetectionTrackingReadiness::RuntimeStatus status) = 0;
  virtual void Publish() = 0;
};

class ObservationLifecycle {
 public:
  virtual ~ObservationLifecycle() = default;
  virtual void BeforeFrame() = 0;
  virtual bool Current() const = 0;
  virtual TargetImageRosAdapter::Metrics Metrics() const = 0;
};

// 任务模式帧处理：任务状态/选人/确认事件经 MissionRosAdapter，confirm 前
// 接 ReID 去重闸门，handled 结论转发 ReID 节点（R04/R05/R06-A + N01-N06）。
class MissionTrackingRuntime final : public TrackingRuntime,
                                     public DetectionTrackingStatusSink,
                                     public ObservationLifecycle {
 public:
  explicit MissionTrackingRuntime(
      std::unique_ptr<MissionRosAdapter> adapter,
      std::shared_ptr<TargetImageRosAdapter> image_adapter,
      PrimaryTargetCropConfig crop_config,
      ReidConfirmGate *reid_gate = nullptr,
      std::shared_ptr<EventPhotoRecorder> photo_recorder = nullptr);

  const char *Name() const override { return "mission"; }
  bool FailFastOnInitializationError() const override { return false; }
  void Report(DetectionTrackingReadiness::RuntimeStatus status) override;
  void Publish() override;
  void BeforeFrame() override;
  bool Current() const override;
  TargetImageRosAdapter::Metrics Metrics() const override;
  PrimaryTargetResult CurrentPrimary() const override;
  std::optional<MissionSnapshot> CurrentMission() const override;
  RuntimeFrameOutput ProcessFrame(
      const std::vector<IdentityObservation> &identities,
      const SourceFrameMetadata &source_metadata,
      const cv::Mat &source_image) override;

 private:
  // 当前帧内目标 semantic_id 的人体裁剪（带 padding、钳到图像边界）。
  // 找不到/越界返回空图，ReidConfirmGate 对空图直接放行。
  static cv::Mat FindPersonCrop(
      int target_id,
      const std::vector<IdentityObservation> &identities,
      const cv::Mat &source_image, float padding_ratio);

  std::unique_ptr<MissionRosAdapter> adapter_;
  std::shared_ptr<TargetImageRosAdapter> image_adapter_;
  PrimaryTargetCropConfig crop_config_;
  ReidConfirmGate *reid_gate_{nullptr};
  std::shared_ptr<EventPhotoRecorder> photo_recorder_;
  // handled 转发与 reset 检测的帧间状态（仅帧处理串行路径访问）
  int prev_snapshot_handled_{0};  // N06：上一观察快照的 handled_target_id
  int reprocessing_target_{0};    // N06：观察到的"正在处置中"目标
  bool saw_active_mission_{false};
};

}  // namespace dog_patrol_perception_tracking
