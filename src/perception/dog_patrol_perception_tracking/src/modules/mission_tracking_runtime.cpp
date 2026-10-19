#include "dog_patrol_perception_tracking/modules/mission_tracking_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "dog_patrol_perception_tracking/modules/primary_target_observer.hpp"

namespace dog_patrol_perception_tracking {

MissionTrackingRuntime::MissionTrackingRuntime(
    std::unique_ptr<MissionRosAdapter> adapter,
    std::shared_ptr<TargetImageRosAdapter> image_adapter,
    PrimaryTargetCropConfig crop_config,
    ReidConfirmGate *reid_gate,
    std::shared_ptr<EventPhotoRecorder> photo_recorder)
    : adapter_(std::move(adapter)), image_adapter_(std::move(image_adapter)),
      crop_config_(crop_config), reid_gate_(reid_gate),
      photo_recorder_(std::move(photo_recorder)) {}

void MissionTrackingRuntime::Report(
    DetectionTrackingReadiness::RuntimeStatus status) {
  adapter_->ReportDetectionTrackingRuntimeStatus(std::move(status));
}

void MissionTrackingRuntime::Publish() { adapter_->PublishCapabilityStatus(); }

void MissionTrackingRuntime::BeforeFrame() { image_adapter_->Consume(std::nullopt); }

bool MissionTrackingRuntime::Current() const {
  return image_adapter_->HasCurrentObservation();
}

TargetImageRosAdapter::Metrics MissionTrackingRuntime::Metrics() const {
  return image_adapter_->GetMetrics();
}

PrimaryTargetResult MissionTrackingRuntime::CurrentPrimary() const {
  return adapter_->CurrentPrimary();
}

std::optional<MissionSnapshot> MissionTrackingRuntime::CurrentMission() const {
  return adapter_->CurrentMission();
}

RuntimeFrameOutput MissionTrackingRuntime::ProcessFrame(
    const std::vector<IdentityObservation> &identities,
    const SourceFrameMetadata &source_metadata,
    const cv::Mat &source_image) {
  if (reid_gate_ != nullptr && reid_gate_->enabled()) {
    // N03：跨天同步放在每帧固定入口，不依赖 confirm 事件触发 Allow——
    // 跨天时选人层的 ReID 排除项一并失效。
    if (reid_gate_->SyncDay()) {
      adapter_->ClearReidDedupExclusions();
    }
    // R04：取走异步到达的"seen+handled"排除结论，注入选人资格边界——
    // 被去重者不再占用 primary，同屏其他新人可被选择
    reid_gate_->DrainExclusions([this](int semantic_id) {
      adapter_->ExcludeForReidDedup(semantic_id);
    });
    // R06-A/N06：把 supervisor 的 handled 结论转发 ReID 节点落库。转发按
    // "新的权威处置结果"去重，而非仅按 ID 是否变化：
    //   ① 转换检测：handled_target_id 相对上一观察快照发生变化（0→X 即新
    //      完成；同值 heartbeat 重发、RECOVER→PATROL 携带、跨天保留的旧
    //      快照均不触发）；
    //   ② 重武装：观察到 X 处于活动处置阶段（CONFIRM/APPROACH/VERIFY/
    //      TRACK，此阶段 handled≠target）后，X 的结论再次出现时允许同 ID
    //      再次转发——即使跳帧漏掉中间的 handled≠X 快照。
    // 重进 STARTUP（任务 reset/新会话）时清除 handled 标记与本地转发上下文。
    const auto mission = adapter_->CurrentMission();
    if (mission.has_value()) {
      if (mission->phase == MissionPhase::kStartup) {
        if (saw_active_mission_) {
          saw_active_mission_ = false;
          prev_snapshot_handled_ = 0;
          reprocessing_target_ = 0;
          reid_gate_->NotifyHandledReset();
        }
      } else {
        saw_active_mission_ = true;
        const int handled = mission->handled_target_id;
        const bool active_target_phase =
            mission->phase == MissionPhase::kConfirmTarget ||
            mission->phase == MissionPhase::kApproachTarget ||
            mission->phase == MissionPhase::kVerifyIdentity ||
            mission->phase == MissionPhase::kTrackIntruder;
        if (active_target_phase && mission->target_id > 0 &&
            handled != mission->target_id) {
          reprocessing_target_ = mission->target_id;
        }
        const bool new_result =
            handled > 0 &&
            (handled != prev_snapshot_handled_ || handled == reprocessing_target_);
        if (new_result) {
          if (handled == reprocessing_target_) {
            reprocessing_target_ = 0;
          }
          reid_gate_->NotifyHandled(handled);
        }
        prev_snapshot_handled_ = handled;
      }
    }
  }
  std::function<bool(int)> confirm_gate;
  if (reid_gate_ != nullptr && reid_gate_->enabled()) {
    confirm_gate = [this, &identities, &source_image](int target_id) {
      return reid_gate_->Allow(
          target_id,
          FindPersonCrop(target_id, identities, source_image,
                         crop_config_.padding_ratio));
    };
  }
  auto frame = adapter_->ProcessFrame(
      identities, MissionCoordinator::Clock::now(),
      source_metadata, confirm_gate ? &confirm_gate : nullptr);
  auto observation = BuildPrimaryTargetObservation(
      frame.primary, source_metadata, source_image, crop_config_);
  if (photo_recorder_) {
    photo_recorder_->ObserveFrame(frame.mission,
                                 observation ? observation->target_id : 0,
                                 frame.events, source_metadata, source_image,
                                 frame.mission_stamp_ns);
  }
  image_adapter_->Consume(std::move(observation));
  return {std::move(frame.primary), std::move(frame.primary_decision_reason),
          std::move(frame.primary_reject_reason)};
}

cv::Mat MissionTrackingRuntime::FindPersonCrop(
    int target_id,
    const std::vector<IdentityObservation> &identities,
    const cv::Mat &source_image, float padding_ratio) {
  for (const auto &identity : identities) {
    if (identity.semantic_id != target_id || !identity.visible ||
        identity.class_id != ClassId::kPerson) {
      continue;
    }
    const double pad_x = static_cast<double>(identity.bbox.width) * padding_ratio;
    const double pad_y = static_cast<double>(identity.bbox.height) * padding_ratio;
    const int x_min = std::max(0, static_cast<int>(std::floor(identity.bbox.x - pad_x)));
    const int y_min = std::max(0, static_cast<int>(std::floor(identity.bbox.y - pad_y)));
    const int x_max =
        std::min(source_image.cols,
                 static_cast<int>(std::ceil(identity.bbox.x + identity.bbox.width + pad_x)));
    const int y_max =
        std::min(source_image.rows,
                 static_cast<int>(std::ceil(identity.bbox.y + identity.bbox.height + pad_y)));
    if (x_max <= x_min || y_max <= y_min) {
      return {};
    }
    return source_image(cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min));
  }
  return {};
}

}  // namespace dog_patrol_perception_tracking
