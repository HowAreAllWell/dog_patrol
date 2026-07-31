# 跟踪算法设计逻辑与现场调参说明

本文给 Orin 侧现场调试使用，目标是解释当前算法为什么这样设计，以及遇到不同场景时优先调哪些参数。

## 1. 主链路

当前 ROS 2 节点的处理链路：

```text
CameraIngest
  -> PreprocessInfer
  -> DetFilter
  -> MotTracker
  -> IdentityManager
  -> PrimaryTargetManager
  -> MissionRosAdapter / VisualizerRecorder
```

各模块职责：

- `CameraIngest`：通过 Hikrobot MVS SDK 读取相机帧，并向下游提供自持有 BGR8 图像与源帧元数据。
- `PreprocessInfer`：加载 YOLO TensorRT engine，输出原始检测框。
- `DetFilter`：只保留当前业务需要的 `person` 和 `car`，并按类别阈值过滤。
- `MotTracker`：短期多目标跟踪层，维护 raw track。
- `IdentityManager`：语义身份层，把短期 raw track 映射成更稳定的 `semantic_id`。
- `PrimaryTargetManager`：主目标层，只允许 `person` 成为主目标。
- `MissionRosAdapter`：通过 `dog_patrol_interfaces` 发布 mission event 和当前帧 target bbox。
- `VisualizerRecorder`：实时显示或录制带叠字视频。

## 2. 三类 ID 不要混淆

当前算法刻意区分三类 ID：

- `raw_id`：MOT 内部短期轨迹 ID。它可能因为遮挡、丢检、重叠而变化，不对下游承诺稳定。
- `semantic_id`：画面叠字、离线评估、身份层对外使用的稳定身份。现场看到的 `id=` 是这个。
- `primary_target_id`：业务主目标 ID。主目标短时丢失时会保持，不会因为 raw_id 变化立刻切换。

现场判断 ID 稳不稳时，看 `semantic_id` 和 mission bbox/event 中的 `target_id`，不要用 `raw_id` 做业务判断。

## 3. Detector 层

关键参数在 `config/demo_params.yaml`：

- `detector.raw_conf_threshold`
- `detector.person_conf_threshold`
- `detector.car_conf_threshold`
- `detector.input_width`
- `detector.input_height`

逻辑：

- `raw_conf_threshold` 是 detector 原始输出进入后续链路前的最低门槛。
- `person_conf_threshold` / `car_conf_threshold` 是按类别过滤的门槛。
- 当前 tracker 的低分恢复依赖低分检测框，所以这些阈值不能随意调得过高。

调参建议：

- 人经常漏检：先小幅降低 `person_conf_threshold`，例如每次降 `0.02~0.05`。
- 虚影、局部身体、反光经常被当成人：先小幅提高 `person_conf_threshold`，再考虑 birth gate 参数。
- 远处小人需要更早出现：降低阈值会有帮助，但会增加误检和 ID 污染风险。

## 4. MotTracker 短期跟踪层

配置入口：

```text
src/vision_demo_host/config/bot_sort.yaml
```

核心思想：

- 使用 Kalman 运动预测保持短期连续性。
- 用两阶段关联处理高分检测和低分检测。
- 关联成本由 IoU、运动距离、外观特征共同组成。
- GMC 用 `sparseOptFlow` 抵消移动平台或相机抖动带来的全局画面运动。

关键参数：

- `track_high_thresh`：高分检测关联门槛。
- `track_low_thresh`：低分检测参与恢复的门槛。
- `new_track_thresh`：新建 raw track 的检测分数门槛。
- `track_buffer`：raw track 在短时丢失后保留多久。
- `stage1_iou_min` / `stage2_iou_min`：两阶段 IoU 下限。
- `stage1_max_cost` / `stage2_max_cost` / `lost_recovery_max_cost`：最终关联成本门槛。
- `assoc_iou_weight` / `assoc_motion_weight` / `assoc_app_weight`：IoU、运动、外观权重。
- `gmc_method` / `gmc_downscale`：全局运动补偿方式和降采样。

调参建议：

- 相机或机器人运动明显，目标框整体漂移：保留 `gmc_method=sparseOptFlow`，优先调 `gmc_downscale`。
- FPS 不足：先关可视化/录制；仍不足时可临时设 `tracker.gmc_enabled=false` 验证瓶颈。
- 短遮挡后 raw track 频繁断：适当增大 `track_buffer` 或放宽 `lost_recovery_max_cost`，但会增加错绑风险。
- 两个人交叉时 raw track 错换：不要只放宽阈值；优先看 Identity 层是否能把 semantic_id 拉回来。

## 5. Identity 语义身份层

这是当前优化的重点。它的目标不是简单把每个新 raw track 都变成新 ID，而是尽量解释为：

1. 已有目标的 raw 连续。
2. 已有 ACTIVE 语义身份短时 missing 后恢复。
3. 已有身份的重复框、分裂框、局部身体或虚影。
4. 确认可信的新真人。

只有第 4 类才分配新的 `semantic_id`。

### ACTIVE / INACTIVE

- `ACTIVE`：语义身份仍在保留窗口内，即使当前帧没有框，也可能只是遮挡或短时丢检。
- `INACTIVE`：超过 missing 窗口后，身份不再优先绑定，只能用更严格的外观条件恢复。

核心参数：

- `target.lost_threshold_frames`：主目标和 semantic identity 的 missing 保留窗口，默认 `180`。
- `sid.recover_sim_thresh_strict`：INACTIVE 恢复的严格外观相似度阈值。
- `sid.recover_sim_thresh_relaxed`
- `sid.recover_relaxed_max_missing_frames`

现场换 FPS 时要重算帧数。例如 30fps 下 180 帧约 6 秒；25fps 下约 7.2 秒。

### missing assignment

ACTIVE 身份短时消失后重新出现时，Identity 层会综合几何、面积、外观和时间成本。

关键参数：

- `sid.missing_assign_min_area_ratio`
- `sid.missing_assign_max_area_ratio`
- `sid.missing_assign_max_center_dist_norm`
- `sid.missing_assign_max_app_cost`
- `sid.active_assign_max_cost`
- `sid.min_assignment_margin`

调参方向：

- 主目标短遮挡回来却变新 ID：适当放宽 `sid.missing_assign_max_app_cost` 或 `sid.min_assignment_margin`，必要时增加 `target.lost_threshold_frames`。
- 新人被错绑到刚消失的旧人：收紧 `sid.missing_assign_max_app_cost`，或提高 `sid.missing_assign_min_area_ratio`。
- 远处小目标短丢后无法恢复：不要直接大幅放宽所有门限，优先小幅放宽面积下限或 missing app cost。

### birth gate / hidden candidate

当前算法不会把所有新 raw track 立刻显示成新 ID。可疑目标会成为 hidden candidate：

- 反光或虚影。
- 手、身体局部、底部碎片。
- 另一个 active identity 的重复检测框。
- 一个人被 YOLO 切成两个框后的分裂框。

hidden candidate 的规则：

- 不显示。
- 不占用 `semantic_id` 编号。
- 不参与 primary target。
- 如果后续连续稳定、像真实新人，可晋升为正式 `semantic_id`。

相关参数：

- `sid.overlap_iou_freeze`
- `sid.split_stable_frames`
- `sid.merge_hold_frames`
- `sid.stable_frames_before_feature_update`
- `target.min_person_area_px`

调参方向：

- 虚影/手/碎片拿到了正式 ID：提高 `target.min_person_area_px`，收紧 birth 判断相关面积/重叠参数，或提高 detector person 阈值。
- 真实新人出现太慢：适当降低 `target.min_person_area_px`，缩短稳定帧要求，但要复查误检是否变多。
- 两人重叠后 ID 污染：提高 feature freeze 保护，避免遮挡期间把错误外观写进身份库。

## 6. ReID / appearance

当前有两类外观特征：

- `light`：默认轻量 HSV 特征，不需要模型，速度稳定。
- `osnet_onnx`：包内 ReID ONNX，外观表达更强，但当前通过 OpenCV DNN CPU 后端跑，可能降 FPS。

相关参数：

- `tracker.reid_backend`
- `tracker.reid_model_path`
- `tracker.reid_input_width`
- `tracker.reid_input_height`
- `sid.reid_backend`
- `sid.reid_model_path`
- `sid.reid_input_width`
- `sid.reid_input_height`

启用 ONNX ReID 示例：

```bash
REID_MODEL="/path/to/my_workplace/vision_demo_ws/assets/models/reid/osnet_x1_0_market1501_256x128.onnx"

ros2 run vision_demo_host vision_demo_node --ros-args \
  -p tracker.reid_backend:=osnet_onnx \
  -p tracker.reid_model_path:="$REID_MODEL" \
  -p sid.reid_backend:=osnet_onnx \
  -p sid.reid_model_path:="$REID_MODEL"
```

建议：

- 首次跑通先用默认 `light`。
- 多人频繁交叉、衣着差异明显、FPS 有余量时，再试 ONNX ReID。
- 光照变化强、夜间噪声大时，ONNX ReID 不一定比 light 稳，必须看现场视频。

## 7. PrimaryTarget 主目标层

主目标策略：

- 只有 `person` 能成为主目标，`car` 不参与。
- 初次锁定时选最大有效人框。
- 锁定后 continuity-first，不因为 raw_id 变化立刻切换。
- 短时丢失进入 `OCCLUDED`，继续等待同一语义目标恢复。
- 超过 `target.lost_threshold_frames` 后进入 `LOST`，才允许重选。

相关参数：

- `target.lost_threshold_frames`
- `target.min_person_area_px`
- `target.max_center_jump_norm`
- `target.min_area_ratio`
- `target.max_area_ratio`
- `target.pending_recovery_frames`

调参方向：

- 主目标离画面几秒回来应保持同 ID：增大 `target.lost_threshold_frames`。
- 主目标已离开但系统迟迟不换人：减小 `target.lost_threshold_frames`。
- 主目标被旁边大框抢走：检查 Identity 层是否把旧目标恢复成功，再调 primary 的面积/中心跳变约束。

## 8. 现场调参流程

推荐顺序：

1. 固定一个可复现场景，录制 20~60 秒视频。
2. 只改一组参数，记录改动原因。
3. 先看视频叠字 ID，再看 mission event/bbox 中的 `target_id`。
4. 再看 FPS 和资源占用。
5. 通过后把参数写回 `config/demo_params.yaml` 或单独保存现场配置。

不要同时改 detector、tracker、identity、primary 四层，否则无法判断收益和副作用来自哪里。

## 9. 常见场景对照

| 场景 | 优先看 | 可能调整 |
| --- | --- | --- |
| 相机运动导致框关联失败 | GMC / tracker | `tracker.gmc_enabled`, `gmc_downscale`, `lost_recovery_max_cost` |
| 远处小人短丢后换 ID | Identity missing | `sid.missing_assign_min_area_ratio`, `sid.missing_assign_max_app_cost`, `sid.min_assignment_margin` |
| 新人误绑旧人 | Identity missing | 收紧 `sid.missing_assign_max_app_cost`，提高面积下限 |
| 反光/手/碎片占 ID | birth gate | 提高 `target.min_person_area_px`，提高 detector 阈值，收紧 split/hidden 相关参数 |
| 两人交叉后身份错乱 | Identity + feature freeze | `sid.overlap_iou_freeze`, `sid.split_stable_frames`, `sid.merge_hold_frames` |
| 主目标短时离开后 LOST | Primary | 增大 `target.lost_threshold_frames` |
| FPS 不足 | Runtime/GMC/ReID | 关闭可视化录制，调 GMC，避免 ONNX ReID |

## 10. 最小验收标准

现场跑通不能只看节点不崩溃，至少要确认：

- MVS 相机能稳定读帧。
- TensorRT engine 能加载，画面有人时能检测。
- 视频叠字里的 `id=` 在短遮挡和交叉时不频繁跳。
- `/mission/event` 和 `/perception/selected_target_bbox` 能被外部节点观测。
- 主目标 `target_id` 在短时遮挡中保持，事件状态可从 `TARGET_CONFIRMED` 到 `TARGET_LOST` 再回 `TARGET_REACQUIRED`。
