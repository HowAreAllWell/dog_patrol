# 当前 tracking identity 实现状态

日期：2026-07-03

本文记录当前代码的真实实现状态，用于接手和排查 identity 相关问题。它不是新的架构设计文档；`orin_hik_h264_MOT` 中 01/02 已修问题和后续架构风险见 `docs/orin_hik_h264_MOT_01_02_issue_resolution.md`。Phase 5 birth / hidden candidate readiness 证据见 `docs/phase5_birth_hidden_candidate_readiness.md`。

## 1. 当前主链路

当前运行主链路在 `src/vision_demo_host/src/main.cpp` 中组织，大致为：

```text
camera_ingest
  -> preprocess_infer
  -> det_filter
  -> mot_tracker
  -> identity_manager
  -> primary_target_manager
  -> bearing_estimator / udp_json_adapter / visualizer_recorder
```

关键调用关系：

```text
tracks = MotTracker::Update(filtered_detections, frame)
tracklet_hypotheses = MotTracker::LastTrackletHypotheses()
identity_result = IdentityManager::Update(TrackletObservationsFromTracks(tracks), tracklet_hypotheses, previous_primary, frame)
primary = PrimaryTargetManager::Update(identity_result.identities)
bearing = BearingEstimator::Estimate(primary.primary_track, ...)
UdpJsonAdapter::Send(primary, bearing, identity_result)
VisualizerRecorder::Render(frame, tracks, primary, identity_result)
```

`offline_eval_recordings` 也复用主链模块进行离线回放评估，并输出 CSV、summary 和可选叠加视频。

## 2. 当前模块职责

### 2.1 `MotTracker`

当前定位：短期 tracker / tracklet 层。

已具备：

- Kalman 预测与更新；
- GMC；
- ByteTrack / BoT-SORT 风格高低分阶段关联；
- Hungarian 匹配；
- stage final cost gate；
- appearance 特征；
- lost recovery / low-score update 标记；
- occlusion suspect / protection；
- `AssociationEvidence` 输出。
- `TrackletHypothesis` shadow 输出，包含 tracked、suppressed duplicate candidate 和 duplicate output hidden 等候选事实。

输出到上层的核心结构：

- `Track`
- `TrackletObservation`
- `AssociationEvidence`
- `TrackletHypothesis`

注意：`MotTracker` 仍存在 `old_minimal` 与 `new_core` 两类路径，当前默认目标是 `new_core`。后续不应再把长期 semantic identity 逻辑塞回 `MotTracker`。

### 2.2 `IdentityManager`

当前定位：身份层公开接口。

当前真实状态：`IdentityManager` 不是完全重写后的 identity 层，它现在主要是 `LegacyIdentityMatcher` 的适配壳。

当前做的事情：

- 接收 `TrackletObservation` 和可选 `TrackletHypothesis`；
- 转回 `Track` 后调用 `LegacyIdentityMatcher::Update`；
- 从 legacy snapshot 生成 `IdentityObservation`；
- 暴露 `IdentityManagerResult`；
- 转换 legacy score debug row 为 `IdentityAssignmentEvidence`；
- 暴露当前 identity mode 和 feature freeze 状态。
- 输出 `phase3_shadow_state.csv` 使用的 shadow-only debug rows，包括 hypothesis input、MergedGroup、SplitCandidate、single-blob handoff decision、pairwise matrix、Phase 4 handoff evidence 和 Phase 5 `NewBirthCandidate` lifecycle evidence。
- 提供默认关闭的 `sid.enable_phase5_birth_manager` 迁移 flag；显式启用时，accepted birth allocation 由 `IdentityManager` Phase 5 path 调用 legacy state seam 应用，并以 `stage=phase5_new_semantic` / `reason=phase5_birth_manager_allocated` 记录。

因此，当前 identity 层接口和一部分 shadow / Phase 4 evidence route 已经迁移，但核心 semantic id birth、hidden candidate、feature bank 更新等逻辑仍主要在 legacy matcher 内部。

### 2.3 `LegacyIdentityMatcher`

当前定位：隔离后的旧 semantic id 匹配逻辑。

仍包含：

- raw track 到 semantic id 的绑定；
- primary semantic id 管理；
- active / inactive recovery；
- appearance / geometry / time cost；
- assignment max cost / margin；
- feature bank；
- reliable geometry；
- merged / split recovery mode；
- birth / hidden candidate gate；
- feature update freeze；
- score debug rows。

该模块是后续重构要逐步替换或拆解的重点。它不应被视为目标架构完成态。

### 2.4 `PrimaryTargetManager`

当前定位：主目标策略层。

当前输入：`std::vector<IdentityObservation>`。

当前行为要点：

- 主目标绑定 semantic id，而不是裸 raw track id；
- 可见主目标会经过 sanity check；
- 可拒绝明显异常的可见主目标，例如：
  - 非 person；
  - bbox 面积过小；
  - unconfirmed；
  - occlusion suspect；
  - association final gate 未通过；
  - low-score update；
  - just recovered；
  - 中心跳变过大；
  - 面积比例异常；
- 被拒绝的可见主目标会转为 `OCCLUDED`，而不是立即锁定；
- identity 进入 lost / inactive 后会释放 primary。

注意：当前 `pending_recovery_frames` 配置存在，但没有形成独立的对外 `PENDING_RECOVERY` 状态。

### 2.5 输出与可视化

`UdpJsonAdapter` 当前输出包括：

- `target_id`：业务主目标 id / semantic id 口径；
- `track_state`：primary state；
- `identity_state`；
- `identity_missing_frames`；
- `identity_supporting_raw_track_id`；
- bbox 和 bearing 字段。

`VisualizerRecorder` 当前叠加显示：

- semantic id；
- identity state；
- supporting raw id；
- MOT association stage / cost；
- primary id；
- identity feature freeze 状态。

## 3. 已完成的重构内容

当前已经完成的主要迁移：

1. 增加 `AssociationEvidence`，让 MOT 层能把关联质量传给上层。
2. 增加 `TrackletObservation`、`IdentityObservation`、`IdentityManagerResult` 等跨层数据结构。
3. 将原 semantic id 逻辑隔离为 `LegacyIdentityMatcher`。
4. 新增 `IdentityManager` 作为身份层接口。
5. `PrimaryTargetManager` 改为消费 `IdentityObservation`。
6. UDP 和可视化接入 identity 状态。
7. 增加 identity / primary / UDP 相关单元测试。
8. 建立 `tracklet_hypotheses.csv` 和 `phase3_shadow_state.csv`，用于观察 tracker hidden / suppressed candidate、MergedGroup 和 SplitCandidate lifecycle。
9. 已将 Phase 4 四类 handoff 行为迁移到默认开启路径，同时保留显式 `false` rollback：
   - `sid.enable_phase4_merged_split_handoff`
   - `sid.enable_phase4_merged_side_recovery`
   - `sid.enable_phase4_merged_single_blob_handoff`
   - `sid.enable_phase4_pairwise_assignment`
10. 已补充 Phase 5 readiness 文档，固定 birth / hidden candidate 的现有证据面与下一步建议。
11. 已加入 Phase 5 `NewBirthCandidate` shadow-only lifecycle rows：`new_birth_candidate_pending`、`new_birth_candidate_hidden`、`new_birth_candidate_allocated`，用于观察 legacy birth / hidden / allocation 决策。
12. 已新增默认关闭的 Phase 5 birth migration flag：`sid.enable_phase5_birth_manager`。flag-off 回退 legacy `new_semantic` allocation；flag-on 迁移 accepted birth allocation 到 `IdentityManager` Phase 5 path，同时保留 legacy score/debug rows 对照。

## 4. 尚未完成或需要特别注意的点

1. `IdentityManager` 仍包装 `LegacyIdentityMatcher`，不是最终 identity state machine。
2. 当前 identity state 枚举仍是 `ACTIVE / OCCLUDED / INACTIVE / LOST / MERGED / SPLIT_RECOVERY`，与设计文档中的目标状态机不完全一致。
3. 合并、拆分、遮挡生命周期仍保留 legacy 对照口径，但 Phase 3/4 已有 shadow rows 和 default-on migrated handoff evidence。
4. feature bank 更新虽然已有更保守的 gate，但长期特征管理仍主要在 legacy matcher 内完成。
5. `IdentityAssignmentEvidence` 已输出，但 Primary 当前主要使用 track / association / bbox sanity 信息，并未完整消费 identity assignment confidence。
6. `pending_recovery_frames` 当前不是完整 pending recovery 状态机。
7. 配置中仍存在 legacy / diagnostics 对照配置，后续需要明确保留、归档或删除。
8. 当前测试通过不等于算法效果达标；离线评估和视频复盘仍是必要输入。
9. Phase 5 birth / hidden candidate 已有 shadow-only `NewBirthCandidate` lifecycle evidence 和默认关闭的 accepted allocation 迁移 flag，但还没有独立 `BirthManager`。当前 hidden/pending 判断仍保留 legacy gate 作为对照；flag-on 时 accepted allocation 通过 `IdentityManager` Phase 5 path 应用。

## 5. 当前效果判断口径

当前固定离线数据集为 `data/datasets/orin_hik_h264_MOT`，默认评估输出在 `data/eval_results/`。后续不能只看 primary switch 或 `LOCKED` ratio，还需要结合：

- primary locked ratio；
- occluded ratio；
- lost ratio；
- semantic id 是否稳定；
- raw id 切换时 semantic id 是否保持；
- visible primary 被拒绝的原因；
- feature update 是否在遮挡/合并/低质量观测中被冻结；
- hidden / suppressed candidate reason 是否符合 `tracklet_hypotheses.csv`、`phase3_shadow_state.csv` 和 `sid_scores.csv` 的证据矩阵；
- 叠加视频中的具体错误帧。

## 6. 后续继续修改前的建议检查清单

在继续改算法前，建议先完成：

1. 确认当前分支是 `dev`，并保留本轮修改前的导出结果或 commit 作为对比依据。
2. 明确下一阶段只改哪一层：MOT、IdentityManager、LegacyIdentityMatcher 迁移，还是 Primary。
3. 固定离线评估数据集和关键帧段。
4. 对典型失败片段做帧级复盘，区分错误来源：
   - MOT 误关联；
   - identity 错绑；
   - primary sanity check 过严或过松；
   - detector / low-score 预过滤；
   - feature bank 污染；
   - 合并/拆分状态判断错误。
5. 每次算法修改后至少运行相关单测，并按固定数据集做离线对比。

## 7. 推荐下一步

优先做小步修改和小步重构：

1. 保持 `IdentityManager` 作为外部边界不变；
2. 先验收 `sid.enable_phase5_birth_manager` flag-on/off 在固定离线数据上的等价性；
3. 再逐步把 legacy 内部状态迁移到目标 `IdentityManager` 状态机；
4. 不在同一轮同时大改 MOT、Identity 和 Primary。
