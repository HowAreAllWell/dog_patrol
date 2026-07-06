# 当前 tracking identity 实现状态

日期：2026-07-05

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

当前真实状态：`IdentityManager` 不是完全重写后的 identity 层，但它已经是公开身份接口和当前 runtime state owner。内部 `IdentityAssignmentEngineAdapter` 只作为 assignment / recovery / update engine adapter 在 `IdentityManager::Impl` 持有的 state 上运行。

当前做的事情：

- 接收 `TrackletObservation` 和可选 `TrackletHypothesis`；
- 转回 `Track` 后调用 `IdentityAssignmentEngineAdapter::Update`；
- 从内部 snapshot 生成 `IdentityObservation`；
- 暴露 `IdentityManagerResult`；
- 转换 adapter score debug row 为 `IdentityAssignmentEvidence`；
- 暴露当前 identity mode 和 feature freeze 状态。
- `FeatureUpdatePolicy` 已作为 identity 层可单测决策 helper 抽取，assignment engine adapter 负责把当前状态适配为显式 policy 输入后委托该 helper。
- `FeatureGeometryUpdateState` 已作为 identity 层可单测 mutation helper 抽取，assignment engine adapter 的 identity record 内嵌该 helper 使用的 feature bank / reliable geometry 子状态，并在 upsert 时直接委托 helper mutation。
- `FeatureBankCost` 已作为 identity 层可单测 read/cost helper 抽取，assignment engine adapter 在 scoring 时把现有 feature bank 传入该 helper 计算 appearance cost。
- `ReliableGeometryCost` 已作为 identity 层可单测 read/prediction/cost helper 抽取，assignment engine adapter 在 scoring 和 missing gate 时把现有 reliable geometry state 传入该 helper。
- `AssignmentCost` 已作为 identity 层内部可单测 cost composition helper 抽取，assignment engine adapter 在 scoring 时委托它组合 appearance / geometry / time / weighted final cost。
- `ActiveAssignmentSolver` 已作为 identity 层内部可单测 active assignment solving helper 抽取，assignment engine adapter 把已准备好的 active assignment matrix / candidate metadata 交给 helper 执行 Hungarian selection、assignment margin、active max-cost reject、margin reject / cushion 和 2x2 pairwise appearance override 决策。
- `ActiveAssignmentInputCollector` 已作为 identity 层内部可单测 active assignment input collector 抽取，assignment engine adapter 把 unassigned-track / free-semantic-id filtering、per-track feature selection/index adaptation 和 score/gate evidence adaptation 委托给 helper，再把结果交给 `AssignmentCandidateBuilder` 和 `ActiveAssignmentSolver`。
- `InactiveRecoveryInputCollector` 已作为 identity 层内部可单测 inactive recovery input collector 抽取，assignment engine adapter 把 recovery-track filtering、free inactive semantic-id filtering、recoverable identity filtering、feature selection/index adaptation、recover threshold 和 missing identity gate evidence adaptation 委托给 helper，再把结果交给 `InactiveRecoverySolver` 和 `AssignmentCandidateBuilder`。
- `InactiveRecoverySolver` 已作为 identity 层内部可单测 inactive recovery selection helper 抽取，assignment engine adapter 把已准备好的 inactive recovery candidate score / threshold / gate evidence 交给 helper 执行 accepted / rejected candidate classification、best-sim recovery selection、Hungarian recovery selection 和 margin 计算。
- `RawContinuityDecision` 已作为 identity 层内部可单测 raw continuity assignment decision helper 抽取，assignment engine adapter 把已计算的 cost / missing-gate / MOT association evidence 交给 helper 判定 `raw_continuity` accepted / reject reason / margin。
- `BirthCandidateDecision` 已作为 identity 层内部可单测 birth / hidden candidate decision helper 抽取，assignment engine adapter 把已计算的 ambiguous recovery、duplicate split、形态隐藏、Phase 5 flag 和小目标稳定帧 evidence 交给 helper 判定 hidden / pending / allocation action。
- `BirthCandidateStore` 已作为 identity 层内部可单测 birth candidate storage helper 抽取，assignment engine adapter 把小目标新人 per-raw-track consecutive hit / last-seen frame 状态交给 helper 维护。
- `BirthManager` 已作为 identity 层内部可单测 birth / hidden candidate facade 抽取，assignment engine adapter 把 ambiguous recovery、duplicate split、形态隐藏、Phase 5 flag、小目标稳定帧、semantic id allocation call timing、birth debug row 构造和 Phase 5 accepted allocation apply seam 交给该 facade 协调。
- `UnresolvedTrackFinalResolutionCoordinator` 已作为 identity 层内部可单测 final resolution helper 抽取，assignment engine adapter 在 raw continuity、active assignment、inactive recovery 后，把仍未分配的 visible person track 编排交给该 helper，覆盖 occlusion-suspect skip、selected margin-reject ambiguous recovery hold、Phase 4 side recovery 前的 legacy apply suppression、BirthManager input adaptation、pending erase 请求和 accepted birth assignment output。
- `SemanticIdAllocator` 已作为 identity 层内部可单测 semantic id allocation helper 抽取，assignment engine adapter 在新 birth、Phase 5 accepted birth allocation 和 duplicate semantic collision fallback 中把当前 identity storage 的 occupied ids 交给 helper 分配下一个非 primary semantic id。
- `AssignmentCandidateBuilder` 已作为 identity 层内部可单测 assignment candidate/debug helper 抽取，assignment engine adapter 把 active assignment candidate score / gate evidence 和 inactive recovery candidate decisions 交给 helper 生成 `assign_candidate` / `inactive_recover_candidate` rows、solver 输入矩阵和 solver-result row updates。
- `MergedSingleBlobAssignmentDecision` 已作为 identity 层内部可单测 merged single-blob decision helper 抽取，assignment engine adapter 在 `kMerged` + one visible person 路径中继续负责 identity storage、feature/cost/gate/inactive candidate preparation、semantic id allocation 和 raw binding，helper 只决定 active candidate selection、continuity margin、best-appearance handoff eligibility、Phase 4 handoff 前的 continuity preference、inactive recovery fallback 与 new semantic id fallback。
- `AssignmentApplicationPlan` 已作为 identity 层内部可单测 assignment application planning helper 抽取，assignment engine adapter 把已决 assignment candidates、accepted debug rows、当前 frame 和 occupied semantic ids 交给 helper 规划 duplicate semantic collision fallback、assignment cost / margin / stage 和下一版 raw-to-semantic map。
- `AssignmentApplicationExecutor` 已作为 identity 层内部可单测 assignment apply executor 抽取，assignment engine adapter 把 planned applications、accepted/rejected debug rows、identity record store、raw binding store 和 update-policy / upsert seams 交给 helper 执行 accepted mutation、row update-reason 回写、merged/split/overlap 遮挡保护与 planned raw-map replacement。
- `Phase4DirectApplyHelper` 已作为 identity 层内部 Phase 4 direct apply mutation helper 抽取；`IdentityRuntimeMutationApplier` 在 identity runtime/coordinator 层调用该 helper，并接管 accepted Phase 4/5 runtime mutation。`IdentityAssignmentEngineAdapter` 不再公开 Phase 4/5 direct apply methods；single-blob carrier missing side effect、side-recovery force-geometry-update、Phase 5 accepted birth allocation row refresh、raw binding 和 birth cleanup 由 runtime applier 保持。
- `RawSemanticBindingStore` 已作为 identity 层内部可单测 raw-to-semantic binding helper 抽取，assignment engine adapter 把 previous-frame raw snapshot、planned raw map replacement、lookup、Phase 4/5 direct bind 和 reset/clear 委托给该 helper。
- `IdentityRuntimeRecordLifecycle` 已作为 identity 层内部可单测 lifecycle helper 抽取，assignment engine adapter 在 observation apply、seen/missing aging、occlusion protection、single-blob carrier handoff missing 和 snapshot projection 时委托该 helper。
- `IdentityRuntimeStore` 已作为 identity 层内部可单测 record store facade 抽取，assignment engine adapter 把 semantic-id lookup、occupied-id / active-inactive person 枚举、frame-begin reset、age-one-frame、occlusion protection、carrier handoff missing 和 sorted snapshot projection 委托给该 facade。
- `IdentityManager::Impl` 已持有 identity runtime state，集中包含 `IdentityRuntimeStore`、`RawSemanticBindingStore`、`SemanticIdAllocator`、`BirthManager`、frame index、primary/bootstrap state、occlusion mode state 和 debug row buffers；`IdentityAssignmentEngineAdapter` 作为内部 adapter 在该 state 上运行。
- 输出 `phase3_shadow_state.csv` 使用的 shadow-only debug rows，包括 hypothesis input、MergedGroup、SplitCandidate、single-blob handoff decision、pairwise matrix、Phase 4 handoff evidence 和 Phase 5 `NewBirthCandidate` lifecycle evidence。
- Phase 5 BirthManager 已固定为唯一运行时 birth / hidden candidate 路径；hidden / pending / accepted birth decision surface 由 `IdentityManager` Phase 5 helper 表达，accepted birth allocation 通过 legacy state seam 应用，并以 `stage=phase5_birth_candidate` / `stage=phase5_new_semantic` 及 `new_birth_candidate_*` rows 记录。

因此，当前 identity 层接口、一部分 shadow / Phase 4/5 evidence route、update-policy 决策面、feature-bank read / appearance-cost 规则、reliable-geometry read / prediction / cost / gate 规则、assignment cost composition 规则、active assignment input collection 规则、active assignment solving / acceptance 规则、inactive recovery input collection 规则、inactive recovery selection 规则、raw continuity assignment decision 规则、birth / hidden candidate decision 规则、birth candidate storage / stability counter 规则、birth / hidden candidate facade 协调规则、unresolved-track final resolution 编排规则、semantic id allocation counter / skip-existing-id 规则、assignment candidate/debug 构造规则、assignment application planning 规则、assignment apply mutation orchestration 规则、Phase 4/5 accepted runtime mutation mechanics、raw-to-semantic binding storage 规则、identity runtime store facade 规则、runtime mutable state holder、feature bank / reliable geometry mutation 规则、identity runtime record 类型边界，以及 record lifecycle mutation 规则已经迁移。底层 runtime state ownership 已 hoist 到 `IdentityManager::Impl`，`IdentityAssignmentEngineAdapter` 已收缩为内部 assignment/update adapter，primary/output 仍保持当前公开边界。

### 2.3 `IdentityAssignmentEngineAdapter`

当前定位：内部 assignment/update engine adapter。

仍包含的适配职责：

- raw track 到 semantic id 的绑定编排与 public mapping surface 适配；
- primary semantic id 管理适配；
- active / inactive recovery；
- birth apply 之后的 runtime state projection 适配；
- 通过 `IdentityManager::Impl` 持有的 `IdentityRuntimeStore` 访问 identity record storage，包括 feature bank / reliable geometry 子状态；
- `FeatureUpdatePolicy` 输入适配、`FeatureBankCost` feature-bank read 委托、`ReliableGeometryCost` reliable-geometry read 委托、`AssignmentCost` cost composition 委托、`ActiveAssignmentInputCollector` active assignment input collection 委托、`ActiveAssignmentSolver` active assignment solving 委托、`InactiveRecoverySolver` inactive recovery selection 委托、`RawContinuityDecision` raw-continuity decision 委托、`BirthCandidateDecision` birth / hidden candidate decision 委托、`BirthCandidateStore` birth candidate storage / stability counter 委托、`BirthManager` birth / hidden candidate facade 协调委托、`UnresolvedTrackFinalResolutionCoordinator` unresolved-track final resolution 编排委托、`SemanticIdAllocator` non-primary semantic id allocation 委托、`AssignmentCandidateBuilder` assignment candidate/debug 委托、`MergedSingleBlobAssignmentDecision` merged single-blob assignment decision 委托、`AssignmentApplicationPlan` assignment application planning 委托、`AssignmentApplicationExecutor` assignment apply mutation orchestration 委托、`IdentityRuntimeMutationApplier` Phase 4/5 accepted runtime mutation 委托、`Phase4DirectApplyHelper` Phase 4 direct apply mutation 委托、`RawSemanticBindingStore` raw binding storage 委托、`IdentityRuntimeStore` record lookup / enumeration / lifecycle / snapshot 委托、`FeatureGeometryUpdateState` feature/reliable-geometry mutation 委托与 `IdentityRuntimeRecordLifecycle` record lifecycle mutation 委托；
- merged / split recovery mode；
- birth / hidden candidate gate；
- feature update freeze；
- score debug rows。
- 指向 `IdentityManager::Impl` 所拥有 `RuntimeState` 的内部 adapter seam；该 state 集中管理 identity store、raw binding、semantic id allocator、BirthManager、frame/primary/occlusion/debug mutable state。

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
- 被拒绝的可见主目标会转为 `PENDING_RECOVERY`，保留当前 `primary_target_id`，但不输出 `primary_track`；
- 当前 primary identity 处于 `MERGED` 或 `SPLIT_RECOVERY` 时也会转为 `PENDING_RECOVERY`；
- 普通缺失仍转为 `OCCLUDED`，而不是立即释放 primary；
- identity 进入 lost / inactive 后会释放 primary。

注意：`PENDING_RECOVERY` 已是公开 `PrimaryState` / UDP `track_state` 值；当前只覆盖可见 primary sanity rejection 以及 identity `MERGED` / `SPLIT_RECOVERY` 的窄范围语义，不表示完整 pending recovery 状态机已经迁移完成。

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
3. 将原 semantic id 逻辑收缩为内部 `IdentityAssignmentEngineAdapter`。
4. 新增 `IdentityManager` 作为身份层接口。
5. `PrimaryTargetManager` 改为消费 `IdentityObservation`。
6. UDP 和可视化接入 identity 状态。
7. 增加 identity / primary / UDP 相关单元测试。
8. 建立 `tracklet_hypotheses.csv` 和 `phase3_shadow_state.csv`，用于观察 tracker hidden / suppressed candidate、MergedGroup 和 SplitCandidate lifecycle。
9. 已将 Phase 4 四类 handoff 行为固定为唯一运行时路径：`phase4_merged_split_handoff`、`phase4_merged_side_recovery`、`phase4_merged_single_blob_handoff` 和 `phase4_pairwise_assignment`。对应旧切换参数和 legacy fallback 分支已移除。
10. 已补充 Phase 5 readiness 文档，固定 birth / hidden candidate 的现有证据面与下一步建议。
11. 已加入 Phase 5 `NewBirthCandidate` shadow-only lifecycle rows：`new_birth_candidate_pending`、`new_birth_candidate_hidden`、`new_birth_candidate_allocated`，用于观察 legacy birth / hidden / allocation 决策。
12. Phase 5 BirthManager 已固定为唯一运行时 birth / hidden candidate 路径，hidden / pending / accepted birth decision surface 由 `IdentityManager` Phase 5 path 表达。
13. 已将 Phase 6 update-policy decision calculation 抽取为 `FeatureUpdatePolicy` helper；`IdentityAssignmentEngineAdapter` 继续负责可靠观测判定、overlap/global freeze 输入适配和 rollback/debug 对照。
14. 已将 Phase 6 feature bank / reliable geometry update-state mutation 规则抽取为 `FeatureGeometryUpdateState` helper；`IdentityManager::Impl` 持有的 runtime state 内保留 identity runtime record，record 内已嵌入 helper 使用的 feature bank / reliable geometry 子状态，adapter 在 upsert 时委托 helper mutation。
15. 已将 Phase 6 feature-bank read / appearance-cost 规则抽取为 `FeatureBankCost` helper。
16. 已将 Phase 6 reliable-geometry read / prediction / geometry-cost / missing-gate 规则抽取为 `ReliableGeometryCost` helper；runtime state 内的 identity runtime record 保留 reliable geometry storage，adapter 把嵌入子状态适配给 helper。
17. 已将 Phase 6 feature bank / reliable geometry scattered fields 收敛为 identity runtime record 内的 `FeatureGeometryUpdateState::State` 子状态；本步只改变 record 内部存储形状，未迁移完整 identity record ownership。
18. 已将 Phase 6 identity record shape 从旧私有嵌套结构抽为 identity 层内部 `IdentityRuntimeRecord` 类型；record storage ownership 已位于 `IdentityManager::Impl` 持有的 runtime state。
19. 已将 Phase 6 non-assignment identity record lifecycle mutation 抽取为内部 `IdentityRuntimeRecordLifecycle` helper，覆盖 observation apply、frame begin seen reset、unseen aging、occlusion protection、single-blob carrier handoff missing 和 snapshot projection；adapter 只编排 assignment / birth / raw mapping 适配。
20. 已将 Phase 6 assignment cost composition 抽取为内部 `AssignmentCost` helper，覆盖 appearance、geometry、time 和 weighted final score 组合；`IdentityAssignmentEngineAdapter` 仍持有 candidate collection、active assignment solving、assignment acceptance/rejection、pairwise override、missing gates、assignment apply 和 raw-to-semantic mapping ownership。
21. 已将 Phase 6 active assignment solving 抽取为内部 `ActiveAssignmentSolver` helper，覆盖 Hungarian selection、assignment margins、active max-cost rejection、assignment margin rejection / cushion 和 2x2 pairwise appearance override selection；`IdentityAssignmentEngineAdapter` 仍持有 candidate collection、score/debug row aggregation、raw-to-semantic mapping、semantic-id allocation call sites、assignment application、birth decisions、Phase 4/5 apply paths 和 identity record storage ownership。
22. 已将 Phase 6 inactive recovery selection 抽取为内部 `InactiveRecoverySolver` helper，覆盖 inactive candidate accepted / rejected classification、recover threshold、recovery max-cost rejection、single-track best-sim recovery selection、Hungarian recovery selection 和 margin reporting；`IdentityAssignmentEngineAdapter` 仍持有 inactive identity storage、candidate collection、score/debug row aggregation、recovery application、raw-to-semantic mapping、semantic-id allocation call sites、birth decisions 和 Phase 4/5 apply paths。
23. 已将 Phase 6 raw continuity assignment decision 抽取为内部 `RawContinuityDecision` helper，覆盖 accepted、`identity_not_found`、`missing_identity_gate_reject`、`raw_continuity_max_cost_reject`、`weak_mot_association` 和 margin / selected / accepted 语义；`IdentityAssignmentEngineAdapter` 仍持有 identity storage、cost/gate evidence 适配、score/debug row aggregation、raw-to-semantic mapping、assignment apply、semantic-id allocation call sites、birth decisions 和 Phase 4/5 apply paths。
24. 已将 Phase 6 birth / hidden candidate decision 抽取为内部 `BirthCandidateDecision` helper，覆盖 `ambiguous_recovery_pending`、`duplicate_split_hidden`、`skinny_partial_hidden`、`wide_fragment_hidden`、legacy 小目标稳定等待/晋升、Phase 5 pending 和允许分配；adapter 仍编排 score/debug row aggregation、raw-to-semantic map application 和 public behavior 兼容。
25. 已将 Phase 6 semantic id allocation counter / skip-existing-id 规则抽取为内部 `SemanticIdAllocator` helper，覆盖从 `2` 开始、跳过 `1`、跳过当前 identity storage 已占用 semantic id、单调前进和无 allocation call 不消耗 ID；`IdentityAssignmentEngineAdapter` 仍负责决定何时在 birth / Phase 5 allocation / duplicate collision fallback 中调用分配，以及 primary bootstrap 的 semantic id `1` 行为。
26. 已将 Phase 6 birth candidate storage / small-new-person stability counter 抽取为内部 `BirthCandidateStore` helper，覆盖 per-raw-track consecutive hit increment、gap reset、raw-id independent counter、erase 和 clear。
27. 已将 Phase 6 birth / hidden candidate facade 抽取为内部 `BirthManager` helper，覆盖 ambiguous recovery / duplicate split / morphology hide / small-new-person stability 的 decision input adaptation、pending store 擦除时机、semantic id allocation call timing、birth debug row 构造，以及 `phase5_birth_candidate -> phase5_new_semantic` accepted allocation apply seam；`IdentityAssignmentEngineAdapter` 仍持有 birth apply 后的 identity storage / primary / output ownership 和 rollback 对照面。
28. 已将 Phase 6 assignment application planning 抽取为内部 `AssignmentApplicationPlan` helper，覆盖已决 assignment candidates 的 duplicate semantic collision fallback、accepted debug-row lookup、assignment cost / margin / stage 推导和 next raw-to-semantic map construction；`IdentityAssignmentEngineAdapter` 仍负责 update-policy evaluation、`UpsertIdentity`、occlusion protection、row reason defaults、`AgeAndPruneIdentities` 和实际 state mutation。
29. 已将 Phase 6 raw-to-semantic binding storage 抽取为内部 `RawSemanticBindingStore` helper，覆盖 previous snapshot、planned replacement、lookup、direct bind/erase/reset 和保持当前 raw-map 插入口径；`IdentityAssignmentEngineAdapter` 仍负责决定何时在 `Update`、Phase 4 handoff、pairwise assignment 和 Phase 5 accepted birth allocation 路径调用该 helper。
30. 已将 Phase 6 identity runtime record store facade 抽取为内部 `IdentityRuntimeStore` helper，覆盖 semantic-id lookup、occupied-id 枚举、active/inactive person semantic id 枚举、frame-begin seen reset、age-one-frame、occlusion protection、single-blob carrier handoff missing 和 sorted snapshot projection；adapter 不再直接协调这些 record container 操作，只保留 candidate collection、score/debug row aggregation 和 assignment application 编排。
31. 已将 Phase 6 assignment candidate/debug 构造抽取为内部 `AssignmentCandidateBuilder` helper，覆盖 active assignment candidate score/gate evidence 到 `assign_candidate` rows 与 solver matrix 的适配、inactive recovery candidate decision 到 `inactive_recover_candidate` rows 的适配，以及 solver result selected / margin / accepted / pending row 回写；`IdentityAssignmentEngineAdapter` 仍持有 pairwise debug rows、实际 assignment application、raw-to-semantic mapping、semantic-id allocation call sites、birth decisions、Phase 4/5 apply paths 和 identity record storage ownership。
32. 已将 Phase 6 active assignment input collection 抽取为内部 `ActiveAssignmentInputCollector` helper，覆盖 unassigned-track filtering、free semantic id filtering、feature row/index adaptation、missing identity / appearance gate evidence 与 accepted/pairwise-pending row compatibility 所需输入准备；`IdentityAssignmentEngineAdapter` 仍持有 score truth helpers、solver invocation、pairwise debug rows、assignment application、raw-to-semantic mapping、birth decisions、Phase 4/5 apply paths 和 identity record storage ownership。
33. 已将 Phase 6 assignment application mutation orchestration 抽取为内部 `AssignmentApplicationExecutor` helper，覆盖按既有顺序迭代 planned applications、accepted row 的 update-policy reason 回写、feature / reliable-geometry mutation、merged/split/overlap 遮挡保护、rejected row default reason 填充，以及 planned raw-to-semantic replacement；`IdentityAssignmentEngineAdapter` 通过 `EvaluateUpdatePolicy` / `UpsertIdentity` seams 提供现有行为，仍持有 birth decisions、Phase 4/5 direct apply call sites 与 birth apply 后的 identity storage / primary / output ownership。
34. 已将 Phase 6 merged single-blob assignment decision 抽取为内部 `MergedSingleBlobAssignmentDecision` helper，覆盖 continuity kept、best final-cost selection、close continuity candidate margin、best-appearance handoff eligibility、Phase 4 handoff 前的 continuity preference、inactive recovery fallback 和 new semantic id fallback；`IdentityAssignmentEngineAdapter` 仍持有 identity storage、feature extraction、cost/gate 适配、inactive recovery candidate preparation、semantic id allocation、raw binding、Phase 4 apply methods、primary/output/CSV/UDP/overlay ownership。
35. 已将 Phase 6 inactive recovery input collection 抽取为内部 `InactiveRecoveryInputCollector` helper，覆盖 recovery-track filtering、free inactive semantic-id filtering、`CanRecoverInactiveIdentity` 过滤、strict / relaxed recover threshold 适配、missing identity gate evidence 和 accepted / selected row compatibility 所需输入准备；`IdentityAssignmentEngineAdapter` 仍持有 score truth helpers、solver invocation、recovery application、raw-to-semantic mapping、birth decisions、Phase 4/5 apply paths 和 identity record storage ownership。
36. 已将 Phase 6 unresolved-track final resolution 编排抽取为内部 `UnresolvedTrackFinalResolutionCoordinator` helper，覆盖 active / inactive assignment 后仍未分配 track 的 occlusion-suspect skip、ambiguous recovery pending、Phase 4 side recovery 前的 legacy apply suppression、duplicate split / morphology hide、小目标 pending、Phase 5 pending 和 accepted birth assignment output；adapter 仍负责 score truth callbacks、BirthManager / SemanticIdAllocator call sites、debug row frame/mode projection、runtime-state mutation 适配和 raw binding 编排。
37. 已将 Phase 4 direct identity apply mutation mechanics 抽取为内部 `Phase4DirectApplyHelper`，覆盖 raw track lookup、target semantic id validation、feature extraction、app/geo/time/final cost、accepted Phase 4 score debug row、update policy、upsert、raw binding 和 accepted raw birth cleanup。
38. 已将 accepted Phase 4/5 runtime mutation ownership 迁到 `IdentityRuntimeMutationApplier` / coordinator 层；`IdentityAssignmentEngineAdapter` 不再公开 `ApplyPhase4*` 或 `ApplyPhase5BirthAllocation` methods。`Phase4HandoffCoordinator` 仍在原时机应用 pairwise、merged single-blob、side recovery 和 merged split actions；Phase 5 accepted birth allocation 仍刷新 raw-to-semantic binding、score rows 和 `new_birth_candidate_*` lifecycle rows。

## 4. 尚未完成或需要特别注意的点

1. `IdentityManager` 仍使用内部 `IdentityAssignmentEngineAdapter`，不是最终 identity state machine。
2. 当前 identity state 枚举仍是 `ACTIVE / OCCLUDED / INACTIVE / LOST / MERGED / SPLIT_RECOVERY`，与设计文档中的目标状态机不完全一致。
3. 合并、拆分、遮挡生命周期仍保留 legacy 对照口径，但 Phase 3/4 已有 shadow rows 和 default-on migrated handoff evidence。
4. feature update policy 决策面、feature-bank read/cost helper、reliable-geometry read/cost helper、assignment cost composition helper、active assignment input/solving helper、inactive recovery input/selection helper、raw-continuity decision helper、birth / hidden candidate decision helper、birth candidate storage helper、birth facade helper、unresolved-track final resolution helper、semantic id allocation helper、assignment application planning helper、assignment apply executor helper、Phase 4/5 accepted runtime mutation helper、raw-to-semantic binding storage helper、feature bank / reliable geometry mutation helper、identity runtime record 类型边界和 record lifecycle mutation helper 已独立，runtime record 内也已有 feature bank / reliable geometry 子状态；但长期特征管理、feature bank ownership 和 reliable geometry ownership 仍未迁移成目标状态机。
5. `IdentityAssignmentEvidence` 已输出，但 Primary 当前主要使用 track / association / bbox sanity 信息，并未完整消费 identity assignment confidence。
6. `PENDING_RECOVERY` 已作为 public primary / UDP 状态接入，但当前只覆盖 visible primary sanity rejection 和 identity `MERGED` / `SPLIT_RECOVERY`；`pending_recovery_frames` 仍不是完整 pending recovery 状态机。
7. 配置中仍存在 legacy / diagnostics 对照配置，后续需要明确保留、归档或删除。
8. 当前测试通过不等于算法效果达标；离线评估和视频复盘仍是必要输入。
9. Phase 5 birth / hidden candidate 已有独立 `BirthManager` facade、`NewBirthCandidate` lifecycle evidence 和默认开启的 birth migration flag。当前默认路径由 `IdentityManager` Phase 5 helper 表达 hidden / pending / accepted decision surface；legacy gate 和 score rows 仍保留为显式 rollback / 对照。

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
2. 明确下一阶段只改哪一层：MOT、IdentityManager、IdentityAssignmentEngineAdapter 迁移，还是 Primary。
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
2. 在已抽取的 `FeatureUpdatePolicy` / `FeatureBankCost` / `ReliableGeometryCost` / `AssignmentCost` / `ActiveAssignmentSolver` / `InactiveRecoverySolver` / `RawContinuityDecision` / `BirthCandidateDecision` / `BirthCandidateStore` / `FeatureGeometryUpdateState` / `IdentityRuntimeRecordLifecycle` seam、`IdentityRuntimeRecord` 类型边界和 runtime record 内嵌子状态基础上继续小步迁移 feature bank / reliable geometry / assignment / birth ownership；
3. 再逐步把 legacy 内部状态迁移到目标 `IdentityManager` 状态机；
4. 不在同一轮同时大改 MOT、Identity 和 Primary。
