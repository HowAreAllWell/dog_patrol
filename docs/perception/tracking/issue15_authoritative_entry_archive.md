# Issue #15 权威入口切换与旧仓研究治理

## 结论和边界

`HowAreAllWell/dog_patrol` 是 tracking 正式开发、构建、测试和部署的唯一权威仓库。
旧私有仓库 `HowAreAllWell/vision_demo_ws` 保留迁移前历史、紧急回退依据和非生产研究空间；只有
`deploy/dog_patrol-integration` 由 GitHub active ruleset 冻结，`main`、`dev` 和按需创建的
`research/*` 均保持可写。主仓的构建、测试、standalone 或部署入口均不依赖旧仓 checkout、overlay、
研究分支或资产目录，研究成果必须重新通过主仓 PR、CI 和评审进入正式实现。

本次切换只完成 tracking 的仓库入口收口。face、voice 尚未接入生产实现，导航实现尚未迁入且
最终导航 Orin 的整机验收仍待现场完成。#15 关闭时没有自行关闭父 Spec #3；父 Spec 后续独立验收见
[`issue3_spec_acceptance.md`](issue3_spec_acceptance.md)。

## 权威入口

- 稳定项目状态和普通构建测试：[`../../../README.md`](../../../README.md)
- 感知模块职责：[`../../../src/perception/README.md`](../../../src/perception/README.md)
- Orin 平台、资产、full-runtime 构建、环境门禁和启动：
  [`../../../src/perception/requirements.md`](../../../src/perception/requirements.md)
- tracking standalone、参数和诊断：
  [`../../../src/perception/dog_patrol_perception_tracking/README.md`](../../../src/perception/dog_patrol_perception_tracking/README.md)
- 当前 Orin/Hik 验收：[`issue14_tracking_hardware_acceptance.md`](issue14_tracking_hardware_acceptance.md)
- 多许可证范围：[`../../../LICENSES/README.md`](../../../LICENSES/README.md)

## 追溯和回退锚点

- 旧仓迁移冻结分支：`deploy/dog_patrol-integration`。
- tracking 导入时原始 tip：`7878d70e6d86ad2a283911f8719345171b1c1d2a`；原始 tree：
  `4bc50d674e322f7cd6d38f8c1860cbb6571ffe54`。
- 旧仓首次归档告示提交：冻结分支 `4f3df1505a05e8b4bd3d1498cec22caa5b769c10`，默认分支
  `599bdfca1bf1f32602a2dbafbac6b19aa83cffe6`；允许非生产研究后的治理告示分别为
  `6673baa` 和 `d58c59e`。
- 主仓过滤历史 tip：`6faaed42bf0531239b0203885607a9ff318eedc7`；完整的 116 个相关
  过滤提交固定在 annotated tag `tracking-import/vision-demo-ws-7878d70`。过滤规则、路径映射和
  SHA 改写原因见 [`import_provenance.md`](import_provenance.md)。

普通追溯优先使用主仓 tag，不需要旧工作区：

```bash
git log --follow tracking-import/vision-demo-ws-7878d70 -- \
  src/perception/dog_patrol_perception_tracking/<path>
```

若主仓发生无法通过正常 revert 修复的迁移级问题，从上述冻结分支或原始 tip 建立只读对照 clone。
回退时只提取经审查的源码/文档提交并重新走主仓 PR；旧仓 `main`、`dev` 或 `research/*` 均不得
直接成为生产入口，也不得导入模型、TensorRT engine、录像、日志、相机序列号、凭据或其他本机资产。

## 历史归档与当前研究治理

旧仓最初只有在以下证据全部成立后才设为 archived：

- 旧仓默认分支和冻结分支的 README 均已推送上述权威入口告示，且本地冻结分支与远端同步；
- 主仓 CODEOWNERS、许可证和来源锚点已复核；
- 从当前分支的干净 clone 按根 README 完成 portable 五包构建和全量测试；
- 当前 Orin 的 full-runtime build/test、统一环境检查、standalone/Hik/真人/crop/反压验收已有
  #14 证据，`requirements.md` 提供从构建到启动的连续入口；
- #15 的主仓 PR 已通过 CI 并合入 `main`，主仓 `main` 与 `origin/main` 同步且干净；
- face、voice 和导航整机验收继续明确保留为后续，父 Spec #3 由独立整体验收决定状态。

该归档曾作为权威入口切换的强写保护，但不是权威性的必要条件。用户随后明确允许旧仓继续承担
非生产研究，因此仓库已取消归档，并建立 `Freeze deployment baseline` branch ruleset：只匹配
`deploy/dog_patrol-integration`，以 `update`、`deletion` 和 `non_fast_forward` 规则禁止更新、删除
和强推，且没有 bypass actor。`main`、`dev` 和 `research/*` 不匹配该 ruleset，可以正常更新。

## 最终验证

- 主仓 PR #25 和 follow-up PR #26 均通过 CI 并 squash merge；执行归档门禁时的 merged-main
  验收 SHA 为 `c60ea3e028fd4abbeb83e21e14360d7041834385`。
- 从远端最终 `main` 全新 clone 后，portable 五包构建通过，测试汇总为 399 tests、0 errors、
  0 failures、0 skipped，clone 工作树保持干净。
- `vision_demo_ws` 当前 `isArchived=false`；默认分支治理告示为 `d58c59e`，冻结部署分支治理告示为
  `6673baa`；ruleset `Freeze deployment baseline` 为 `active` 且 `current_user_can_bypass=never`。
- 旧仓本地停留在同步且干净的 `deploy/dog_patrol-integration`；ignored 的 MVS 日志、模型、
  engine、ReID 和录像/数据目录仍在，未删除或纳入 Git。
- face、voice 和导航整机验收继续保留为后续；它们未被父 Spec #3 的独立验收标记为完成。
