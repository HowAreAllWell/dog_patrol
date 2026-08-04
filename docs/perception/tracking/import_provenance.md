# Tracking 历史导入来源

`dog_patrol_perception_tracking` 的实现历史来自
`HowAreAllWell/vision_demo_ws` 的 `deploy/dog_patrol-integration` 分支。导入时的来源 tip 为
`7878d70e6d86ad2a283911f8719345171b1c1d2a`，tree 为
`4bc50d674e322f7cd6d38f8c1860cbb6571ffe54`；它包含 #5 portable build 和 #6 正式改名。

为满足公开主仓边界，导入使用 `docs/issue4_tracking_baseline_audit.md` 的白名单过滤历史：

- package 映射到 `src/perception/dog_patrol_perception_tracking/`；
- 稳定 tracking 文档映射到 `docs/perception/tracking/`；
- 构建产物、日志、数据、模型、录像、个人工具和旧仓管理文件没有导入；
- 退役的 bearing、UDP、RTSP benchmark、`record_test_set` 和 legacy identity 路径从导入历史移除；
- 私网端点、RTSP URL 和个人绝对路径在过滤后的历史中统一匿名化。

过滤后来源 tip 为 `6faaed42bf0531239b0203885607a9ff318eedc7`。主仓通过 merge parent
保留这 116 个非空相关提交；`git log --follow --
src/perception/dog_patrol_perception_tracking/<path>` 可以追溯迁移前的实现演进。由于路径和内容
过滤会重写对象标识，过滤前后的提交 SHA 不相同，原始 tip 和 tree 作为来源锚点保留在本文。

源码 package 在来源仓即声明 Apache-2.0；主仓继续保持该声明，并在 `LICENSES/` 提供
许可证全文和组件范围。导入后为路径、默认配置和仓库集成所作修改属于主仓修改。
