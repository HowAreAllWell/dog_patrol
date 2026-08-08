# 人脸算法接入协作规则

本文是 `dog_patrol_perception_face` 的迁入和联调门禁。目标是让人脸算法独立演进，同时不改变
tracking 的主目标选择、实时推理、现有预览、录制和 mission 行为。若算法源仓的实现与本文冲突，
先在本仓 PR 中拆分或适配，不得直接绕过现有边界。

## 1. 代码与所有权边界

- 人脸生产代码、配置模板、launch、测试和部署检查只放在本 package；ROS package 名称保持
  `dog_patrol_perception_face`。
- 人脸包可以依赖 `dog_patrol_interfaces` 和 `dog_patrol_perception_interfaces`，不得 include、import、
  链接或复制 `dog_patrol_perception_tracking` 的私有类、内部头文件和实现源码。
- 不得修改 tracking 的 detector、tracker、semantic identity、主目标选择、相机接入或 mission 事件
  行为来迁就人脸算法。若现有公共消息确实不足，先提交最小合同变更并由 perception owner 评审。
- 外部算法迁入必须记录来源仓、固定提交、许可证、允许迁入清单和排除项；不得把外部仓作为构建或
  运行依赖。

## 2. 主目标图像输入合同

- 生产人脸算法唯一图像输入是 `/perception/tracked_target_image` 上的
  `dog_patrol_perception_interfaces/msg/TrackedTargetImage`。不得再次打开 Hik 相机、订阅另一条原始
  相机流、从 tracking 内存取帧，或在人脸侧重新检测并选择业务主目标。
- 订阅 QoS 必须匹配生产发布端：keep-last 1、best-effort、volatile。消费者应处理丢帧，只处理最新
  可用图像，不能假设每一帧都会到达或要求发布端重传。
- `crop_data` 按 `encoding`、`crop_width`、`crop_height` 和 `crop_step` 解码。当前生产值是 `bgr8`，
  接入代码仍须校验编码、尺寸、步长、数据长度、正数 `target_id`、bbox 和原图尺寸；非法消息丢弃并
  记录有界诊断，不得导致节点退出。
- `target_id` 是 tracking 分配的语义主目标 ID。`source_stamp`、可用时的
  `source_frame_number`、`source_frame_id` 和 bbox 用于来源追溯与结果关联，不得用人脸库内部 ID
  替换 `target_id`。
- tracking 只在当前帧存在可信、可见的 `LOCKED` 主目标时发布 crop；离场或失效通过停发和后续
  mission 状态体现，没有“空图像”作为取消信号。人脸消费者必须配置新鲜度上限，超时后清空待处理
  图像和未提交结果，不能继续使用最后一张脸。
- 接收回调只做校验、所有权转移和有界入队。推理在独立 worker 执行；队满丢旧保新，算法变慢、
  加载失败或退出都不得对 tracking 形成反压。

## 3. Mission 会话与授权结果

- provider 必须同时跟随 `/mission/state`。仅在未 blocked 的 `VERIFY_IDENTITY` 中处理与当前
  `MissionState.target_id` 相同的新鲜 crop，并把结果绑定到当前 `state_seq + target_id`。
- 状态离开 `VERIFY_IDENTITY`、`state_seq` 或 `target_id` 改变、任务 blocked、目标图像过期或节点
  shutdown 时，必须取消当前任务并清空队列。旧 worker 即使随后返回也不得发布迟到结果。
- 结果通过现有 `/perception/authorization_evidence` 发布
  `dog_patrol_perception_interfaces/msg/AuthorizationEvidence`，`provider` 固定为 `face`；不得由人脸
  节点直接发布 mission `AUTHORIZED`、`UNAUTHORIZED` 或 `EXECUTION_ERROR` 事件。
- readiness 与 evidence 分离。只有模型、受控白名单、运行时和必要设备均通过真实 preflight，且
  `observed_startup_state_seq` 匹配当前 STARTUP 时，生产 readiness 节点才可发布 `face` READY。
  测试 fake provider 不能安装到生产入口，也不能作为 readiness 验收证据。
- 日志和 `detail` 不得包含姓名、原始人脸图像、特征向量、白名单内容或其他生物识别隐私数据。

## 4. 唯一预览与 overlay 接入

- tracking 的 `VisualizerRecorder` 是 live 模式唯一正式预览、overlay canvas 和诊断录制所有者。
  人脸生产节点不得调用 `cv::imshow`、`cv::waitKey` 或创建窗口，不得启动第二个 GUI 事件循环、
  第二套录制器或第二个相机 reader。
- 外部人脸仓若自带预览，迁入时必须拆分推理与显示：移除生产路径中的窗口、相机和录制生命周期；
  只保留可表达为结果数据的 face bbox、匹配状态、置信度和必要诊断字段。
- 人脸 overlay 通过可选 adapter 汇入现有 `VisualizerRecorder`。adapter 只能接收轻量结果数据，按
  `target_id + source_stamp`（可用时再校验 `source_frame_number`）关联；目标不匹配、超过新鲜度上限
  或来自旧 mission 会话的结果不得绘制。
- overlay adapter、结果缓存和绘制必须在现有有界诊断 worker 边界内工作。未启动人脸节点、没有
  结果、结果过期或 adapter 异常时，现有 tracking overlay、preview、record 和推理必须保持原行为。
- 预览继续由现有 `visualization.enable`、统一 tracking launch 和同一个窗口控制。不得新增另一套
  face preview 开关作为生产入口。人脸 overlay 默认可缺省，关闭预览时不得创建 GUI 副作用。
- 若轻量 overlay 结果需要新增 ROS 消息，先用单独 PR 说明字段、QoS、失效语义、隐私边界和测试，
  放入 `dog_patrol_perception_interfaces`；不得让 tracking 依赖人脸包私有类型。

## 5. 资产、配置与隐私

- 模型、白名单、人脸图片、embedding/特征向量、现场录像、凭据和设备专用配置不进入 Git；通过
  部署机受控路径提供。
- 仓库只保存无隐私的配置模板、模型格式/校验要求和测试 fixture。测试图像必须具有明确授权和来源，
  否则使用合成数据。
- 默认不得保存输入 crop。诊断落盘必须显式开启、限定受控目录和保留期，并经过隐私评审；不得写入
  tracking 的 clean capture dataset。

## 6. PR 与验收门禁

- 通过短分支和 PR 迁入，至少由 `.github/CODEOWNERS` 中的 perception owner 审查。PR 必须说明
  外部来源固定提交、许可证、依赖、模型格式、资源预算、失败语义和回退方式。
- 单元测试至少覆盖消息校验、latest-only 有界队列、图像过期、目标切换、mission 取消、旧结果拒绝、
  readiness 失败和隐私字段清理。
- ROS 集成测试至少覆盖 crop 到 evidence 的 `state_seq + target_id` 绑定，以及人脸 worker 变慢或退出
  时 tracking 不反压。
- 预览验收必须证明：只打开一个相机和一个窗口；face overlay 在同一 tracking canvas；错目标、旧帧、
  旧会话不显示；关闭/杀死人脸节点不影响 tracking preview/record；关闭 preview 不产生 GUI。
- Orin 验收记录功能结果、处理延迟、输入/推理/结果丢弃数、CPU/GPU/RAM 和温度。性能门槛须由项目
  owner 明确确认，不能在迁入时自行降低 tracking 的既有行为或验收标准。
- 在上述实现和验收完成前，本 package 只能标记为 scaffold/not-integrated，不得声称生产人脸能力
  已接入，也不得发布伪造 READY。
