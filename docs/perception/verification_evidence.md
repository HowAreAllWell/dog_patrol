# 配置项与照片使用说明：检测、跟踪、语音

本文按需求截图中的名称和顺序，逐项对应配置项与照片，并说明如何启动、修改配置、保存配置和查找照片。
我们负责检测、跟踪、语音口令及必要的任务编排；人脸和 ReID 的配置由对应模块负责。

当前默认规则：语音最多播报 **2 次**，每次播报结束后等待 **10 秒**；次数耗尽后停止播报和监听。
开始时单独核验人脸，默认 **3 秒**；未通过后进入联合核验，人脸持续识别，覆盖口令播报、等待和所有后续轮次。
联合核验不设独立的人脸截止时间。人脸或口令任一通过立即结束；口令次数耗尽且仍未通过时统一结束并进入入侵跟踪。
进入入侵跟踪后最多跟踪 **60 秒**；目标连续丢失 **6 秒**判定丢失。当前不增加核验总时长，也不加入中文语音。

## 一、平台可配置项：与截图逐项对应

下表保留截图原名称。截图中的“核验口令”是机器人播报的内容，“正确回答”是人员需要说出的答案。
当前提供 ROS 参数、启动参数和 YAML 配置入口；平台界面及网络配置下发需要平台侧接入这些入口。

| 截图顺序 | 截图配置项 | 负责模块 | 对应节点参数 | 当前默认值 |
|---|---|---|---|---|
| 1 | 纯人脸核验时长 | 人脸 | `runtime/face.yaml` 中的 `initial_face_window_seconds` | `3.0` 秒 |
| 2 | 单轮口令等待时长 | 语音 | `response_timeout_seconds` | `10.0` 秒 |
| 3 | 口令播报次数 | 编排、语音 | `voice_prompt_count` | `2` 次，包含首次 |
| 4 | 核验口令 | 语音 | `first_prompt`、`retry_prompt` | 首次、后续英文提示语，见下文 |
| 5 | 正确回答 | 语音 | `passphrase` | `blue star` |
| 6 | 入侵跟踪时长 | 主控、跟踪 | `intruder_tracking_timeout` | `60.0` 秒 |
| 7 | 目标丢失判定时长 | 跟踪 | `target.lost_event_timeout_sec` | `6.0` 秒 |
| 8 | 重复核验忽略时长 | ReID | 由 ReID 模块提供 | 本次不调整 |

我们负责其中 **6 项业务设置，对应 7 个参数**；“核验口令”拆为首次、后续两段文本。
另外提供照片开关和保存目录，共 **9 个业务参数**。内部消息话题单独列在文末。

### 1. 纯人脸核验时长

对应截图中“开始人脸核验，直至开启口令核验前”的时长，由人脸模块负责。
默认 3 秒；在人脸模块的 `runtime/face.yaml` 设置 `initial_face_window_seconds`，重启人脸节点后生效。
设为 `0` 时回退到该文件的 `window_timeout_seconds`。此时长仅影响纯人脸阶段，不限制联合核验。

### 2. 单轮口令等待时长

参数 `response_timeout_seconds` 属于 `/perception_voice_provider`，默认 `10.0` 秒，必须是大于 0 的有限数值。
从本轮播报结束、实际开始等待回答时计时；每一轮各有自己的等待时间，播报用时不计入这段等待。
正确回答可以提前结束本轮等待。播报、设备准备和恢复可能增加流程的实际总耗时。

可运行时修改，从**下一次核验会话**生效。修改示例见第四节。
这项参数影响每轮何时判定未通过；未通过照片仍取该轮等待开始时的画面。

### 3. 口令播报次数

参数 `voice_prompt_count` 属于 `/perception_authorization`，默认 `2` 次，包含首次播报。
取正整数，例如 `1`、`2`、`3`；接口支持范围为 `1` 至 `4294967295`。
默认情况下，首次未通过后再播报一次；核验通过则结束，不强制用完次数。
次数耗尽且核验仍未通过后，停止播报和监听，进入入侵跟踪。

可运行时修改，从**下一次核验会话**生效。配置为 3 次时，语音照片轮次依次标记为 `r1`、`r2`、`r3`。
该参数不放入语音的 `voice_verification.yaml`，由编排节点单独接收。

### 4. 核验口令

这里指机器人播报的提示文本，属于 `/perception_voice_provider`：

| 参数 | 用途 | 默认英文文本 |
|---|---|---|
| `first_prompt` | 第一次播报 | `Please face my camera directly or say the passphrase loudly.` |
| `retry_prompt` | 第二次及以后每次播报 | `Once again, please face my camera directly or say the passphrase loudly.` |

两项都必须是非空文本，按当前英文语音能力配置。希望每次播报相同内容时，将两项设置成相同文本。
可运行时修改，从**下一次核验会话**生效。

### 5. 正确回答

参数 `passphrase` 属于 `/perception_voice_provider`，默认 `blue star`，是人员需要说出的英文口令。
不能为空。配置答案和识别文本都会转为小写、合并多余空白后按文本完全匹配；例如 `Blue   Star` 与 `blue star` 按同一答案处理。
修改播报文本不会自动修改正确回答，两者分别配置。

可运行时修改，从**下一次核验会话**生效。通过时生成该轮口令通过照片。

### 6. 入侵跟踪时长

参数 `intruder_tracking_timeout` 属于 `/mission_supervisor`，默认 `60.0` 秒，必须是大于 0 的有限数值。
从核验未通过、进入 `TRACK_INTRUDER` 阶段开始计时；到期结束入侵跟踪，恢复原巡检任务。
短暂丢失后重新发现目标不会重置这段计时。

该参数在**主控节点启动时读取**，修改需重启对应节点。
它不包含前面的检测、人脸和语音核验时间，也不是截图边界规则中“从首次检测开始”的核验总时长。
本次没有额外的入侵跟踪超时照片。

### 7. 目标丢失判定时长

节点参数 `target.lost_event_timeout_sec` 属于 `/dog_patrol_perception_tracking_node`，默认 `6.0` 秒，
必须是大于 0 的有限数值。整套感知启动时的参数名为 `lost_event_timeout_sec`。

连续没有当前目标的可信观测达到该时长，判定目标丢失，结束当前目标流程并恢复巡检。
不足该时长重新发现目标时，清零丢失计时并继续当前流程，不重置已开始的入侵跟踪计时。
判定丢失时保存该目标最后一次可信可见的整帧。

该参数在**跟踪节点启动时读取**，修改需重启对应节点。

### 8. 重复核验忽略时长

对应截图中“同一人物核验通过后，在一定时长内再次识别时不重复核验”，由 ReID 负责。
跟踪侧继续向 ReID 转发处理结果，并使用 ReID 返回的排除结论。

## 二、照片保存的补充配置

这两项用于控制我们负责的事件照片，不属于截图中的八项核验业务设置。

| 用途 | 跟踪节点参数 | 整套感知启动参数 | 默认值 | 生效时机 |
|---|---|---|---|---|
| 是否保存照片 | `photos.enabled` | `photos_enabled` | `true` | 节点启动 |
| 照片根目录 | `photos.root` | `photos_root` | `~/.ros/dog_patrol/photos` | 节点启动 |

关闭照片保存不会关闭检测、跟踪或语音核验，也不控制人脸或 ReID 自己的图片功能。
当前照片功能用于任务模式；完整感知启动入口已使用该模式。
根目录不能为空；建议使用绝对路径或 `~/...`。`~` 是运行节点的用户主目录，后台服务运行用户不同，默认保存位置也会不同。
照片目录按需自动创建，运行节点的用户须能写入该路径。

## 三、机器人回传照片：与截图逐项对应

本节沿用截图的标题。当前实现将照片**保存到机器人本地目录**，没有增加平台上传功能。
我们负责的照片均为**原始整帧 JPEG**，不裁剪、不绘制检测框。

| 截图顺序 | 截图照片项 | 负责模块 | 本次保存位置（相对照片根目录） |
|---|---|---|---|
| 1 | 建立事件照片 | 检测、跟踪 | `tracking/event/` |
| 2 | 图像检测结果照片 | 人脸 | 由人脸模块生成；`face/` 仅预留 |
| 3 | 口令检测结果照片 | 语音、照片记录 | 通过：`voice/passed/`；未通过：`voice/failed/` |
| 4 | 目标丢失照片 | 跟踪 | `tracking/lost/` |

### 1. 建立事件照片

巡检中实际确认目标、建立检测事件时，保存该帧的原始整图。
只有通过 ReID 门禁并实际发出目标确认事件后才保存；同一次事件的确认重发不会重复保存。
照片位于 `tracking/event/`，例如 `20261019_113025_t12.jpg`。

### 2. 图像检测结果照片

截图指的是纯人脸核验阶段的通过、未通过照片，归人脸模块负责。
本文不替人脸模块生成照片，也不改变其取帧规则；本次只预留 `face/` 目录。

### 3. 口令检测结果照片

按每一轮口令核验分别保存，结果由目录区分，轮次由文件名区分：

| 本轮结果 | 保存目录 | 取哪一帧 | 文件名示例 |
|---|---|---|---|
| 通过 | `voice/passed/` | 通过时刻或之前最近的有效场景整帧 | `20261019_113038_t12_r1.jpg` |
| 未通过 | `voice/failed/` | 本轮播报结束、实际开始等待回答时固定的整帧 | `20261019_113048_t12_r2.jpg` |

`r1` 是首次播报后的核验，`r2` 是第二次，后续依次类推。文件名不再带 `ok`、`fail`，也不另建轮次子目录。
同一轮只按实际结果保存；取消会话或切换目标不补造未通过照片。
未通过照片在等待开始时固定选帧，收到未通过结果后才写盘。
选帧时只使用不晚于事件时刻的帧，最多相差 1 秒，并受历史缓存长度限制；无法找到有效帧时记录错误，不保存错误目标或未来画面。
语音照片保存的是该时刻的整幅场景，目标短暂不在画面时也可使用同一目标流程中的有效场景帧。

### 4. 目标丢失照片

连续丢失达到 `target.lost_event_timeout_sec`（默认 6 秒）时，保存当前目标**最后一次可信可见的整帧**。
照片位于 `tracking/lost/`。文件名使用该帧的拍摄时间，不是 6 秒后判定丢失的时间。
例如最后可见帧是 11:30:45，即使稍后才判定丢失，仍命名为 `20261019_113045_t12.jpg`。

### 保存目录与文件名

默认根目录为 `~/.ros/dog_patrol/photos`。下面展示不同结果的命名示例，不代表同一轮会同时通过和未通过。

```text
~/.ros/dog_patrol/photos/
├── tracking/
│   ├── event/
│   │   └── 20261019_113025_t12.jpg
│   └── lost/
│       └── 20261019_113045_t12.jpg
├── voice/
│   ├── passed/
│   │   └── 20261019_113038_t12_r1.jpg
│   └── failed/
│       └── 20261019_113048_t12_r2.jpg
├── face/     # 预留
└── reid/     # 预留
```

| 文件名部分 | 含义 |
|---|---|
| `20261019_113038` | 拍摄时间，本地时区，精确到秒 |
| `t12` | 目标语义 ID 为 12，与任务消息的 `target_id` 对应 |
| `r1` | 第一次口令核验；仅语音照片带轮次 |
| 末尾 `_1`、`_2` | 发生同名时追加的短序号，用于防止覆盖 |

语义 ID 是系统使用的目标标识，不是底层跟踪器的 track ID，也不是人脸或 ReID 的人员身份编号。
跟踪照片格式为 `年月日_时分秒_t语义ID.jpg`；语音照片格式为 `年月日_时分秒_t语义ID_rN.jpg`。
同目录发生重名时，例如已有 `20261019_113038_t12_r1.jpg`，下一张保存为 `20261019_113038_t12_r1_1.jpg`。

首次写照片时创建模块目录和跟踪分类目录；`voice/passed/`、`voice/failed/` 在首次保存相应结果时创建。
尚未出现对应结果时，相关目录可能还不存在。当前没有自动清理或保留天数设置。

## 四、怎么用

以下命令用于已完成编译部署、模型资产和硬件准备的机器人。
在工作区根目录执行，并将示例中的 `/path/to/...` 换成实际路径。每个新终端都需加载 ROS 和工作区环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export DOG_PATROL_ASSETS_ROOT=/path/to/dog_patrol_perception_assets_20260813
```

本次消息接口有新增字段，部署时需统一重新编译接口和使用它的节点，包括既有人脸节点；不能混用旧消息接口。
完整安装与构建说明见[仓库 README](../../README.md)。

### 1. 启动时设置配置

完整感知入口的示例：保持 2 次播报、每次等待 10 秒、连续丢失 6 秒，并将首次和后续提示设为同一句英文。

```bash
ros2 launch dog_patrol_perception_bringup perception_stack.launch.py \
  assets_root:="$DOG_PATROL_ASSETS_ROOT" \
  voice_prompt_count:=2 \
  response_timeout_seconds:=10.0 \
  first_prompt:="Please say the passphrase." \
  retry_prompt:="Please say the passphrase." \
  passphrase:="blue star" \
  lost_event_timeout_sec:=6.0 \
  photos_enabled:=true \
  photos_root:="$HOME/.ros/dog_patrol/photos"
```

使用自定义照片目录时，将最后一行改为 `photos_root:=/data/dog_patrol/photos`。
不保存照片时，将 `photos_enabled` 改为 `false`。语音四项设置可以省略，省略时读取部署 YAML。

入侵跟踪时长由主控独立配置，不是感知启动参数：

```bash
ros2 launch dog_patrol_manager mission_supervisor.launch.py \
  intruder_tracking_timeout:=60.0
```

如果已有总启动脚本管理主控，应在该脚本中设置这个参数，不要额外启动第二个主控。
感知启动入口也不会启动导航或下发巡检任务；照片由正常巡检中的相应事件触发，单纯启动节点不会立即产生照片。

### 2. 运行中修改语音设置

下例将后续核验改为最多播报 3 次、每轮等待 8 秒，并更换英文答案：

```bash
ros2 param set /perception_authorization voice_prompt_count 3
ros2 param set /perception_voice_provider response_timeout_seconds 8.0
ros2 param set /perception_voice_provider first_prompt 'Please say the passphrase.'
ros2 param set /perception_voice_provider retry_prompt 'Please try again.'
ros2 param set /perception_voice_provider passphrase 'green light'
```

这 5 个参数从**下一次核验会话**生效；当前会话的所有轮次继续使用会话开始时的设置。
等待新会话再验证改动，不要把第二轮理解为“下一次会话”。
命令只修改当前进程的参数，不会写回 YAML，也不会在节点重启后自动保留。

入侵跟踪时长、目标丢失判定时长、照片开关和照片路径是**启动时读取**的设置。
应修改启动配置并重启对应节点，不使用 `ros2 param set` 作为它们的生效方式。

### 3. 重启后仍然使用修改后的配置

把所需参数写入机器人实际使用的启动脚本或服务启动命令；主控参数写入主控启动入口。
语音的四个参数也可以保存到部署 YAML。整套感知启动过程中，语音模块读取的配置文件是：

```text
<assets_root>/runtime/voice_verification.yaml
```

这是一份语音专用配置，保存口令、提示语、单轮等待时长和音频设备设置，不是所有感知模块共用的配置文件。
各项配置的位置如下：

| 配置内容 | 持久配置入口 |
|---|---|
| 核验口令、正确回答、单轮口令等待时长及音频设备 | `runtime/voice_verification.yaml`；显式语音启动参数可覆盖业务设置 |
| 口令播报次数 | 感知启动命令中的 `voice_prompt_count`，传给编排节点 |
| 目标丢失判定时长、照片开关和路径 | 感知启动命令中的 `lost_event_timeout_sec`、`photos_enabled`、`photos_root`，覆盖 `runtime/perception_tracking.yaml` 中对应值 |
| 入侵跟踪时长 | 主控启动命令中的 `intruder_tracking_timeout` |
| 人脸、ReID 设置 | 分别由 `runtime/face.yaml`、`runtime/reid.yaml` 及对应模块的启动参数管理 |

修改该文件中以下四项，保留文件内其他硬件和音频设置：

```yaml
passphrase: green light
first_prompt: Please say the passphrase.
retry_prompt: Please try again.
response_timeout_seconds: 8.0
```

**配置优先级和文件区别：**

- 语音四项：显式启动参数覆盖 YAML；启动参数省略或留空时沿用 YAML。运行时设置覆盖当前进程的待生效配置。
- 修改 YAML 不会自动通知正在运行的节点；重启读取，或用上一节的命令同步更新当前进程。
- `voice_prompt_count` 仍由编排节点的启动参数配置，不能追加到上述语音 YAML 中。
- 语音 YAML 使用上面的平铺格式，不能直接用 `ros2 param dump` 生成的 `节点/ros__parameters` 格式替换。
- 完整感知启动读取 `<assets_root>/runtime/perception_tracking.yaml`，但随后用启动参数覆盖目标丢失时长、照片开关和照片根目录。
  即使没有显式传入，也会使用启动入口的默认值 `6.0`、`true`、`~/.ros/dog_patrol/photos`。调整这三项时应同步修改启动命令。
- 资产目录的 `voice/voice_verification.yaml` 和源码包的 `config/voice_verification.yaml` 是参考配置；整套感知启动时为语音模块选择 `runtime/voice_verification.yaml`，不能只修改参考文件。
- 单独启动 `voice.launch.py` 时，使用 `config_file:=/实际路径/voice_verification.yaml` 指定语音 YAML；其四个同名启动参数仍具有覆盖优先级。

### 4. 确认参数值

```bash
ros2 param get /perception_authorization voice_prompt_count
ros2 param get /perception_voice_provider response_timeout_seconds
ros2 param get /perception_voice_provider first_prompt
ros2 param get /perception_voice_provider retry_prompt
ros2 param get /perception_voice_provider passphrase
ros2 param get /mission_supervisor intruder_tracking_timeout
ros2 param get /dog_patrol_perception_tracking_node target.lost_event_timeout_sec
ros2 param get /dog_patrol_perception_tracking_node photos.enabled
ros2 param get /dog_patrol_perception_tracking_node photos.root
```

语音参数查询显示已经接受的新设置；正在进行的会话可能仍按旧设置执行，下一次会话才使用新值。
上述节点名使用默认部署名称；如果部署增加了命名空间，查询和修改命令需使用对应完整节点名。

### 5. 查找并核对照片

先通过上一节查询 `photos.root`，再进入该目录。默认路径下可使用：

```bash
ls -R "$HOME/.ros/dog_patrol/photos"
```

按一次正常任务检查：

1. 实际建立目标事件后，在 `tracking/event/` 查看整帧照片。
2. 目标进入口令核验后，本轮通过查看 `voice/passed/`，本轮未通过查看 `voice/failed/`，核对文件名中的 `t<ID>` 和 `rN`。
3. 连续丢失达到配置时长后，在 `tracking/lost/` 查看最后可见画面；其拍摄时间会早于丢失判定时间。

没有照片时，先确认照片开关、实际根目录、对应事件是否发生，再查看 tracking 节点中的 `event photo:` 错误日志。
相机缺帧、语音事件与相机时间不一致、写队列满或目录不可写会导致对应照片无法保存，并报告错误。
照片异步编码和写盘；事件到来后文件可能稍后出现。当前没有平台上传和自动清理功能。

## 五、截图中的边界规则与当前实现

| 截图规则 | 当前处理方式 |
|---|---|
| 核验总计时从首次检测开始，重复播报、短暂丢失不重置 | 当前不增加核验总时长；只配置进入入侵跟踪后的时长，默认 60 秒 |
| 目标短暂丢失不足阈值，重新发现后继续原流程 | 按此规则执行；清零丢失计时，不重置入侵跟踪计时 |
| 核验通过、时限内未通过、目标丢失是互斥最终结果 | 当前沿用任务事件流程：主控按状态及 `state_seq` 拒绝失效结果；核验未通过后进入入侵跟踪，之后仍可能因丢失或跟踪到期结束。本次未新增平台统一最终结果上报 |
| 重复核验忽略仅针对同一人物 | 由 ReID 负责，跟踪侧使用 ReID 返回的排除结论 |
| 播报次数耗尽后继续跟踪入侵者 | 停止播报和监听，按配置的入侵跟踪时长跟踪；没有额外的超时照片 |

## 六、部署集成补充

以下是内部通信和部署事项，不属于截图中的日常业务配置：

| 设置 | 默认值与用途 |
|---|---|
| 完整感知启动的 `voice_verification_event_topic` | `/perception/voice_verification_event`；通知照片模块实际等待开始、通过和未通过时刻 |
| tracking 的 `photos.voice_event_topic` | 必须与语音节点的 `voice_verification_event_topic` 一致；完整感知启动会统一传入 |
| `use_sim_time` | 默认 `false`；仿真时保持相机、语音、主控在同一时间域，供选帧使用 |

保存照片使用源图副本，JPEG 编码和写盘在后台线程中执行。帧历史和写队列默认各最多 16 帧，退出时排空已经入队的照片。
这些缓存大小没有作为本次面向使用者的 ROS 配置项开放。

`AuthorizationCommand`、`AuthorizationEvidence` 新增 `round_number`，另有 `VoiceVerificationEvent`。
部署时要让 tracking、voice、orchestrator 和人脸节点使用同一套重新生成的消息，并同步升级人脸与编排节点。
第三轮及以后语音继续使用 `DUAL_SECOND` 接口并携带轮次；人脸保留首次联合命令的 stage，
跨口令轮次持续提供有效结果，后续口令命令不重新启动人脸核验。
成功结果继续转发 ReID，旧放行缓存作废；下一次确认重新询问 ReID，服务不可用时保留已有旁路规则。

照片相关 22 项自动化测试已通过，覆盖取帧、语义 ID、轮次、目录、同名防覆盖和实际跟踪事件。
普通环境可通过 `TRACKING_ENABLE_ORIN_RUNTIME=OFF` 编译检查；相机、R818 收音播报和 Orin 连续运行仍需部署设备验收。
