# 感知配置项与照片使用说明

本文按需求截图中的名称和顺序，逐项对应配置项、照片与置信度，并说明如何启动、修改配置、保存配置和查找结果。
统一覆盖检测、跟踪、人脸、语音口令、ReID 及必要的任务编排。各模块仍分别读取自己的配置文件。

当前默认规则：语音最多播报 **2 次**，每次播报结束后等待 **10 秒**；次数耗尽后停止播报和监听。
开始时单独核验人脸，默认 **3 秒**；未通过后进入联合核验，人脸持续识别，覆盖口令播报、等待和所有后续轮次。
联合核验不设独立的人脸截止时间。人脸或口令任一通过立即结束；口令次数耗尽且仍未通过时统一结束并进入入侵跟踪。
进入入侵跟踪后最多跟踪 **60 秒**；目标连续丢失 **6 秒**判定丢失。当前不增加核验总时长，也不加入中文语音。
同一人物核验通过后，默认 **24 小时**内免重复核验；跨午夜或正常持久化后的重启不重新起算这段时长。

## 一、平台可配置项：与截图逐项对应

下表保留截图原名称。截图中的“核验口令”是机器人播报的内容，“正确回答”是人员需要说出的答案。
当前提供 ROS 参数、启动参数和 YAML 配置入口；平台界面及网络配置下发需要平台侧接入这些入口。

| 截图顺序 | 截图配置项 | 负责模块 | 配置入口或节点参数 | 当前默认值 |
|---|---|---|---|---|
| 1 | 纯人脸核验时长 | 人脸 | `runtime/face.yaml` 中的 `initial_face_window_seconds` | `3.0` 秒 |
| 2 | 单轮口令等待时长 | 语音 | `response_timeout_seconds` | `10.0` 秒 |
| 3 | 口令播报次数 | 编排、语音 | `voice_prompt_count` | `2` 次，包含首次 |
| 4 | 核验口令 | 语音 | `first_prompt`、`retry_prompt` | 首次、后续英文提示语，见下文 |
| 5 | 正确回答 | 语音 | `passphrase` | `blue star` |
| 6 | 入侵跟踪时长 | 主控、跟踪 | `intruder_tracking_timeout` | `60.0` 秒 |
| 7 | 目标丢失判定时长 | 跟踪 | `target.lost_event_timeout_sec` | `6.0` 秒 |
| 8 | 重复核验忽略时长 | ReID | `runtime/reid.yaml` 中的 `handled_ignore_seconds` | `86400.0` 秒，即 24 小时 |

截图中的 **8 项业务设置对应 9 个参数**；“核验口令”拆为首次、后续两段文本。
另有照片总开关、共享根目录和人脸专用照片设置，见第二节。内部消息话题单独列在文末。

### 1. 纯人脸核验时长

对应截图中“开始人脸核验，直至开启口令核验前”的时长，由人脸模块负责。
默认 3 秒；在人脸模块的 `runtime/face.yaml` 设置 `initial_face_window_seconds`，重启人脸节点后生效。
设为 `0` 时回退到该文件的 `window_timeout_seconds`。此时长仅影响纯人脸阶段，不限制联合核验。
该值是人脸 YAML 配置项，不是同名 ROS 参数；不要通过 `ros2 param set` 修改它。具体操作见第四节。

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

对应截图中“同一人物核验通过后，在一定时长内再次识别时不重复核验”，参数
`handled_ignore_seconds` 属于 `/perception_reid`，默认 `86400.0` 秒，即 24 小时；`0.0` 表示关闭免核验。

跟踪侧将核验通过结果转发给 ReID，由 ReID 按人物外观延续资格；换了底层 track ID 也不重新计时。
仅见过、未通过或取消核验的人不会因此获得资格，也不影响对其他人物发起核验。
从本次核验通过结果落地时起计时，跨午夜继续有效，正常持久化后重启按剩余时间继续判断，停机期间也计入。
重复识别、云端模板更新和服务响应延迟不会刷新原来的起点；真正到期后再次识别，需要重新完整核验。
任务 reset 会清除免核验资格。

完整感知启动读取 `<assets_root>/runtime/reid.yaml`；该文件采用 ROS 参数文件格式。
参数在 **ReID 节点启动时读取**，修改 YAML 后重启对应节点或整套感知服务。
当前不支持通过 `ros2 param set` 热更新此业务逻辑；跟踪侧无需再配一份忽略时长。
配置、查询及验证步骤见第四节。

## 二、照片保存的补充配置

照片设置不属于截图中的八项核验业务设置。完整感知启动用下面两项统一控制跟踪、语音和人脸的任务照片：

| 用途 | 跟踪节点参数 | 整套感知启动参数 | 默认值 | 生效时机 |
|---|---|---|---|---|
| 是否保存照片 | `photos.enabled` | `photos_enabled` | `true` | 节点启动 |
| 照片根目录 | `photos.root` | `photos_root` | `~/.ros/dog_patrol/photos` | 节点启动 |

`photos_enabled:=false` 同时关闭跟踪、语音和人脸任务照片，不关闭各模块核验功能。
语音每轮的置信度 JSON 仍会保存；此时图片地址为空，照片状态为 `disabled`。
`photos_root` 是共享根目录，跟踪写入其 `tracking/`，语音写入 `voice/`，人脸写入 `face/`。
当前照片功能用于任务模式；完整感知启动入口已使用该模式。
根目录不能为空；建议使用绝对路径或 `~/...`。`~` 是运行节点的用户主目录，后台服务运行用户不同，默认保存位置也会不同。
照片目录按需自动创建，运行节点的用户须能写入该路径。

人脸还读取 `runtime/face.yaml` 中以下照片设置，均在启动时生效：

| 人脸 YAML 配置项 | 部署默认值 | 用法 |
|---|---|---|
| `evidence_photo_enabled` | `true` | 人脸照片独立开关；必须与总开关同时开启才保存 |
| `evidence_photo_root` | `/data/dog_patrol/photos/face` | 未传共享根目录时的回退位置，已包含 `face` 后缀 |
| `evidence_photo_format` | `jpg` | 支持 `jpg`、`png`；只影响人脸照片 |
| `evidence_photo_jpeg_quality` | `90` | JPEG 质量，整数 1～100 |
| `evidence_photo_retention_seconds` | `604800` | 保留时长，默认 7 天，须大于 0 |
| `evidence_photo_max_files` | `10000` | 人脸照片总数量上限，须为正整数 |

完整感知入口会用 `<photos_root>/face/` 覆盖人脸回退路径，因此单改 `evidence_photo_root` 不会改变完整启动时的保存位置。
单独启动 `face.launch.py` 时，传入 `photos_root` 也会自动追加 `/face`，不要把共享根目录写成已经带 `face` 的路径。
不传 `photos_root` 才使用 YAML 的 `evidence_photo_root` 原值。

人脸保存新照片时按文件修改时间清理过期照片，并删除最旧照片以满足数量上限；不是后台定时清理。
跟踪和语音任务照片当前没有自动清理或保留天数配置。
这些开关和路径不控制 ReID 的身份库、缩略图或 embedding 载荷；它们由 `runtime/reid.yaml` 中的 `data_dir` 等设置管理。

## 三、机器人回传照片及置信度：与截图逐项对应

本节沿用截图的标题。当前实现将照片**保存到机器人本地目录**，没有增加平台上传功能。
检测、跟踪和语音照片均为**原始整帧 JPEG**，不裁剪、不绘制检测框。
人脸保存的是 tracking 提供的主目标局部图（crop），不再二次裁剪或绘制；文件格式由人脸照片配置决定。

| 截图顺序 | 截图照片项 | 负责模块 | 本次保存位置（相对照片根目录） |
|---|---|---|---|
| 1 | 建立事件照片 | 检测、跟踪 | `tracking/event/` |
| 2 | 图像检测结果照片及置信度 | 人脸 | 照片：`face/state-<状态序号>/target-<语义ID>/`；现有相似度通过人脸证据消息提供，图片与分数对应的限制见下文 |
| 3 | 口令检测结果照片及置信度 | 语音、照片记录 | 通过：`voice/passed/`；未通过：`voice/failed/`；每轮分数保存在同名 JSON，并通过语音事件消息发布 |
| 4 | 目标丢失照片 | 跟踪 | `tracking/lost/` |

### 1. 建立事件照片

巡检中实际确认目标、建立检测事件时，保存该帧的原始整图。
只有通过 ReID 门禁并实际发出目标确认事件后才保存；同一次事件的确认重发不会重复保存。
照片位于 `tracking/event/`，例如 `20261019_113025_t12.jpg`。

### 2. 图像检测结果照片及置信度

截图指的是纯人脸核验阶段的通过、未通过照片；联合核验中的人脸通过也会保存照片。

| 人脸阶段及结果 | 取哪一帧 | 文件名中的标记 |
|---|---|---|
| 纯人脸通过 | 实际识别通过的输入局部图 | `initial_face_passed` |
| 纯人脸未通过 | 本阶段第一次成功检测识别人脸的局部图 | `initial_face_not_passed` |
| 联合阶段人脸通过 | 实际识别通过的输入局部图 | 通常为 `dual_first_passed` |

人脸在联合核验中持续识别，后续口令换轮不重启人脸任务，文件名保留启动该任务时的阶段标记。
因此 `dual_first` 表示这项人脸任务从首次联合阶段开始，不能用它判断当时口令已进行到第几轮。
联合阶段不另设人脸失败计时，也不为每轮口令生成一张人脸失败照片；每轮未通过照片按下一项的语音规则保存。
未通过照片携带的 `similarity` 必须由同一张局部图实际计算，不能使用收集阶段的默认
`0.0`，也不能替换为后续图像的最高分。默认五帧收集阶段会为证据图单独计算单帧相似度，
但最终通过/不通过仍沿用多帧平均特征，证据计算不会改变核验决策流程。
通过照片对应实际触发通过的输入局部图；其 `similarity` 是现有多帧核验结果分数，允许包含
多帧平均特征与既有尺寸加权，不要求改成该输入图的单帧分数。
全程没有成功检测识别人脸时不保存人脸照片，`similarity=0.0` 表示没有人脸证据，而非一次
有效的零分测量，并同时返回 `similarity_status=no_face_detected`。成功计算时状态为
`measured`，计算异常时为 `measurement_error`，非人脸提供方或取消结果可使用
`not_applicable`；任务取消也不会补造未通过照片。

照片按状态序号和目标语义 ID 分目录，例如 `face/state-17/target-12/`。
`state-17` 是任务消息的 `state_seq`，`target-12` 是目标语义 ID，不是人员姓名或人脸库身份编号。
保存后的绝对本地地址写入 `AuthorizationEvidence.image_url`，例如
`file:///data/dog_patrol/photos/face/state-17/target-12/1792438238000000000_target-12_initial_face_passed_f83.jpg`。
这类 `file://` 地址只能指向机器人本地文件，平台需另行读取或上传，不能当成公网图片链接。

读取人脸核验证据时，照片地址取 `AuthorizationEvidence.image_url`，身份取 `person_id`，
分数取 `similarity`，并结合 `similarity_status` 判断该分数是有效测量、无人脸、计算异常或不适用。

### 3. 口令检测结果照片及置信度

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

每轮另保存一份与照片同名的 JSON，例如 `voice/passed/20261021_143010_t12_r1.json`。
图片和 JSON 的文件名主体一致；发生同名时，两者使用相同的 `_1`、`_2` 后缀。
JSON 通过 `state_seq + target_id + round_number` 关联当前核验轮次。
口令置信度读取 `confidence_percent`，取值范围为 `0`～`100`；先检查 `confidence_valid` 是否为 `true`。
没有有效分数时 JSON 中为 `null`，不要当成零分。取消或设备故障不补造正常的未通过记录。

默认保存路径为 `~/.ros/dog_patrol/photos/voice/passed/` 或 `voice/failed/`，分数不写进图片内容或文件名。
通过启动参数 `photos_root` 修改共享根目录后，新生成的照片与 JSON 会保存到新根目录下的相应分类目录，已有文件保留在原处。
下面仅列出读取置信度和关联照片所需的 JSON 字段：

```json
{
  "state_seq": 17,
  "target_id": 12,
  "round_number": 1,
  "result": "passed",
  "confidence_valid": true,
  "confidence_percent": 75,
  "image_file": "20261021_143010_t12_r1.jpg",
  "photo_status": "saved"
}
```

| JSON 字段 | 怎么用 |
|---|---|
| `state_seq`、`target_id`、`round_number` | 联合关联任务状态、目标语义 ID 和本次口令轮次，避免混用不同目标或轮次的结果 |
| `result` | `passed` 为通过，`not_passed` 为未通过；后者保存在 `voice/failed/` |
| `confidence_valid` | 先检查此标记，只有 `true` 才能把分数作为有效测量读取 |
| `confidence_percent` | 本轮口令置信度，`75` 表示 `75%`；没有有效测量时 JSON 为 `null`，不是零分 |
| `wait_started_stamp_ns`、`result_stamp_ns`、`image_stamp_ns` | 本轮实际等待开始、判定、照片源帧时间，单位为纳秒，使用事件与相机一致的时间域；缺图时源帧时间为 `null` |
| `image_file` | 同目录下照片的文件名，不是绝对路径或网络 URL；用 JSON 所在目录拼接即可定位照片，缺图时为 `null` |
| `photo_status` | `saved` 已保存照片；`disabled` 关闭照片；`missing_frame` 缺有效帧；`write_failed` 图片编码或写入失败 |

置信度记录独立于照片是否成功：照片关闭、缺帧或编码/写入失败但目录仍可写时，保存 JSON，
`image_file=null`，状态分别为 `disabled`、`missing_frame`、`write_failed`。
没有有效源帧时用判定时间命名，`image_stamp_ns=null`。
目录不可写或写队列满会记录错误；核验主流程不等待磁盘 I/O。JSON 不保存口令文本或音频。

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
│   │   ├── 20261019_113038_t12_r1.jpg
│   │   └── 20261019_113038_t12_r1.json
│   └── failed/
│       ├── 20261019_113048_t12_r2.jpg
│       └── 20261019_113048_t12_r2.json
└── face/
    └── state-17/
        └── target-12/
            └── 1792438238000000000_target-12_initial_face_passed_f83.jpg
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

人脸采用模块现有命名格式：`<拍摄时间戳>_target-<语义ID>_<阶段>_<结果>_f<帧号>.jpg`。
其中拍摄时间戳是输入源图的纳秒时间戳；`f83` 是源帧号，源帧号不可用时为 `fna`。
人脸通过、未通过由文件名区分，同名时在扩展名前追加 `-1`、`-2` 等短序号防止覆盖。
修改人脸格式为 PNG 后，扩展名为 `.png`；跟踪、语音仍使用上表中的 JPEG 命名。

截图四项回传照片不包含 ReID 专用任务照片，因此这里不自动创建 `reid/` 子目录。
ReID 的缩略图和特征文件是去重及云端交互载荷，位于 `data_dir` 下，不能当作本表中的任务照片。
实际路径可通过 `ros2 param get /perception_reid data_dir` 查询。只有启用云端交互且生成载荷时，才会出现
`<data_dir>/<请求ID>/thumbnail.webp`（编码回退时为 `thumbnail.jpg`）和 `embedding.bin`；它们不受总照片开关或共享照片根目录控制。

首次写照片时创建模块目录和跟踪分类目录；`voice/passed/`、`voice/failed/` 在首次保存相应结果时创建。
尚未出现对应结果时，相关目录可能还不存在。人脸目录在保存时按上述保留期、数量上限清理；跟踪、语音目录需按部署需求另行维护。

## 四、怎么用

以下命令用于已完成编译部署、模型资产和硬件准备的机器人。
在工作区根目录执行，并将示例中的 `/path/to/...` 换成实际路径。每个新终端都需加载 ROS 和工作区环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export DOG_PATROL_ASSETS_ROOT=/path/to/dog_patrol_perception_assets_20260813
```

部署版本需同时包含人脸、语音照片和 ReID 忽略时长的接口及实现。
更新时统一重新编译消息接口和使用它们的节点，尤其是 `dog_patrol_reid_provider` 与 tracking，不能混用新旧 `ReIDDedup` 接口。
完整安装与构建说明见[仓库 README](../../README.md)。

### 1. 启动时设置配置

先按本节第 3 项检查人脸和 ReID 的部署 YAML，再运行完整感知入口。
下面示例保持 2 次播报、每次等待 10 秒、连续丢失 6 秒，并将首次和后续提示设为同一句英文：

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
不保存任务照片时，将 `photos_enabled` 改为 `false`，跟踪、语音和人脸照片都会关闭。
语音四项设置可以省略，省略时读取部署 YAML。
完整感知入口没有 `initial_face_window_seconds` 或 `handled_ignore_seconds` 同名启动参数；这两项分别修改人脸和 ReID 的 YAML。

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

纯人脸核验时长、ReID 忽略时长、入侵跟踪时长、目标丢失判定时长，以及照片开关、路径和人脸照片设置
都是**启动时读取**的设置。应修改对应 YAML 或启动配置并重启，不使用 `ros2 param set` 作为它们的生效方式。

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
| 纯人脸核验时长及人脸独立照片设置 | `runtime/face.yaml`；共享照片开关和根目录由完整感知启动参数统一传入 |
| 重复核验忽略时长及 ReID 身份库、载荷目录 | `runtime/reid.yaml` 中 `/perception_reid/ros__parameters` 下的对应项 |

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
- 语音、人脸 YAML 使用平铺格式，不能直接用 `ros2 param dump` 生成的 `节点/ros__parameters` 格式替换；ReID YAML 使用 ROS 参数文件格式，见下面示例。
- 完整感知启动读取 `<assets_root>/runtime/perception_tracking.yaml`，但随后用启动参数覆盖目标丢失时长、照片开关和照片根目录。
  即使没有显式传入，也会使用启动入口的默认值 `6.0`、`true`、`~/.ros/dog_patrol/photos`。调整这三项时应同步修改启动命令。
- 资产目录的 `voice/voice_verification.yaml` 和源码包的 `config/voice_verification.yaml` 是参考配置；整套感知启动时为语音模块选择 `runtime/voice_verification.yaml`，不能只修改参考文件。
- 单独启动 `voice.launch.py` 时，使用 `config_file:=/实际路径/voice_verification.yaml` 指定语音 YAML；其四个同名启动参数仍具有覆盖优先级。

**人脸的设置方法：**

编辑机器人实际使用的 `$DOG_PATROL_ASSETS_ROOT/runtime/face.yaml`，按需修改下面这些已有字段，保留模型路径、识别阈值等其他内容：

```yaml
initial_face_window_seconds: 3.0
evidence_photo_enabled: true
evidence_photo_format: jpg
evidence_photo_jpeg_quality: 90
evidence_photo_retention_seconds: 604800
evidence_photo_max_files: 10000
```

例如改为 `initial_face_window_seconds: 5.0`，会把纯人脸阶段改为 5 秒；联合阶段仍持续识别。
只关闭人脸照片时设 `evidence_photo_enabled: false`，同时保持总入口 `photos_enabled:=true`，跟踪和语音照片继续保存。
修改后重启人脸节点或整套感知服务；修改文件不会自动通知正在运行的节点。

完整启动仍用 `photos_root` 决定人脸照片路径。只有单独调试、且不传共享根目录时，才按需修改 YAML 的
`evidence_photo_root`。单独启动人脸的示例为：

```bash
ros2 launch dog_patrol_perception_face face.launch.py \
  config_file:="$DOG_PATROL_ASSETS_ROOT/runtime/face.yaml" \
  photos_root:="$HOME/.ros/dog_patrol/photos" \
  photos_enabled:=true
```

该命令会同时启动人脸核验和人脸就绪检查，不要与已经运行的完整感知栈重复启动。

**ReID 的设置方法：**

编辑机器人实际使用的 `$DOG_PATROL_ASSETS_ROOT/runtime/reid.yaml`，在已有节点和 `ros__parameters` 下修改这一项，保留其他配置：

```yaml
/perception_reid:
  ros__parameters:
    handled_ignore_seconds: 86400.0
```

示例中的层级是必需的，不要把这一个字段写成顶层的平铺 YAML，也不要用此片段覆盖整份模型配置。
保持 `86400.0` 就是默认 24 小时；改成 `3600.0` 是 1 小时，`0.0` 是关闭免核验。
完整感知启动会读取这份文件，tracking 无需配置同名参数。修改后重启 ReID 或整套感知服务。
启动时恢复的资格继续使用保存的核验通过时刻判断，重启不会把计时起点改成当前时间。

免核验资格保存在 ReID 的身份库中；`persist.enable: true` 为部署默认值，库文件位于 `data_dir/library.bin`。
正常保存后重启可以恢复；异常断电最多丢失最后一个 `persist.save_interval_sec` 周期内的更新，默认周期 10 秒。
`data_dir` 需要可写并持久保留；停服后删除库文件，或执行任务 reset，会清除原有免核验资格。

单独调试时可加载同一份部署文件，并用 `-p` 覆盖本次进程的忽略时长：

```bash
ros2 run dog_patrol_reid_provider reid_node --ros-args \
  --params-file "$DOG_PATROL_ASSETS_ROOT/runtime/reid.yaml" \
  -p handled_ignore_seconds:=3600.0
```

该覆盖不会写回 YAML。此命令也不能与完整感知栈中的 ReID 节点重复运行。

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
ros2 param get /perception_face_provider config_file
ros2 param get /perception_face_provider photos_enabled
ros2 param get /perception_face_provider photos_root
ros2 param get /perception_reid handled_ignore_seconds
ros2 param get /perception_reid data_dir
```

语音参数查询显示已经接受的新设置；正在进行的会话可能仍按旧设置执行，下一次会话才使用新值。
上述节点名使用默认部署名称；如果部署增加了命名空间，查询和修改命令需使用对应完整节点名。
人脸的 `initial_face_window_seconds` 和独立照片设置不以同名 ROS 参数公开；根据 `config_file` 查询结果核对实际 YAML，并在修改后重启。
ReID 的查询结果应与启动时读取的 YAML 或 `-p` 覆盖一致；不要用运行时 `ros2 param set` 改出的参数显示值判断业务已经热更新。

### 5. 查找并核对照片

先通过上一节查询 `photos.root`，再进入该目录。默认路径下可使用：

```bash
ls -R "$HOME/.ros/dog_patrol/photos"
```

按一次正常任务检查：

1. 实际建立目标事件后，在 `tracking/event/` 查看整帧照片。
2. 人脸有核验结果后，在 `face/state-<序号>/target-<语义ID>/` 查看目标局部图；按 `initial_face_passed`、`initial_face_not_passed` 或联合阶段的 `passed` 标记核对取帧规则。
3. 目标进入口令核验后，本轮通过查看 `voice/passed/`，本轮未通过查看 `voice/failed/`，核对文件名中的 `t<ID>` 和 `rN`；打开同名 JSON 查看本轮口令置信度，并用 `image_file` 对应照片。
4. 连续丢失达到配置时长后，在 `tracking/lost/` 查看最后可见画面；其拍摄时间会早于丢失判定时间。

也可监听人脸证据消息，查看 `provider: face` 的 `image_url`、目标 ID 和阶段：

```bash
ros2 topic echo /perception/authorization_evidence
```

`image_url` 为空时检查人脸照片总开关、独立开关、有效输入和写盘日志；核验结果本身不保证一定有图片。

**查看口令置信度：** 默认目录中的一份记录可用以下命令打开；文件名替换成实际生成的 JSON：

```bash
python3 -m json.tool "$HOME/.ros/dog_patrol/photos/voice/passed/20261021_143010_t12_r1.json"
```

先核对 `target_id`、`round_number`、`result`，再检查 `confidence_valid` 和 `confidence_percent`。
JSON 与图片同名，发生文件名碰撞时两者使用相同短序号；照片未保存时也应检查是否有独立的 JSON 记录。
照片与 JSON 的写入由任务模式的 tracking 照片记录器负责，单独启动语音节点只会发布消息，不会自动保存 JSON。

实时查看每轮口令置信度可使用：

```bash
ros2 topic echo /perception/voice_verification_event
```

`event: 0` 是等待开始，`confidence_valid: false`，消息中的默认零值不代表零分；
`event: 1` 是通过，`event: 2` 是未通过，读取终态消息中有效的 `confidence_percent` 即可。
通过消息中的 `observed_state_seq`、`target_id`、`round_number` 关联本轮；保存到 JSON 后状态序号字段名为 `state_seq`。
默认话题可通过启动参数 `voice_verification_event_topic` 修改，使用自定义值时需监听对应话题。
实时 ROS 话题与本地 JSON 均不等于已经上传到平台；平台需要自行订阅该话题，或读取 JSON 及其关联的照片。

没有照片时，先确认照片开关、实际根目录、对应事件是否发生，再查看 tracking 节点中的 `event photo:` 错误日志。
相机缺帧、语音事件与相机时间不一致、写队列满或目录不可写会导致对应照片无法保存，并报告错误。
跟踪和语音照片异步编码、写盘，事件到来后文件可能稍后出现；人脸在保存后随证据消息给出本地文件地址。
跟踪、语音当前没有自动清理；人脸按配置在保存新照片时执行清理。各模块均没有在本流程中增加平台上传。

### 6. 验证重复核验忽略时长

先用 `ros2 param get /perception_reid handled_ignore_seconds` 确认启动值，再完成一次真实的人脸或口令核验通过。
在忽略时长内重新识别同一人物，应跳过重复核验；换底层 track ID、跨午夜或正常重启后仍按原截止时间判断。
换成另一人物时应照常核验。真正到期后再次识别原人物，应重新走纯人脸和必要的口令流程。

联调可临时在 YAML 配置较短时间并重启，便于观察到期；验证结束后改回 `86400.0` 并再次重启。
只检测到人、核验失败或取消任务不能作为“已通过”样本；不要通过清空库、任务 reset 来测试正常重启是否保留资格。

## 五、截图中的边界规则与当前实现

| 截图规则 | 当前处理方式 |
|---|---|
| 核验总计时从首次检测开始，重复播报、短暂丢失不重置 | 当前不增加核验总时长；只配置进入入侵跟踪后的时长，默认 60 秒 |
| 目标短暂丢失不足阈值，重新发现后继续原流程 | 按此规则执行；清零丢失计时，不重置入侵跟踪计时 |
| 核验通过、时限内未通过、目标丢失是互斥最终结果 | 当前沿用任务事件流程：主控按状态及 `state_seq` 拒绝失效结果；核验未通过后进入入侵跟踪，之后仍可能因丢失或跟踪到期结束。本次未新增平台统一最终结果上报 |
| 重复核验忽略仅针对同一人物 | 由 ReID 维护默认 24 小时资格；跨天、正常持久化后的重启按原通过时刻继续判断，其他人物不继承 |
| 播报次数耗尽后继续跟踪入侵者 | 停止播报和监听，按配置的入侵跟踪时长跟踪；没有额外的超时照片 |

## 六、部署集成补充

以下是内部通信和部署事项，不属于截图中的日常业务配置：

| 设置 | 默认值与用途 |
|---|---|
| 完整感知启动的 `voice_verification_event_topic` | `/perception/voice_verification_event`；通知照片模块实际等待开始、通过和未通过时刻，终态事件携带每轮口令置信度 |
| tracking 的 `photos.voice_event_topic` | 必须与语音节点的 `voice_verification_event_topic` 一致；完整感知启动会统一传入 |
| `use_sim_time` | 默认 `false`；仿真时保持相机、语音、主控在同一时间域，供选帧使用 |

跟踪、语音保存照片使用源图副本，JPEG 编码和写盘在后台线程中执行。帧历史和写队列默认各最多 16 帧，退出时排空已经入队的照片。
语音 JSON 与图片在同一个后台任务中写入；`VoiceVerificationEvent` 结果事件携带本轮置信度及有效标记，保存时须与当前目标、轮次对应。
这些缓存大小没有作为本次面向使用者的 ROS 配置项开放。

`AuthorizationCommand`、`AuthorizationEvidence` 新增 `round_number`，另有 `VoiceVerificationEvent`。
部署时要让 tracking、voice、orchestrator 和人脸节点使用同一套重新生成的消息，并同步升级人脸与编排节点。
第三轮及以后语音继续使用 `DUAL_SECOND` 接口并携带轮次；人脸保留首次联合命令的 stage，
跨口令轮次持续提供有效结果，后续口令命令不重新启动人脸核验。
成功结果继续转发 ReID，旧放行缓存作废；下一次确认重新询问 ReID，服务不可用时保留已有旁路规则。
ReID 的 `ReIDDedup` 通过 `handled_until` 给出免核验固定截止时刻，单位为系统墙钟 epoch 秒。
tracking 自动换算并释放到期排除项，调用方不应把“收到响应时刻”作为新的忽略期起点。
此业务截止使用系统墙钟，不使用 `use_sim_time` 的仿真时钟；现场与仿真联调时都要保持运行进程的系统时间一致。

人脸图像和路径配置细节见[人脸模块说明](../../src/perception/dog_patrol_perception_face/README.md)；
去重接口与联调细节见[ReID 模块说明](../../src/perception/dog_patrol_reid_provider/README.md)。

照片记录器 26 项自动化测试已通过，覆盖取帧、语义 ID、轮次、目录、同名防覆盖及语音置信度 JSON。
普通环境可通过 `TRACKING_ENABLE_ORIN_RUNTIME=OFF` 编译检查；相机、R818 收音播报和 Orin 连续运行仍需部署设备验收。
