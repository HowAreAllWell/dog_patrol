# PR #51 工作区实机测试说明

## 测试版本

- 工作区：`/mnt/nvme/workspace/dog_patrol`。
- 当前使用分支：`merge`；原测试分支 `feature/reid-dedup-gate` 已在合并后按用户要求清理。
- 已审查提交：`88fa5286dd3fb04720ae4b193fe74ec7d60b1bc0`。
- 2026-10-12 已按用户授权合入本地及远端 `merge`，合并提交为 `1cffd24`；测试提交保留在合并历史中。
- GitHub PR #51 已确认 merged；后续在 merge 分支使用和修改。
- 使用当前工作区的 `install/setup.bash`，无需加载此前 /tmp 审查环境。

## 当前验收结论（2026-10-12）

用户已完成基本实机测试，反馈主要行为正常，可以接受当前 PR。运行范围按用户确认：
tracking 与 ReID 一起执行，不考虑单侧离线或独立重启；这些场景不列为本轮待修问题。

保留一项用户已接受的低概率限制：处置完成后，如果下一次图像帧前任务已被终止等状态变化
清掉最新 handled 字段，ReID 可能漏记该次完成，后续可能重复核验。这条漏发不会把未处理
人员误标为已处理。本轮不为该窗口扩充同步机制，详细证据和验收依据见
[最新评审结论](../reviews/pr_50_reid_review.md)。验收后的合并已完成并同步远端，PR #51 已接收。

本轮新增 CLIP 人体特征用于判断“这个人今天是否已经完成处置”。人员跟踪和 MobileFaceNet
人脸认证沿用现有实现。当前感知生产 launch 已自动启动 ReID，控制台的感知按钮一并管理它，
无需单独启动服务。整体感知 READY 等待 detection_tracking、reid、face、voice 四项能力匹配
当前 STARTUP 序号；ReID 实际模型加载和预热失败时阻止 READY，不能直接 START。

## 推荐：由控制台统一启动

```bash
cd /mnt/nvme/workspace/dog_patrol
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/navigation/fast_livo_dog/config/fastdds_udp_only.xml"
export DOG_PATROL_VENV=/home/orin/venv/m20_nav
export ROS_LOG_DIR="$PWD/log/robot-console"
mkdir -p data/reid "$ROS_LOG_DIR"
/home/orin/venv/m20_nav/bin/python src/orchestration/robot_console/robot_console/app.py
```

按原有方式启动传感器、定位、导航和感知，准备路线后显式 START。启动感知时应看到
TensorRT engine 就绪和模型预加载完成，使用的特征维度为 1280。
模型已转换并放入感知资产包的 `reid/`，无需再次转换；配置中的 model_path 指向该目录的
`reid_clipreid_vitb16_after.fp16.engine`。数据和日志放在 NVMe 工作区，避免写入系统盘的默认 `/data/reid`。
完整感知栈自动读取 `runtime/reid.yaml` 并按 assets_root 设置 engine 路径；不再单独运行
`reid.launch.py`。此前若有手工启动的 ReID 服务，请自行先停止，避免与统一启动产生重复
节点/服务。现有运行进程不会因编译自动更新；下一次启动感知才使用新版本。

2026-10-12 资源整理后已验证：资产包 `SHA256SUMS` 全部 31 项通过；使用上述 runtime
配置在隔离 ROS domain 223、独立测试身份库中实际加载新路径 engine，依次验证新人、
已见未处置、跨 track 已处置、清除处置但保留已见四种结果，均通过。验证未重启生产节点，
未修改 `data/reid`。记录见 `build/pr51-review-88fa528/resources_service_smoke.log` 和
`resources_service_smoke_node.log`。

## 不使用控制台时：启动完整感知栈

```bash
cd /mnt/nvme/workspace/dog_patrol
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/navigation/fast_livo_dog/config/fastdds_udp_only.xml"
export DOG_PATROL_PYTHON=/home/orin/venv/m20_nav/bin/python
export ROS_LOG_DIR="$PWD/log/perception-runtime"
mkdir -p "$ROS_LOG_DIR" data/reid
ros2 launch dog_patrol_perception_bringup perception_stack.launch.py
```

此方式与控制台的感知按钮二选一，需要已有相机和 supervisor/导航链。所有进程的
ROS_DOMAIN_ID 必须一致。关闭完整感知 launch 会一起关闭 ReID，正常退出保存当日库；
ReID 关键进程意外退出也会关闭感知组。READY 是启动锁存，PAUSE/RESUME 不重新做模型检查。

## 统一启动与 READY 改动的验证（2026-10-12）

资源整理已先单独提交为 `75697a4`；本版在此基础上增加感知统一启动和 ReID READY 门控。
使用系统 `/usr/bin/colcon` 编译 provider、orchestrator 和 bringup 均成功。正式注册测试结果：
provider 6 项、orchestrator 50 项、bringup 2 项，均为 0 errors / 0 failures / 0 skipped；
另有 fake integration 脚本回归通过。provider 计数包含 CTest 及其内嵌 Python 节点用例，
不等同于独立业务场景数量。

真实 Orin GPU 节点测试覆盖缺失模型阻止整体 READY、在同一 STARTUP 补齐模型后重试成功、
reset 后新序号重新聚合、READY 后迟到旧 STARTUP 不生效、heartbeat 超时停止发布、同序号
冲突锁止及新序号恢复。生产 launch action 的实际退出测试验证 ReID 结束时关闭其同组进程。
所有测试均使用隔离域和独立身份库，没有重启现场相机/导航/感知，没有修改生产 `data/reid`。

复跑相关构建与测试（测试域与现场域 42 分开）：

```bash
cd /mnt/nvme/workspace/dog_patrol
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/navigation/fast_livo_dog/config/fastdds_udp_only.xml"
export ROS_LOG_DIR="$PWD/log/reid-acceptance"
export DOG_PATROL_REID_TEST_ENGINE="$PWD/src/perception/dog_patrol_perception_assets_20260813/reid/reid_clipreid_vitb16_after.fp16.engine"
/usr/bin/colcon build --packages-select dog_patrol_reid_provider \
  dog_patrol_perception_orchestrator dog_patrol_perception_bringup --symlink-install \
  --cmake-args -DREID_TEST_PYTHON=/home/orin/venv/m20_nav/bin/python
source install/setup.bash
/usr/bin/colcon test --packages-select dog_patrol_reid_provider \
  dog_patrol_perception_orchestrator dog_patrol_perception_bringup --executor sequential
```

关闭额外 pytest 插件自动加载用于避开本机无关插件依赖冲突，业务逻辑不为该环境问题变更。
日志在 `build/pr51-review-88fa528/unified_reid_final_colcon_tests.log`，GPU 用例和资源校验记录
同样保留在此目录。测试终端使用隔离域，现场启动请新开终端按前面的域 42 命令操作。

## 确认新服务实际在用

另开终端，加载相同 ROS 环境并设置 `ROS_DOMAIN_ID=42`：

```bash
cd /mnt/nvme/workspace/dog_patrol
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/navigation/fast_livo_dog/config/fastdds_udp_only.xml"

ros2 service list --no-daemon --spin-time 2 | grep -Fx '/perception/reid/dedup'
ros2 param get --no-daemon --spin-time 2 /perception_reid device
ros2 param get --no-daemon --spin-time 2 /perception_reid model_path
ros2 topic echo /perception/reid/status --once
```

预期后端是 `tensorrt`、模型路径是上面的 engine，状态 JSON 中 `model_loaded=1`、
`cloud_enabled=0`。开启感知时应有 `reid_confirm_gate 已启用` 的日志。

检查命令使用系统自带的 grep，无需安装 rg。若右侧过滤命令不存在，左侧 ros2 service list
可能因管道接收端退出而报 BrokenPipeError，这不代表 ReID 服务故障。上述 --no-daemon
绕过 CLI daemon 的图缓存，--spin-time 2 为当前查询留出两秒发现时间。单次 Node not found
表示这次参数命令的节点发现阶段没有找到目标；应重查服务、参数和状态，不必先重启节点。

2026-10-12 对正在运行的域 42 服务只读复查：服务存在，device=tensorrt，model_path 指向
已转换的正式 engine，model_loaded=1、cloud_enabled=0；未重启或修改运行节点。

建议按以下顺序观察：

1. READY、未 START 和 PAUSE 时不选业务目标、不发布确认事件。
2. 新人或见过但未核验通过的人照常进入核验；取消/失败不能仅凭“见过”免核验。
3. 核验通过后，同人再次具备选人资格时，ReID 应识别当天已处理并抑制再次确认。
4. 已核验通过的 A 和未通过的 B 同屏时，A 不能持续占用 primary，B 应能被选择。
5. PAUSE/RESUME、RETURN、TERMINATE 的原有行为正常。
6. 两轮核验失败进入 TRACK_INTRUDER，再因 TARGET_LOST 恢复巡逻后，handled_target_id
   应保持为 0；同人重新出现并具备选人资格时，应再次触发核验。

如果原 semantic ID 仍在基础排除表中，会直接被原有选人逻辑排除，不一定再次出现 ReID
问询日志；不要只按每帧是否有日志判断功能是否生效。重点观察是否重复核验及同屏新人是否被挡。

当日库按日期清理，可恢复同日已落盘的身份。当前验收按 tracking/ReID 一起运行使用，不以
单独重启验证故障恢复。在线 tracking 观察到显式 mission reset 的 STARTUP 转换时，会发送
handled 清理通知；测试“当天免重复处置”时应区分正常暂停/继续与完整 reset。停止测试时先在控制台
终止任务并停止感知，ReID 随感知组正常退出并保存当日库。

核验未完成时 PAUSE 不会为当前人员新增 handled 标记：可以已经 is_seen=true，但仍是
handled=false。暂停使业务进入 READY、清除当前 target_id、递增 state_seq 并取消认证；旧
认证结果不能在暂停后补记完成。若 supervisor 在暂停之前已经接受有效 AUTHORIZED，则原有
handled 会保留，暂停不会撤销已经成立的核验通过结果。

## 红绿框与“处理完成”标记

当前人体框色按画面中该人的 semantic_id 是否等于 primary_target_id 绘制：匹配主目标 ID
画红框，其余画绿框。绘制函数没有读取 ReID handled，也没有单独检查认证是否通过；保留
绑定 ID 的恢复状态也可能继续画红框。因此红框表示当前主目标身份，绿框表示非主目标，
不能从颜色直接得出这个人已经完成处置。

未处理的人恢复到 PATROL 后仍可被重新选择；再次成为主目标就会变红。PAUSE 的 READY、
暂停后 RECOVER_PATROL、另一个人成为主目标等情况下，他可以是绿框，handled 仍为 false。
当前图上没有独立的“已处理”文字或图标。

只有 supervisor 在 VERIFY_IDENTITY 接受有效 AUTHORIZED，才为当前人员新增已处理标记：

| 业务情况 | 当前人员是否产生新 handled 标记 |
| --- | --- |
| VERIFY_IDENTITY 接受有效 AUTHORIZED | 是 |
| UNAUTHORIZED 进入 TRACK_INTRUDER | 否 |
| TRACK_INTRUDER 接受最终 TARGET_LOST 并进入恢复 | 否；恢复巡逻后同人仍可再次核验 |
| CONFIRM/APPROACH/VERIFY 提前 TARGET_LOST | 否 |
| 任何阶段的 EXECUTION_ERROR | 不新增；恢复阶段保留此前核验通过的结果 |
| PAUSE/RESUME | 不新增；保留此前已经成立的结果 |

supervisor 发布 handled_target_id 后，生产 runtime 将新结果通过 /perception/reid/handled
转发，provider 按 track 绑定或可信外观匹配找到记录并置 handled=true。之后问询必须同时
满足 is_seen=true、handled=true 才免重复处置。这里的写入标记、选人排除和红绿框是不同
层次；/mission/state 的 handled_target_id 只表示最近的结论，不是所有已处理人员的列表。

## 重新构建

部署“仅核验通过才标记”的规则时，旧 ReID handled 记录只有布尔值，无法区分核验通过
和跟踪结束。重建并统一重启后，应让 tracking/ReID 一起运行，待 tracking 已观察到 READY
后，通过 `/mission/reset` 显式重进 STARTUP，同时清理 tracking 内存排除表与 ReID handled
标记，保留外观库；确认 ReID 输出“任务 reset：清除 handled 标记”日志。
单纯重启会恢复已落盘标记；只清 ReID 不能清除 tracking 的旧排除项。

感知使用系统 colcon，生产 runtime 和 TensorRT 均开启：

```bash
cd /mnt/nvme/workspace/dog_patrol
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=3 /usr/bin/colcon build \
  --packages-up-to dog_patrol_reid_provider dog_patrol_perception_tracking \
  --symlink-install \
  --cmake-args -DREID_BUILD_TENSORRT=ON -DTRACKING_ENABLE_ORIN_RUNTIME=ON -DBUILD_TESTING=ON
source install/setup.bash
```

该命令也会构建接口和 manager 依赖。工作区准备日志位于
`build/pr51-review-88fa528/workspace_build.log`；PR 复审证据见
[评审文档](../reviews/pr_50_reid_review.md)。

## 本次工作区准备结果（2026-10-11）

| 检查 | 结果 |
| --- | --- |
| 系统 colcon 构建 | 5 包成功：dog_patrol_interfaces、dog_patrol_perception_interfaces、dog_patrol_manager、dog_patrol_reid_provider、dog_patrol_perception_tracking |
| 生产配置 | REID_BUILD_TENSORRT=ON、TRACKING_ENABLE_ORIN_RUNTIME=ON，产物安装到当前工作区 install |
| manager | 70 项通过 |
| ReID provider | 2/2 CTest 通过 |
| tracking | 最后一次完整正式运行 58/58 CTest 通过，包含新增 19 项和原 adapter 19 项 gtest |
| 两个接口包 | 构建成功；m20_nav 的消息/service/supervisor 导入检查通过；接口包本身无注册测试 |
| 控制台 | m20_nav 执行 app.py --check-environment 成功，TASK_RUNNING=1、START=0 |
| 安装路径 | provider、tracking 均解析到 /mnt/nvme/workspace/dog_patrol/install；控制台、感知、导航和 supervisor 入口文件存在 |
| 真实模型服务 | 当前安装的 reid_node 在隔离 ROS_DOMAIN_ID=216 下实际加载正式 TensorRT 引擎，查询/handled/clear 通道验证通过 |
| 构建测试时分支 | feature/reid-dedup-gate=88fa528；当时 merge=47f7d48。验收后已合并并同步 origin/merge，业务代码一致 |

tracking 初次整包测试中，既有 test_target_image_ros_smoke 失败；单项复跑通过后，再执行
一次完整 tracking 正式测试，58/58 通过，test-result 为 0 errors、0 failures。保留初跑和
重跑日志，没有修改测试或业务源码来绕过失败。其余包的 test-result 均为零失败；CTest
包装层与 gtest XML 不重复累计为业务用例总数。

真实服务检查使用合成人体 crop 和独立测试库：首次查询 false/false，再次见过但未处置
true/false，标记处置并换 track 后 true/true，清除处置标记后 true/false。它验证部署链路，
不代表真实人员精度。测试库放在 build/pr51-review-88fa528 下，未写入实机使用的 data/reid；
该隔离服务已经正常停止并退出落盘。相机、导航和巡检由用户实机测试时启动。

主要日志均在 `build/pr51-review-88fa528/`：

- `workspace_build.log`：主工作区 5 包构建。
- `workspace_test.log`、`workspace_tracking_result.log`：首次测试及原始失败记录。
- `workspace_target_image_retry.log`：失败单项复跑。
- `workspace_tracking_retry.log`、`workspace_tracking_result_final.log`：tracking 最终完整测试及零失败结果。
- `workspace_service_smoke.log`、`workspace_service_smoke_node.log`：真实服务四步查询、GPU 加载和正常退出记录。
