# 导航模块

本目录包含当前 dog_patrol 使用的完整 FAST-LIVO-DOG 导航链，不只是路径规划：

- Livox/MVS 驱动、FAST-LIVO 建图与 2D/3D 定位；
- Nav2 planner、waypoint、Pure Pursuit、PRIEST/RL 局部路径和 DWB 控制适配；
- 机器狗底层控制接口 `/NAV_CMD`；
- `navigation_mission_coordinator`，负责感知目标框与雷达点云融合、目标接近和跟踪。

## 目录结构

```text
src/navigation/fast_livo_dog/
  config/                         设备标定、定位和导航参数
  data/                           2D/3D 地图及定位数据
  navigation/move/                Nav2、waypoint、局部路径和 NAV_CMD 链
  navigation/dog_patrol_navigation/ 任务导航协调器
```

导航运行时通过 `DOG_PATROL_NAV_ROOT` 指向
`src/navigation/fast_livo_dog`，不依赖外部 fast_livo_dog 工作区。

## 启动方式

整机推荐使用总控入口：

```bash
cd /mnt/nvme/workspace/dog_patrol
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/orchestration/robot_console/robot_console/app.py
```

在 UI 中按顺序启动传感器、建图或定位、感知任务和导航。`app.py` 负责模块启停，
同时保证任务 manager、感知和导航不会重复启动。

只调试导航时，可以手动启动：

```bash
export DOG_PATROL_NAV_ROOT=/mnt/nvme/workspace/dog_patrol/src/navigation/fast_livo_dog
ros2 launch move navigation.launch.py \
  use_sim_time:=false \
  params_file:=$DOG_PATROL_NAV_ROOT/config/nav_parameters.yaml
```

`navigation.launch.py` 默认包含 Nav2、move 外部控制链和导航任务协调器。不要再额外
启动一份 `navigation_mission_coordinator.launch.py`。

## 关键接口

```text
/livox/lidar -> FAST-LIVO / 定位 / 点云转 scan
/global_path -> Pure Pursuit -> /subgoal
/global_path + /subgoal -> PRIEST/RL -> /local_path
/local_path -> DWB adapter -> /NAV_CMD
```

任务协调器还使用：

```text
/mission/state
/perception/selected_target_bbox
/livox/lidar
/odom
       -> 目标地图位置 -> /compute_path_to_pose -> /global_path
```

当前坐标约定是：全局 `map`，平面导航 `base_footprint`，机身中心 `base_link`，雷达
`livox_frame`，目标框相机 `camera_link`。感知目标框默认对应
`/left_camera/image_raw` 的 `1280x1024` 原图。

## 主要配置

- 设备内参和外参：`fast_livo_dog/config/device_parameters.yaml`；
- FAST-LIVO、定位和 Nav2/move 参数：`fast_livo_dog/config/`；
- 任务协调器参数：
  `fast_livo_dog/navigation/dog_patrol_navigation/config/m20_patrol_navigation.yaml`。

详细的状态机、消息字段、目标融合、3 m 停止条件、启动时序和故障排查见：

[当前导航集成说明](fast_livo_dog/navigation/dog_patrol_navigation/docs/current_workspace_integration.md)

原 FAST-LIVO-DOG 的建图、定位、驱动和依赖说明见：

[FAST-LIVO-DOG 原始 README](fast_livo_dog/README.md)
