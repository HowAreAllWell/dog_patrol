# dog_patrol_perception_face

人脸验证算法在主仓中的唯一正式接入目录。当前只建立可构建的 ROS 2 package 骨架和协作边界，
尚未迁入算法、生产 provider、readiness、模型、白名单或人脸预览 overlay。

后续实现必须遵守 [`COLLABORATION.md`](COLLABORATION.md)。核心输入是 tracking 发布的
`dog_patrol_perception_interfaces/msg/TrackedTargetImage`，默认 topic 为
`/perception/tracked_target_image`；算法不得自行读取相机或重新选择主目标。

本包当前没有可执行入口，不能发布 `face` READY 或 `AuthorizationEvidence`，也不能作为生产人脸
能力已接入的证据。模型、人脸白名单、特征向量、现场图像和录制数据不得提交到公开仓库。
