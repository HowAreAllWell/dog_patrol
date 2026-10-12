# 云端 -> 端侧 库更新扇出（我们的扩展接口：负责人文档未定义，待对齐）
# 发布方 云端请求代理 Node，topic /reid/cloud/library_update，所有狗的 ReID 节点订阅
# 设计说明：跨设备无法共享文件 URL，故向量内联传递（1280 维 float32 序列化约 5KB）
string version              # 接口版本
string message_id           # 消息唯一 ID
string timestamp            # UTC ISO 8601
string device_id            # 触发本次更新的来源设备 ID（接收方跳过自己）
string day                  # 云端库当前本地日期 yyyy-mm-dd；与接收方当日不一致则丢弃
string global_person_id     # 云端全局身份 ID
float32[] embedding         # 云端权威模板（L2 归一化运行均值；维度由数组长度给出）
uint32 merge_count          # 云端模板累计合并次数（接收方覆盖本地计数，保证均值权重正确）
uint8 match_type            # 0 之前的陌生人(含核验失败者) 1 白名单 2 不确定 255 未知
string model_name           # 模型兼容校验：不同模型的向量不可比
string model_version
string other_info           # JSON 序列化字符串，默认 "{}"
