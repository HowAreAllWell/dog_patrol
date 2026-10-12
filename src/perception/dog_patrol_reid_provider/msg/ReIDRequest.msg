# ReID 云端请求（负责人接口文档 v1.1 逐字定义，字段/顺序勿改）
# 发布方 ReID Node -> 订阅方 云端请求代理 Node，topic /reid/cloud/request
string version              # 接口版本，例如 1.0
string message_id           # 当前消息唯一 ID
string timestamp            # 请求时间，UTC ISO 8601
string device_id            # 设备唯一 ID
string node_name            # 发起请求的 ROS 2 Node 名称
string request_id           # 请求唯一 ID，响应必须原样返回
string task_id              # 当前巡检或导航任务 ID，可空
string event_id             # 来源感知事件 ID，可空
string track_id             # 本地人员跟踪 ID
string local_reid_id        # 本地 ReID 身份 ID，可空
string operation            # search | upsert | search_and_upsert
string thumbnail_url        # 人员缩略图文件 URL
string embedding_url        # ReID 特征向量文件 URL
uint32 embedding_dimension  # 特征向量维度
string embedding_dtype      # float32 | float16 | int8
string model_name           # ReID 模型名称
string model_version        # ReID 模型版本，云端兼容性校验
uint32 max_results          # 最大返回匹配数量
float32 min_similarity      # 最小相似度阈值 [0,1]
string other_info           # 扩展信息，JSON 序列化字符串，默认 "{}"
