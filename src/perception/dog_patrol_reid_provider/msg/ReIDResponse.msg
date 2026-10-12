# ReID 云端响应（负责人接口文档 v1.1 逐字定义）
# 发布方 云端请求代理 Node -> 订阅方 ReID Node，topic /reid/cloud/response
string version
string message_id
string timestamp
string device_id
string node_name
string request_id           # 对应请求 ID
string status               # succeeded | failed | timeout | rejected
string global_person_id     # 云端全局人员 ID，可空
bool is_match               # 是否命中已有人员
uint8 match_type            # 0 命中之前的陌生人(含核验失败者) | 1 白名单 | 2 不确定 | 255 未知
                            # 注意：负责人文档的字段表里有此字段但 msg 定义遗漏，本包按表补上
float32 similarity          # 最高匹配相似度，未命中 -1
Match[] matches             # 候选匹配结果列表
string error_code           # 成功时空串
string error_message        # 成功时空串
string other_info
