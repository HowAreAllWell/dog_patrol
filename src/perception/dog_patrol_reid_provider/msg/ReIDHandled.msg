# 感知侧 -> ReID 节点：handled（已完成处置）结论下发（我们的扩展接口）
# 语义（R06-A 方案）：谁算"处置完成"由任务侧 supervisor 决定；ReID 收到本
# 消息后把对应身份标记 handled=true，之后同人问询（含换 track_id 重新出现、
# 重启后恢复）命中时在 dedup 响应中回 handled=true，由调用方决定排除/放行。
# 外观身份库与业务处置结论分开维护：未收到 handled 的身份 is_seen=true 也
# 不代表免处置。每日清零随库自然失效。
std_msgs/Header header      # stamp=结论下发时刻
string track_id             # 已完成处置的语义 track_id（与 dedup 问询同口径）
sensor_msgs/Image crop      # 可选兜底：track 绑定过期时按外观（身份阈值）定位
bool clear_all              # true=清除全部 handled 标记（任务 reset/新会话），
                            #   此时其余字段忽略
