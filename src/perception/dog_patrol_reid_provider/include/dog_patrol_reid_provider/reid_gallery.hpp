// -*- coding: utf-8 -*-
// 端侧身份去重库：当天有效、每日凌晨清零（云端库同样每日清零，由云端侧服务负责）。
// 每个身份保存一个 L2 归一化的"运行均值模板"（merge 后重新归一化），
// 并维护 track_id -> 身份 的映射（带 TTL），防止同一 track 因光照抖动裂成多个身份。
// 纯 C++（仅标准库），不依赖 ROS/OpenCV，便于独立编译自测；线程安全由调用方加锁。
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace dog_patrol_reid_provider {

struct IdentityRecord {
  std::string local_reid_id;     // 如 local-person-00001，当日自增，跨天归零
  std::vector<float> tmpl;       // L2 归一化运行均值模板
  uint32_t merge_count = 0;      // 已并入的特征数
  std::string global_person_id;  // 云端全局身份；"" 表示未绑定
  uint8_t match_type = 255;      // 云端回填的名单类型（0 陌生/1 白/2 不确定/255 未知）
  bool handled = false;          // 任务侧已处置结论（/perception/reid/handled 落地）；
                                 //   仅见过未处置的身份为 false，不能据此免处置
  double first_seen = 0.0;       // steady 时钟秒
  double last_seen = 0.0;
};

struct SearchOutcome {
  bool hit = false;
  std::string local_reid_id;
  float similarity = -1.0F;
  bool via_track = false;        // 未过身份阈值但经 track 映射兜底命中
};

class IdentityGallery {
 public:
  struct Config {
    float threshold = 0.70F;             // 身份命中余弦阈值（ViT 标定 0.70）
    float track_assoc_threshold = 0.50F; // track 兜底关联阈值（低于身份阈值）
    size_t max_identities = 10000;       // 端侧库容量上限，超出按最久未见淘汰
  };

  // day_provider 可注入假日期，用于测试跨天清零；默认取系统本地日期。
  // cfg 不设默认实参：嵌套类 NSDMI 与构造默认实参同用在新版 GCC 下不合法，
  // 显式传参在所有平台一致。
  explicit IdentityGallery(const Config& cfg,
                           std::function<std::string()> day_provider = nullptr);

  // 每次访问前调用；跨天返回 true（库/映射/序号已清零）
  bool check_daily_reset();

  size_t size() const;

  // 检索：先按模板余弦比对（>=threshold 命中）；未命中时若 track_id 已映射且
  // 与该身份相似度 >= track_assoc_threshold，判为同一身份（防 track 裂化）
  SearchOutcome search(const std::vector<float>& feat,
                       const std::string& track_id) const;

  // 新建身份并建立 track 映射，返回 local_reid_id
  std::string create(const std::vector<float>& feat, const std::string& track_id,
                     double now);

  // 命中后并入特征：模板 = normalize(count*旧模板 + feat)，count++，
  // 刷新 last_seen 与 track 映射。返回模板变化量 1-dot(旧,新)（供调用方
  // 判断是否值得回推云端）；找不到身份返回 -1
  float merge(const std::string& local_reid_id, const std::vector<float>& feat,
              const std::string& track_id, double now);

  // 绑定云端全局身份（云端命中或迟到响应）；match_type 一并回填
  bool bind_global(const std::string& local_reid_id,
                   const std::string& global_person_id, uint8_t match_type);

  // 云端扇出更新落地：本地已有绑定该全局 ID 的身份 -> 用云端权威模板覆盖
  //（云端是合并权威，含其他狗的贡献；本地未回推的小增量会在下次命中时补上）；
  // 没有 -> 新建本地身份并直接绑定。返回 local_reid_id
  std::string upsert_global(const std::string& global_person_id,
                            const std::vector<float>& tmpl,
                            uint8_t match_type, uint32_t merge_count,
                            double now);

  // ---- handled（已处置）结论落地（R06-A：业务结论与外观库分开维护）----
  // 按 track 绑定定位身份并标记 handled；绑定不存在/已过期返回 false
  bool set_handled_by_track(const std::string& track_id);
  // 兜底：按外观定位（最高相似度且 >= 身份阈值才标记，避免误标他人）
  bool set_handled_by_feature(const std::vector<float>& feat);
  // 任务 reset/新会话：清除全部 handled 标记（身份与模板保留），返回清除数
  size_t clear_handled();

  bool get(const std::string& local_reid_id, IdentityRecord& out) const;

  // 清理超过 ttl 未活跃的 track 映射（身份记录保留至每日清零/容量淘汰）
  void prune_tracks(double now, double ttl_sec);

  static std::string today_string();  // 本地日期 "yyyy-mm-dd"

  // ---- 当日库持久化（应对巡逻机器人每天多次开关机：重启恢复、跨天作废）----
  struct PersistMeta {
    std::string model_name;     // 模型/版本不符的旧文件不加载（特征不可比）
    std::string model_version;
  };
  enum class LoadStatus { kOk, kMissing, kWrongDay, kModelMismatch, kCorrupt };
  struct LoadResult {
    LoadStatus status = LoadStatus::kMissing;
    size_t records = 0;
    bool from_tmp = false;       // 正式文件缺失时从残留 .tmp 恢复并已提升
  };

  // 自上次成功落盘后库是否有变化（调用方据此防抖落盘）
  bool dirty() const { return dirty_; }

  // 全量落盘：写临时文件后原子替换（Linux rename 原子覆盖；Windows 先删旧，
  // 仅开发机），成功清除 dirty；目录不存在/写失败返回 false（调用方降级为
  // 不持久化，不影响运行中的库）。track 映射不落盘（重启后 track_id 语义
  // 已失效，靠特征阈值重新关联）。
  bool SaveToFile(const std::string& path, const PersistMeta& meta) const;

  // 启动时加载：仅当文件为当日且模型匹配才采纳（先读入临时容器，全部校验
  // 通过才提交，损坏/不符时库保持原状不动）。加载时把记录时间重整到本次
  // 开机的单调时钟基线（跨整机重启后旧时间戳不可比，见 cpp 注释）。
  LoadResult LoadFromFile(const std::string& path, const PersistMeta& meta);

  // LoadFromFile 的启动入口：正式文件缺失时尝试残留 .tmp（替换被中断的
  // 极端情形），有效则采纳并提升为正式文件。
  LoadResult LoadLatest(const std::string& path, const PersistMeta& meta);

 private:
  void evict_if_needed(double now);

  Config cfg_;
  std::map<std::string, IdentityRecord> records_;  // key = local_reid_id
  std::map<std::string, std::pair<std::string, double>> track_map_;  // track -> (id, last_seen)
  std::function<std::string()> day_provider_;
  std::string day_;
  uint32_t seq_ = 0;
  mutable bool dirty_ = false;
};

// 余弦相似度（假定输入已 L2 归一化；内部仍做防御性归一化）
float cosine(const std::vector<float>& a, const std::vector<float>& b);
// 就地 L2 归一化；范数过小返回 false（不动原向量）
bool l2_normalize(std::vector<float>& v);

}  // namespace dog_patrol_reid_provider
