// -*- coding: utf-8 -*-
// 云端全局人员库（运行在云端请求代理节点内）：当天有效、每日凌晨清零，
// 模板为所有设备贡献的运行均值（与端侧同一套数学）。
// 纯 C++（仅标准库），不依赖 ROS/OpenCV，便于独立编译自测；线程安全由调用方加锁。
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace dog_patrol_reid_provider {

struct CloudRecord {
  std::string global_person_id;  // 如 global-person-00001，当日自增，跨天归零
  std::vector<float> tmpl;       // L2 归一化运行均值模板
  uint32_t merge_count = 0;
  uint8_t match_type = 0;        // 新建即"陌生人"（0）；将来人脸结果经 set_match_type 落库
  std::string origin_device;     // 首个贡献该身份的设备
  double first_seen = 0.0;
  double last_seen = 0.0;
};

struct CloudSearchHit {
  std::string global_person_id;
  float similarity = -1.0F;
};

class CloudLibrary {
 public:
  struct Config {
    float threshold = 0.70F;      // 云端命中阈值（与端侧一致）
    size_t max_persons = 20000;   // 容量上限（20000 x 1280 维 float32 约 100MB）
  };

  // day_provider 可注入假日期用于测试；cfg 不设默认实参（平台一致的 C++ 写法）
  explicit CloudLibrary(const Config& cfg,
                        std::function<std::string()> day_provider = nullptr);

  // 每次访问前调用；跨天返回 true（库与序号已清零）
  bool check_daily_reset();

  size_t size() const;

  // 检索（不写库）：返回相似度 >= min_sim 的前 max_results 条，按相似度降序
  std::vector<CloudSearchHit> search(const std::vector<float>& feat,
                                     float min_sim, size_t max_results) const;

  struct UpsertResult {
    std::string global_person_id;  // 合并进/新建的云端身份
    bool is_match = false;         // true=并入已有人员 false=新建
    float similarity = -1.0F;      // is_match 时的命中相似度
  };
  // upsert：命中已有 -> 模板运行均值合并；未命中 -> 新建。
  // 返回的 global_person_id 在两种情况下都有效（新建即分配）
  UpsertResult upsert(const std::vector<float>& feat,
                      const std::string& origin_device, double now);

  // 将来人脸核验结论落库的入口（黑白扩展位）
  bool set_match_type(const std::string& global_person_id, uint8_t match_type);

  bool get(const std::string& global_person_id, CloudRecord& out) const;

  static std::string today_string();

 private:
  void evict_if_needed(double now);

  Config cfg_;
  std::map<std::string, CloudRecord> records_;  // key = global_person_id
  std::function<std::string()> day_provider_;
  std::string day_;
  uint32_t seq_ = 0;
};

}  // namespace dog_patrol_reid_provider
