// -*- coding: utf-8 -*-
#include "dog_patrol_reid_provider/reid_gallery.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <utility>

namespace dog_patrol_reid_provider {

namespace {
std::string default_today_string() {
  // std::localtime 为 ISO C++ 标准接口，Windows/Linux 行为一致（调用方保证
  // 单线程上下文或自行加锁），不使用平台专用的 localtime_r/_s
  const std::time_t tt = std::time(nullptr);
  const std::tm* tm_ptr = std::localtime(&tt);
  if (tm_ptr == nullptr) {
    return "1970-01-01";
  }
  std::tm tm_buf = *tm_ptr;
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
                tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday);
  return std::string(buf);
}

// 单调时钟秒（仅用于库内部的相对时间：TTL/淘汰/加载基线重整）
double steady_now_sec() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// ---- 持久化二进制格式（v2）----
// 头：magic(8B) | version(u32) | day | model_name | model_version |
//     seq(u32) | count(u32)，字符串一律 u32 长度前缀 + 原始字节。
// 记录 ×count：local_reid_id | merge_count(u32) | match_type(u8) | handled(u8) |
//     global_person_id | dim(u32) | dim×f32 tmpl | first_seen(f64) | last_seen(f64)
// v1（无 handled 字节）直接按版本不符拒绝，不做迁移。
// 字节序：x86/ARM 开发机与 Orin 均为小端，直接按本机字节序读写。
constexpr char kPersistMagic[8] = {'R', 'E', 'I', 'D', 'L', 'B', '0', '1'};
constexpr uint32_t kPersistVersion = 2;
// 防御性上限：合法文件不该超过这些值，超了按损坏处理
constexpr uint32_t kMaxPersistRecords = 1000000;
constexpr uint32_t kMaxPersistStr = 4096;
constexpr uint32_t kMaxPersistDim = 8192;

bool wr_u32(std::ofstream& f, uint32_t v) {
  return static_cast<bool>(
      f.write(reinterpret_cast<const char*>(&v), sizeof v));
}
bool wr_u8(std::ofstream& f, uint8_t v) {
  return static_cast<bool>(
      f.write(reinterpret_cast<const char*>(&v), sizeof v));
}
bool wr_f64(std::ofstream& f, double v) {
  uint64_t bits = 0;
  std::memcpy(&bits, &v, sizeof bits);
  return static_cast<bool>(
      f.write(reinterpret_cast<const char*>(&bits), sizeof bits));
}
bool wr_str(std::ofstream& f, const std::string& s) {
  if (s.size() > kMaxPersistStr ||
      !wr_u32(f, static_cast<uint32_t>(s.size()))) {
    return false;
  }
  return s.empty() ||
         static_cast<bool>(f.write(s.data(),
                                   static_cast<std::streamsize>(s.size())));
}

bool rd_u32(std::ifstream& f, uint32_t& v) {
  return static_cast<bool>(
      f.read(reinterpret_cast<char*>(&v), sizeof v));
}
bool rd_u8(std::ifstream& f, uint8_t& v) {
  return static_cast<bool>(
      f.read(reinterpret_cast<char*>(&v), sizeof v));
}
bool rd_f64(std::ifstream& f, double& v) {
  uint64_t bits = 0;
  if (!f.read(reinterpret_cast<char*>(&bits), sizeof bits)) {
    return false;
  }
  std::memcpy(&v, &bits, sizeof v);
  return true;
}
bool rd_str(std::ifstream& f, std::string& s) {
  uint32_t len = 0;
  if (!rd_u32(f, len) || len > kMaxPersistStr) {
    return false;
  }
  s.resize(len);
  return len == 0 ||
         static_cast<bool>(f.read(&s[0], static_cast<std::streamsize>(len)));
}
}  // namespace

std::string IdentityGallery::today_string() { return default_today_string(); }

IdentityGallery::IdentityGallery(const Config& cfg,
                                 std::function<std::string()> day_provider)
    : cfg_(cfg),
      day_provider_(day_provider ? std::move(day_provider)
                                 : [] { return default_today_string(); }),
      day_(day_provider_()) {}

bool IdentityGallery::check_daily_reset() {
  const std::string today = day_provider_();
  if (today == day_) {
    return false;
  }
  std::fprintf(stderr,
               "[reid_gallery] 日期切换(%s -> %s)，端侧库清零：%zu 身份 / %zu track\n",
               day_.c_str(), today.c_str(), records_.size(), track_map_.size());
  records_.clear();
  track_map_.clear();
  seq_ = 0;
  day_ = today;
  dirty_ = true;
  return true;
}

size_t IdentityGallery::size() const { return records_.size(); }

SearchOutcome IdentityGallery::search(const std::vector<float>& feat,
                                      const std::string& track_id) const {
  SearchOutcome out;
  if (feat.empty()) {
    return out;
  }
  std::string best_id;
  float best = -1.0F;
  for (const auto& kv : records_) {
    if (kv.second.tmpl.empty()) {
      continue;
    }
    const float s = cosine(feat, kv.second.tmpl);
    if (s > best) {
      best = s;
      best_id = kv.first;
    }
  }
  if (!best_id.empty() && best >= cfg_.threshold) {
    out.hit = true;
    out.local_reid_id = best_id;
    out.similarity = best;
    return out;
  }
  // 身份阈值未过：查 track 映射兜底（同一 track 允许外观抖动，只要还认得出）
  auto it = track_map_.find(track_id);
  if (it == track_map_.end() || track_id.empty()) {
    return out;
  }
  auto rit = records_.find(it->second.first);
  if (rit == records_.end() || rit->second.tmpl.empty()) {
    return out;
  }
  const float s = cosine(feat, rit->second.tmpl);
  if (s >= cfg_.track_assoc_threshold) {
    out.hit = true;
    out.local_reid_id = rit->first;
    out.similarity = s;
    out.via_track = true;
  }
  return out;
}

std::string IdentityGallery::create(const std::vector<float>& feat,
                                    const std::string& track_id, double now) {
  ++seq_;
  char buf[24];
  std::snprintf(buf, sizeof(buf), "local-person-%05u", seq_);
  IdentityRecord rec;
  rec.local_reid_id = buf;
  rec.tmpl = feat;
  l2_normalize(rec.tmpl);
  rec.merge_count = feat.empty() ? 0 : 1;
  rec.first_seen = rec.last_seen = now;
  records_[rec.local_reid_id] = std::move(rec);
  if (!track_id.empty()) {
    track_map_[track_id] = {buf, now};
  }
  evict_if_needed(now);
  dirty_ = true;
  return buf;
}

float IdentityGallery::merge(const std::string& local_reid_id,
                             const std::vector<float>& feat,
                             const std::string& track_id, double now) {
  auto it = records_.find(local_reid_id);
  if (it == records_.end() || feat.empty() ||
      feat.size() != it->second.tmpl.size()) {
    return -1.0F;
  }
  IdentityRecord& rec = it->second;
  // 运行均值：new = normalize(count * old + feat)（old 已归一化）
  std::vector<float> merged(rec.tmpl.size());
  const double c = static_cast<double>(rec.merge_count);
  for (size_t i = 0; i < merged.size(); ++i) {
    merged[i] = static_cast<float>(c * static_cast<double>(rec.tmpl[i]) +
                                   static_cast<double>(feat[i]));
  }
  const float delta = 1.0F - cosine(rec.tmpl, merged);
  if (l2_normalize(merged)) {
    rec.tmpl = std::move(merged);
    ++rec.merge_count;
  } else {
    ++rec.merge_count;  // 均值退化（零向量）时保留旧模板，仅计数
  }
  rec.last_seen = now;
  if (!track_id.empty()) {
    track_map_[track_id] = {local_reid_id, now};
  }
  dirty_ = true;
  return delta;
}

bool IdentityGallery::bind_global(const std::string& local_reid_id,
                                  const std::string& global_person_id,
                                  uint8_t match_type) {
  auto it = records_.find(local_reid_id);
  if (it == records_.end() || global_person_id.empty()) {
    return false;
  }
  it->second.global_person_id = global_person_id;
  it->second.match_type = match_type;
  dirty_ = true;
  return true;
}

std::string IdentityGallery::upsert_global(const std::string& global_person_id,
                                           const std::vector<float>& tmpl,
                                           uint8_t match_type,
                                           uint32_t merge_count, double now) {
  if (global_person_id.empty() || tmpl.empty()) {
    return "";
  }
  for (auto& kv : records_) {
    if (kv.second.global_person_id == global_person_id) {
      IdentityRecord& rec = kv.second;
      rec.tmpl = tmpl;
      l2_normalize(rec.tmpl);
      rec.merge_count = merge_count;  // 云端计数覆盖：后续本地均值合并的权重基准
      rec.match_type = match_type;
      rec.last_seen = now;
      dirty_ = true;
      return kv.first;
    }
  }
  ++seq_;
  char buf[24];
  std::snprintf(buf, sizeof(buf), "local-person-%05u", seq_);
  IdentityRecord rec;
  rec.local_reid_id = buf;
  rec.tmpl = tmpl;
  l2_normalize(rec.tmpl);
  rec.merge_count = merge_count > 0 ? merge_count : 1;
  rec.global_person_id = global_person_id;
  rec.match_type = match_type;
  rec.first_seen = rec.last_seen = now;
  records_[buf] = std::move(rec);
  evict_if_needed(now);
  dirty_ = true;
  return buf;
}

bool IdentityGallery::set_handled_by_track(const std::string& track_id) {
  auto it = track_map_.find(track_id);
  if (it == track_map_.end()) {
    return false;
  }
  auto rit = records_.find(it->second.first);
  if (rit == records_.end()) {
    track_map_.erase(it);  // 悬挂映射清理
    return false;
  }
  rit->second.handled = true;
  dirty_ = true;
  return true;
}

bool IdentityGallery::set_handled_by_feature(const std::vector<float>& feat) {
  if (feat.empty()) {
    return false;
  }
  std::string best_id;
  float best = -1.0F;
  for (const auto& kv : records_) {
    if (kv.second.tmpl.empty()) {
      continue;
    }
    const float s = cosine(feat, kv.second.tmpl);
    if (s > best) {
      best = s;
      best_id = kv.first;
    }
  }
  if (best_id.empty() || best < cfg_.threshold) {
    return false;  // 定位不到可信身份：宁可不标，不误标他人
  }
  records_[best_id].handled = true;
  dirty_ = true;
  return true;
}

size_t IdentityGallery::clear_handled() {
  size_t n = 0;
  for (auto& kv : records_) {
    if (kv.second.handled) {
      kv.second.handled = false;
      ++n;
    }
  }
  if (n > 0) {
    dirty_ = true;
  }
  return n;
}

bool IdentityGallery::get(const std::string& local_reid_id,
                          IdentityRecord& out) const {
  auto it = records_.find(local_reid_id);
  if (it == records_.end()) {
    return false;
  }
  out = it->second;
  return true;
}

void IdentityGallery::prune_tracks(double now, double ttl_sec) {
  for (auto it = track_map_.begin(); it != track_map_.end();) {
    it = (now - it->second.second > ttl_sec) ? track_map_.erase(it)
                                             : std::next(it);
  }
}

void IdentityGallery::evict_if_needed(double now) {
  if (records_.size() <= cfg_.max_identities) {
    return;
  }
  while (records_.size() > cfg_.max_identities) {
    std::string oldest;
    double oldest_t = 1e18;
    for (const auto& kv : records_) {
      if (kv.second.last_seen < oldest_t) {
        oldest_t = kv.second.last_seen;
        oldest = kv.first;
      }
    }
    if (oldest.empty()) {
      break;
    }
    records_.erase(oldest);
    for (auto it = track_map_.begin(); it != track_map_.end();) {
      it = (it->second.first == oldest) ? track_map_.erase(it) : std::next(it);
    }
  }
  (void)now;
}

// ---------------------------------------------------------------- 持久化

bool IdentityGallery::SaveToFile(const std::string& path,
                                 const PersistMeta& meta) const {
  const std::string tmp = path + ".tmp";
  bool ok = false;
  {
    std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
    if (f) {
      ok = f.write(kPersistMagic, sizeof kPersistMagic) &&
           wr_u32(f, kPersistVersion) && wr_str(f, day_) &&
           wr_str(f, meta.model_name) && wr_str(f, meta.model_version) &&
           wr_u32(f, seq_) && wr_u32(f, static_cast<uint32_t>(records_.size()));
      for (const auto& kv : records_) {
        if (!ok) {
          break;
        }
        const IdentityRecord& r = kv.second;
        ok = wr_str(f, r.local_reid_id) && wr_u32(f, r.merge_count) &&
             wr_u8(f, r.match_type) && wr_u8(f, r.handled ? 1 : 0) &&
             wr_str(f, r.global_person_id) &&
             wr_u32(f, static_cast<uint32_t>(r.tmpl.size())) &&
             (r.tmpl.empty() ||
              f.write(reinterpret_cast<const char*>(r.tmpl.data()),
                      static_cast<std::streamsize>(r.tmpl.size() *
                                                   sizeof(float)))) &&
             wr_f64(f, r.first_seen) && wr_f64(f, r.last_seen);
      }
      f.flush();
      ok = ok && static_cast<bool>(f);
    }
  }
  if (!ok) {
    std::remove(tmp.c_str());
    return false;
  }
  // 原子替换：Linux/POSIX 的 rename 直接原子覆盖已存在目标（Orin 走此路径，
  // 不存在"旧文件已删、新文件未改名"的窗口）；Windows 的 rename 不覆盖，
  // 只能先删旧（仅影响开发机）。替换失败时保留临时文件与旧正式文件
  //——正式文件仍是上一份完整快照，下次落盘或 LoadLatest 可恢复。
#ifdef _WIN32
  std::remove(path.c_str());
#endif
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    return false;
  }
  dirty_ = false;
  return true;
}

IdentityGallery::LoadResult IdentityGallery::LoadFromFile(
    const std::string& path, const PersistMeta& meta) {
  LoadResult res;
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    res.status = LoadStatus::kMissing;
    return res;
  }
  char magic[sizeof kPersistMagic];
  if (!f.read(magic, sizeof magic) ||
      std::memcmp(magic, kPersistMagic, sizeof magic) != 0) {
    res.status = LoadStatus::kCorrupt;
    return res;
  }
  uint32_t version = 0;
  std::string f_day, f_model, f_mver;
  if (!rd_u32(f, version) || version != kPersistVersion || !rd_str(f, f_day) ||
      !rd_str(f, f_model) || !rd_str(f, f_mver)) {
    res.status = LoadStatus::kCorrupt;
    return res;
  }
  if (f_day != day_) {
    res.status = LoadStatus::kWrongDay;
    return res;
  }
  if (f_model != meta.model_name || f_mver != meta.model_version) {
    res.status = LoadStatus::kModelMismatch;
    return res;
  }
  uint32_t seq = 0, count = 0;
  if (!rd_u32(f, seq) || !rd_u32(f, count) || count > kMaxPersistRecords) {
    res.status = LoadStatus::kCorrupt;
    return res;
  }
  std::map<std::string, IdentityRecord> loaded;
  for (uint32_t i = 0; i < count; ++i) {
    IdentityRecord r;
    uint32_t merge_count = 0, dim = 0;
    uint8_t match_type = 0, handled = 0;
    double first_seen = 0.0, last_seen = 0.0;
    if (!rd_str(f, r.local_reid_id) || !rd_u32(f, merge_count) ||
        !rd_u8(f, match_type) || !rd_u8(f, handled) ||
        !rd_str(f, r.global_person_id) || !rd_u32(f, dim) || dim > kMaxPersistDim) {
      res.status = LoadStatus::kCorrupt;
      return res;
    }
    r.tmpl.resize(dim);
    if (dim > 0 &&
        !f.read(reinterpret_cast<char*>(r.tmpl.data()),
                static_cast<std::streamsize>(dim * sizeof(float)))) {
      res.status = LoadStatus::kCorrupt;
      return res;
    }
    if (!rd_f64(f, first_seen) || !rd_f64(f, last_seen) ||
        r.local_reid_id.empty()) {
      res.status = LoadStatus::kCorrupt;
      return res;
    }
    r.merge_count = merge_count;
    r.match_type = match_type;
    r.handled = handled != 0;
    r.first_seen = first_seen;
    r.last_seen = last_seen;
    loaded[r.local_reid_id] = std::move(r);
  }
  if (loaded.size() != count) {  // 存在重复 id，视为损坏
    res.status = LoadStatus::kCorrupt;
    return res;
  }
  records_ = std::move(loaded);
  track_map_.clear();  // track 映射不持久化：重启后旧 track_id 语义已失效
  // 落盘的时间戳来自（可能的）上次开机期间的 steady_clock，整机重启后单调
  // 时钟归零，旧值与本次开机的新记录不可比（会把容量淘汰顺序弄反：专删新人）。
  // 统一重整到加载时刻的同一基线；此后每次命中 merge 都会用当前时钟刷新
  // last_seen，运行期内的相对顺序不受影响。
  const double anchor = steady_now_sec();
  for (auto& kv : records_) {
    kv.second.first_seen = anchor;
    kv.second.last_seen = anchor;
  }
  seq_ = seq;
  dirty_ = false;
  res.status = LoadStatus::kOk;
  res.records = records_.size();
  return res;
}

IdentityGallery::LoadResult IdentityGallery::LoadLatest(
    const std::string& path, const PersistMeta& meta) {
  LoadResult res = LoadFromFile(path, meta);
  if (res.status != LoadStatus::kMissing) {
    return res;
  }
  // 正式文件缺失（替换被中断的极端情形）：尝试残留的 .tmp，有效则采纳
  // 并尽力提升为正式文件（此时库内容已从 .tmp 加载，改名失败不影响内存态）。
  const std::string tmp = path + ".tmp";
  LoadResult tmp_res = LoadFromFile(tmp, meta);
  if (tmp_res.status != LoadStatus::kOk) {
    return res;
  }
#ifdef _WIN32
  std::remove(path.c_str());
#endif
  std::rename(tmp.c_str(), path.c_str());
  tmp_res.from_tmp = true;
  return tmp_res;
}

float cosine(const std::vector<float>& a, const std::vector<float>& b) {
  if (a.size() != b.size() || a.empty()) {
    return -1.0F;
  }
  double dot = 0.0, na = 0.0, nb = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    dot += static_cast<double>(a[i]) * b[i];
    na += static_cast<double>(a[i]) * a[i];
    nb += static_cast<double>(b[i]) * b[i];
  }
  na = std::sqrt(na);
  nb = std::sqrt(nb);
  if (na < 1e-12 || nb < 1e-12) {
    return -1.0F;
  }
  return static_cast<float>(dot / (na * nb));
}

bool l2_normalize(std::vector<float>& v) {
  double s = 0.0;
  for (float x : v) s += static_cast<double>(x) * x;
  s = std::sqrt(s);
  if (s < 1e-12) {
    return false;
  }
  const float inv = static_cast<float>(1.0 / s);
  for (float& x : v) x *= inv;
  return true;
}

}  // namespace dog_patrol_reid_provider
