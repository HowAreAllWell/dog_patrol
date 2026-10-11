// -*- coding: utf-8 -*-
#include "dog_patrol_reid_provider/cloud_library.hpp"

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <utility>

#include "dog_patrol_reid_provider/reid_gallery.hpp"  // cosine / l2_normalize

namespace dog_patrol_reid_provider {

namespace {
std::string default_today_string() {
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
}  // namespace

std::string CloudLibrary::today_string() { return default_today_string(); }

CloudLibrary::CloudLibrary(const Config& cfg,
                           std::function<std::string()> day_provider)
    : cfg_(cfg),
      day_provider_(day_provider ? std::move(day_provider)
                                 : [] { return default_today_string(); }),
      day_(day_provider_()) {}

bool CloudLibrary::check_daily_reset() {
  const std::string today = day_provider_();
  if (today == day_) {
    return false;
  }
  std::fprintf(stderr,
               "[cloud_library] 日期切换(%s -> %s)，云端库清零：%zu 人\n",
               day_.c_str(), today.c_str(), records_.size());
  records_.clear();
  seq_ = 0;
  day_ = today;
  return true;
}

size_t CloudLibrary::size() const { return records_.size(); }

std::vector<CloudSearchHit> CloudLibrary::search(
    const std::vector<float>& feat, float min_sim, size_t max_results) const {
  std::vector<CloudSearchHit> hits;
  if (feat.empty()) {
    return hits;
  }
  for (const auto& kv : records_) {
    if (kv.second.tmpl.empty()) {
      continue;
    }
    const float s = cosine(feat, kv.second.tmpl);
    if (s >= min_sim) {
      hits.push_back({kv.first, s});
    }
  }
  std::sort(hits.begin(), hits.end(),
            [](const CloudSearchHit& a, const CloudSearchHit& b) {
              return a.similarity > b.similarity;
            });
  if (hits.size() > max_results) {
    hits.resize(max_results);
  }
  return hits;
}

CloudLibrary::UpsertResult CloudLibrary::upsert(const std::vector<float>& feat,
                                                const std::string& origin_device,
                                                double now) {
  UpsertResult out;
  if (feat.empty()) {
    return out;
  }
  // 先按模板检索（与端侧一致：命中则并入，未命中则新建）
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
    CloudRecord& rec = records_[best_id];
    std::vector<float> merged(rec.tmpl.size());
    const double c = static_cast<double>(rec.merge_count);
    for (size_t i = 0; i < merged.size(); ++i) {
      merged[i] = static_cast<float>(c * static_cast<double>(rec.tmpl[i]) +
                                     static_cast<double>(feat[i]));
    }
    if (l2_normalize(merged)) {
      rec.tmpl = std::move(merged);
    }
    ++rec.merge_count;
    rec.last_seen = now;
    out.global_person_id = best_id;
    out.is_match = true;
    out.similarity = best;
    return out;
  }
  ++seq_;
  char buf[24];
  std::snprintf(buf, sizeof(buf), "global-person-%05u", seq_);
  CloudRecord rec;
  rec.global_person_id = buf;
  rec.tmpl = feat;
  l2_normalize(rec.tmpl);
  rec.merge_count = 1;
  rec.match_type = 0;  // 新建即"陌生人"
  rec.origin_device = origin_device;
  rec.first_seen = rec.last_seen = now;
  records_[rec.global_person_id] = std::move(rec);
  evict_if_needed(now);
  out.global_person_id = buf;
  out.is_match = false;
  return out;
}

bool CloudLibrary::set_match_type(const std::string& global_person_id,
                                  uint8_t match_type) {
  auto it = records_.find(global_person_id);
  if (it == records_.end()) {
    return false;
  }
  it->second.match_type = match_type;  // 仅改名单类型，不动活跃时间
  return true;
}

bool CloudLibrary::get(const std::string& global_person_id,
                       CloudRecord& out) const {
  auto it = records_.find(global_person_id);
  if (it == records_.end()) {
    return false;
  }
  out = it->second;
  return true;
}

void CloudLibrary::evict_if_needed(double now) {
  if (records_.size() <= cfg_.max_persons) {
    return;
  }
  while (records_.size() > cfg_.max_persons) {
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
  }
  (void)now;
}

}  // namespace dog_patrol_reid_provider
