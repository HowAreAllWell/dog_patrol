// -*- coding: utf-8 -*-
// 端侧库 + 云端请求关联器的独立逻辑自测（不依赖 ROS/OpenCV）。
// 编译运行（任一有 g++ 的机器）：
//   g++ -std=c++17 -Iinclude test/test_gallery_main.cpp src/reid_gallery.cpp -o test_gallery && ./test_gallery
// 或在 ROS 工作空间内 colcon build 后运行 build/dog_patrol_reid_provider/test_reid_gallery
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "dog_patrol_reid_provider/cloud_library.hpp"
#include "dog_patrol_reid_provider/reid_gallery.hpp"
#include <thread>

#include "dog_patrol_reid_provider/request_tracker.hpp"

using dog_patrol_reid_provider::CloudLibrary;
using dog_patrol_reid_provider::CloudRecord;
using dog_patrol_reid_provider::CloudSearchHit;
using dog_patrol_reid_provider::CloudOutcome;
using dog_patrol_reid_provider::IdentityGallery;
using dog_patrol_reid_provider::IdentityRecord;
using dog_patrol_reid_provider::PendingCloud;
using dog_patrol_reid_provider::RequestTracker;
using dog_patrol_reid_provider::SearchOutcome;

static int g_fail = 0, g_total = 0;
#define CHECK(cond)                                                  \
  do {                                                               \
    ++g_total;                                                       \
    if (!(cond)) {                                                   \
      ++g_fail;                                                      \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);    \
    }                                                                \
  } while (0)
#define CHECK_NEAR(a, b, tol)                                            \
  do {                                                                   \
    ++g_total;                                                           \
    const double d_ = std::fabs((a) - (b));                              \
    if (!(d_ <= (tol))) {                                                \
      ++g_fail;                                                          \
      std::printf("FAIL %s:%d  |%s - %s| = %g > %g\n", __FILE__,         \
                  __LINE__, #a, #b, d_, static_cast<double>(tol));       \
    }                                                                    \
  } while (0)

// 测试辅助：经 track 兜底命中反查 local id（f060 对该身份 cos=0.6 唯一）
static std::string id_of(const IdentityGallery& g, const std::string& track);

namespace {
constexpr size_t kD = 8;

std::vector<float> unit(size_t i) {
  std::vector<float> v(kD, 0.0F);
  v[i] = 1.0F;
  return v;
}
// normalize(a*u_i + b*u_j)，与 u_i 的余弦 = a/sqrt(a^2+b^2)
std::vector<float> blend(double a, size_t i, double b, size_t j) {
  std::vector<float> v(kD, 0.0F);
  v[i] = static_cast<float>(a);
  v[j] = static_cast<float>(b);
  dog_patrol_reid_provider::l2_normalize(v);
  return v;
}
}  // namespace

static void test_gallery_basic() {
  IdentityGallery::Config cfg;
  IdentityGallery g(cfg);
  const double t0 = 100.0;

  CHECK(g.create(unit(0), "t1", t0) == "local-person-00001");
  CHECK(g.create(unit(1), "t2", t0) == "local-person-00002");
  CHECK(g.size() == 2);

  // 同人轻微变化（cos≈0.995）应命中
  SearchOutcome sr = g.search(blend(1.0, 0, 0.1, 1), "zz");
  CHECK(sr.hit);
  CHECK(sr.local_reid_id == "local-person-00001");
  CHECK(sr.similarity > 0.99F);
  CHECK(!sr.via_track);

  // 无关特征应未命中
  CHECK(!g.search(unit(7), "zz").hit);
}

static void test_track_assoc() {
  IdentityGallery::Config cfg;
  IdentityGallery g(cfg);
  g.create(unit(0), "t1", 0.0);

  // cos=0.6：低于身份阈值 0.70，但同一 track 且 >=0.50 应兜底命中
  const std::vector<float> f060 = blend(3.0, 0, 4.0, 2);
  SearchOutcome sr = g.search(f060, "t1");
  CHECK(sr.hit);
  CHECK(sr.via_track);
  CHECK_NEAR(sr.similarity, 0.6, 0.01);

  // 换个 track 就不认
  CHECK(!g.search(f060, "other").hit);
}

static void test_merge_running_mean() {
  IdentityGallery::Config cfg;
  IdentityGallery g(cfg);
  const std::string id = g.create(unit(0), "t1", 0.0);

  // 均值：normalize(1*e1 + e3) -> 各分量 1/sqrt(2)，delta = 1-1/sqrt(2)
  const float delta = g.merge(id, unit(2), "t1", 10.0);
  CHECK_NEAR(delta, 1.0 - 1.0 / std::sqrt(2.0), 1e-3);
  IdentityRecord rec;
  CHECK(g.get(id, rec));
  CHECK(rec.merge_count == 2);
  CHECK_NEAR(rec.tmpl[0], 1.0 / std::sqrt(2.0), 1e-4);
  CHECK_NEAR(rec.tmpl[2], 1.0 / std::sqrt(2.0), 1e-4);

  // 再并入同方向特征：模板不动，delta≈0（节点侧据此不回推云端）
  const float d2 = g.merge(id, blend(1.0, 0, 1.0, 2), "t1", 20.0);
  CHECK(d2 < 0.001F);
  CHECK(g.get(id, rec));
  CHECK(rec.merge_count == 3);
  CHECK_NEAR(rec.tmpl[0], 1.0 / std::sqrt(2.0), 1e-4);

  // 未知身份
  CHECK(g.merge("local-person-09999", unit(0), "t1", 20.0) < 0.0F);
}

static void test_bind_global() {
  IdentityGallery::Config cfg;
  IdentityGallery g(cfg);
  const std::string id = g.create(unit(0), "t1", 0.0);
  CHECK(g.bind_global(id, "global-person-1042", 1));
  IdentityRecord rec;
  CHECK(g.get(id, rec));
  CHECK(rec.global_person_id == "global-person-1042");
  CHECK(rec.match_type == 1);
  CHECK(!g.get("local-person-09999", rec));
  CHECK(!g.bind_global("local-person-09999", "g", 0));
  CHECK(!g.bind_global(id, "", 0));  // 空全局 ID 拒绝
}

static void test_daily_reset() {
  auto day = std::make_shared<std::string>("2026-09-28");
  IdentityGallery::Config cfg;
  IdentityGallery g(cfg, [day] { return *day; });
  g.create(unit(0), "t1", 0.0);
  g.create(unit(1), "t2", 0.0);
  CHECK(g.check_daily_reset() == false);  // 同日不清

  *day = "2026-09-29";
  CHECK(g.check_daily_reset() == true);
  CHECK(g.size() == 0);
  CHECK(!g.search(unit(0), "t1").hit);                 // 身份没了
  CHECK(!g.search(blend(3, 0, 4, 2), "t1").hit);       // track 映射也没了
  CHECK(g.create(unit(0), "t1", 1.0) == "local-person-00001");  // 序号重计
  CHECK(g.check_daily_reset() == false);
}

static void test_track_ttl() {
  IdentityGallery::Config cfg;
  IdentityGallery g(cfg);
  g.create(unit(0), "t1", 0.0);
  const std::vector<float> f060 = blend(3.0, 0, 4.0, 2);
  CHECK(g.search(f060, "t1").hit);

  // 用轻微变化的同人特征 merge：既刷新 track 活跃时间，模板又几乎不动
  // （f060 与模板的余弦保持在 0.5~0.7 之间，只能靠 track 兜底命中）
  g.merge(id_of(g, "t1"), blend(1.0, 0, 0.05, 1), "t1", 100.0);
  g.prune_tracks(350.0, 300.0);                // 350-100=250 < 300，保留
  SearchOutcome sr = g.search(f060, "t1");
  CHECK(sr.hit);
  CHECK(sr.via_track);                         // 确认仍是 track 兜底而非特征命中
  g.prune_tracks(500.0, 300.0);                // 500-100=400 > 300，清掉
  CHECK(!g.search(f060, "t1").hit);
  // 身份本体不受影响：高相似仍命中
  CHECK(g.search(unit(0), "zz").hit);
}

static void test_evict() {
  IdentityGallery::Config cfg;
  cfg.max_identities = 2;
  IdentityGallery g(cfg);
  g.create(unit(0), "t1", 0.0);
  g.create(unit(1), "t2", 1.0);
  g.create(unit(5), "t3", 2.0);  // 超容，最久未见的第 1 个被淘汰
  CHECK(g.size() == 2);
  CHECK(!g.search(unit(0), "t1").hit);
  // f060 只与 dim0/dim2 相关，幸存的 t2(dim1)/t3(dim5) 都对不上，
  // 而 t1 的 track 映射已随淘汰清除 -> 必须完全不命中
  CHECK(!g.search(blend(3.0, 0, 4.0, 2), "t1").hit);
  CHECK(g.search(unit(1), "t2").hit);
}

static void test_threshold_respected() {
  IdentityGallery::Config cfg;
  cfg.threshold = 0.75F;  // 默认 track 兜底 0.50 不变
  IdentityGallery g(cfg);
  g.create(unit(0), "t1", 0.0);
  // 0.6 < 0.75 未过身份阈值；track 对不上时必须完全不命中
  CHECK(!g.search(blend(3.0, 0, 4.0, 2), "zz").hit);
}

static void test_tracker_basic() {
  RequestTracker tr;
  CloudOutcome oc;
  CHECK(!tr.poll("r1", oc));  // 未登记

  tr.add("r1", "local-person-00001", "t1", 10.0, 70.0);
  CHECK(!tr.poll("r1", oc));  // 尚无响应

  CloudOutcome in;
  in.succeeded = true;
  in.is_match = true;
  in.similarity = 0.88F;
  in.global_person_id = "global-person-1042";
  in.match_type = 0;
  PendingCloud info;
  CHECK(tr.on_response("r1", in, info));
  CHECK(info.local_reid_id == "local-person-00001");

  CHECK(tr.poll("r1", oc));  // 取到
  CHECK(oc.is_match);
  CHECK(oc.global_person_id == "global-person-1042");
  CHECK_NEAR(oc.similarity, 0.88F, 1e-6);
  CHECK(!tr.poll("r1", oc));               // 已取走
  CHECK(!tr.on_response("r1", in, info));  // 重复响应忽略
}

static void test_tracker_timeout_and_late() {
  RequestTracker tr;
  tr.add("r1", "local-person-00001", "t1", 10.0, 70.0);
  CHECK(tr.mark_timeout("r1"));   // service 按超时应答
  CHECK(!tr.mark_timeout("r1"));  // 幂等

  CloudOutcome in;  // 迟到响应：仍能拿到上下文做绑定
  in.succeeded = true;
  in.is_match = true;
  in.global_person_id = "g-9";
  PendingCloud info;
  CHECK(tr.on_response("r1", in, info));
  CHECK(info.timed_out_reported);
  CHECK(tr.size() == 1);              // 无人 poll，等 sweep
  CHECK(tr.sweep(71.0).size() == 1);  // 过了 expire_at 清掉
  CHECK(tr.size() == 0);
}

static void test_tracker_fire_and_forget() {
  RequestTracker tr;
  tr.add("r1", "local-person-00001", "t1", 0.0, 60.0);  // 不等待
  CloudOutcome in;
  in.succeeded = true;
  in.is_match = false;
  PendingCloud info;
  CHECK(tr.on_response("r1", in, info));
  CHECK(tr.sweep(30.0).empty());  // 未过期保留（迟到绑定窗口）
  CHECK(tr.sweep(61.0).size() == 1);
}

static void test_tracker_sweep_unresponded() {
  RequestTracker tr;
  tr.add("r1", "a", "t1", 10.0, 70.0);
  tr.add("r2", "b", "t2", 10.0, 80.0);
  const std::vector<std::string> rm = tr.sweep(75.0);
  CHECK(rm.size() == 1);
  CHECK(rm[0] == "r1");
  CHECK(tr.size() == 1);
}

static void test_cloud_library() {
  CloudLibrary::Config cfg;
  CloudLibrary lib(cfg);
  const CloudLibrary::UpsertResult r1 = lib.upsert(unit(0), "dog-a", 0.0);
  CHECK(!r1.is_match);
  CHECK(r1.global_person_id == "global-person-00001");

  const CloudLibrary::UpsertResult r2 = lib.upsert(unit(1), "dog-b", 1.0);
  CHECK(!r2.is_match);
  CHECK(r2.global_person_id == "global-person-00002");

  // 近似第 1 人（cos≈0.995）-> 并入，不新建
  const CloudLibrary::UpsertResult r3 =
      lib.upsert(blend(1.0, 0, 0.1, 1), "dog-b", 2.0);
  CHECK(r3.is_match);
  CHECK(r3.global_person_id == "global-person-00001");

  CloudRecord rec;
  CHECK(lib.get(r1.global_person_id, rec));
  CHECK(rec.merge_count == 2);
  CHECK(rec.origin_device == "dog-a");
  CHECK(rec.match_type == 0);

  // 检索：top-k、按相似度降序、min_sim 过滤
  const std::vector<CloudSearchHit> hits = lib.search(unit(0), 0.5F, 3);
  CHECK(hits.size() == 1);
  CHECK(hits[0].global_person_id == "global-person-00001");
  CHECK(hits[0].similarity > 0.99F);
  CHECK(lib.search(unit(5), 0.5F, 3).empty());

  // 名单类型落库（将来人脸结论入口）
  CHECK(lib.set_match_type(r1.global_person_id, 1));
  CHECK(lib.get(r1.global_person_id, rec) && rec.match_type == 1);
  CHECK(!lib.set_match_type("global-person-09999", 1));
}

static void test_cloud_library_daily_reset() {
  auto day = std::make_shared<std::string>("2026-09-28");
  CloudLibrary::Config cfg;
  CloudLibrary lib(cfg, [day] { return *day; });
  lib.upsert(unit(0), "a", 0.0);
  *day = "2026-09-29";
  CHECK(lib.check_daily_reset());
  CHECK(lib.size() == 0);
  CHECK(lib.check_daily_reset() == false);
  CHECK(lib.upsert(unit(0), "a", 1.0).global_person_id ==
        "global-person-00001");  // 序号重计
}

static void test_gallery_upsert_global() {
  IdentityGallery::Config cfg;
  IdentityGallery g(cfg);
  // 首次扇出：新建本地身份并绑定全局 ID
  const std::string lid =
      g.upsert_global("global-person-00007", unit(3), 1, 5, 0.0);
  CHECK(!lid.empty());
  IdentityRecord rec;
  CHECK(g.get(lid, rec));
  CHECK(rec.global_person_id == "global-person-00007");
  CHECK(rec.match_type == 1);
  CHECK(rec.merge_count == 5);
  CHECK(g.search(unit(3), "zz").hit);

  // 同一全局 ID 再次扇出：覆盖同一本地身份（模板/计数/名单类型）
  const std::string lid2 =
      g.upsert_global("global-person-00007", blend(3.0, 3, 4.0, 5), 2, 9, 10.0);
  CHECK(lid2 == lid);
  CHECK(g.get(lid, rec));
  CHECK(rec.match_type == 2);
  CHECK(rec.merge_count == 9);
  CHECK(!g.search(unit(3), "zz").hit);           // 旧模板已覆盖（cos 0.6）
  CHECK(g.search(blend(3.0, 3, 4.0, 5), "zz").hit);

  // 非法输入
  CHECK(g.upsert_global("", unit(0), 0, 1, 0.0).empty());
  CHECK(g.upsert_global("g-2", std::vector<float>(), 0, 1, 0.0).empty());
}

// 跨日运行/跨日重启均走真实 CloudLibrary -> IdentityGallery 更新链。
static void test_cross_day_cloud_identity(bool restart) {
  std::printf("[cross_day_cloud_identity] restart=%d\n", restart ? 1 : 0);
  auto day = std::make_shared<std::string>("2026-10-20");
  IdentityGallery::Config cfg;
  auto gallery = std::make_unique<IdentityGallery>(cfg, [day] { return *day; });
  CloudLibrary cloud(CloudLibrary::Config{}, [day] { return *day; });
  const auto old_cloud = cloud.upsert(unit(0), "dog-a", 100.0);
  const std::string id = gallery->create(unit(0), "track-a", 100.0);
  CHECK(gallery->bind_global(id, old_cloud.global_person_id, 1));
  CHECK(gallery->set_handled_by_track("track-a"));
  IdentityRecord original;
  CHECK(gallery->get(id, original));
  const auto old_generation = gallery->day_generation();
  const IdentityGallery::PersistMeta meta{"test-model", "1"};
  const std::string path = "test_cross_day_cloud_identity.bin";
  if (restart) {
    CHECK(gallery->SaveToFile(path, meta));
  }

  *day = "2026-10-21";
  CHECK(cloud.check_daily_reset());
  if (restart) {
    gallery = std::make_unique<IdentityGallery>(cfg, [day] { return *day; });
    const auto loaded = gallery->LoadFromFile(path, meta);
    CHECK(loaded.status == IdentityGallery::LoadStatus::kOk);
    CHECK(loaded.adopted_cross_day);
    std::remove(path.c_str());
  } else {
    CHECK(gallery->check_daily_reset());
  }
  IdentityRecord kept;
  CHECK(gallery->get(id, kept) && kept.global_person_id.empty());

  // 昨日编号今日分配给不同人物 B：不能覆盖 A、不能继承 A 的资格。
  const auto other_cloud = cloud.upsert(unit(7), "dog-b", 200.0);
  CHECK(other_cloud.global_person_id == old_cloud.global_person_id);
  const auto other_id = gallery->upsert_global(
      other_cloud.global_person_id, unit(7), 0, 1, 200.0);
  CHECK(other_id != id);
  IdentityRecord other;
  CHECK(gallery->get(other_id, other) && !other.handled);
  CHECK(other.handled_epoch_sec == 0.0);
  CHECK(gallery->get(id, kept) && kept.tmpl == original.tmpl);

  const auto new_view = blend(0.9, 0, 0.435889894, 1);
  const auto before = gallery->search(new_view, "new-track-a");
  CHECK(before.hit && before.local_reid_id == id);
  CHECK_NEAR(before.similarity, 0.9, 1e-6);
  CHECK(gallery->get(before.local_reid_id, kept) && kept.handled);
  CHECK(IdentityGallery::HandledRemainingSec(
      kept, IdentityGallery::WallEpochSec(), cfg.handled_ttl_sec) > 0.0);

  // 同人 A 的新视角取得今日新编号：更新前后应命中同一份已核验身份。
  const auto today_cloud = cloud.upsert(new_view, "dog-b", 201.0);
  CHECK(today_cloud.global_person_id != old_cloud.global_person_id);
  CloudRecord authoritative;
  CHECK(cloud.get(today_cloud.global_person_id, authoritative));
  const auto updated_id = gallery->upsert_global(
      today_cloud.global_person_id, authoritative.tmpl,
      authoritative.match_type, authoritative.merge_count, 201.0);
  CHECK(updated_id == id);
  CHECK(gallery->size() == 2);  // A 与 B，不能多出遮蔽 A 资格的副本
  const auto after = gallery->search(new_view, "another-track-a");
  CHECK(after.hit && after.local_reid_id == id);
  CHECK_NEAR(after.similarity, 1.0, 1e-6);
  CHECK(gallery->get(after.local_reid_id, kept) && kept.handled);
  CHECK(kept.global_person_id == today_cloud.global_person_id);
  CHECK(kept.handled_epoch_sec == original.handled_epoch_sec);
  CHECK(IdentityGallery::HandledRemainingSec(
      kept, IdentityGallery::WallEpochSec(), cfg.handled_ttl_sec) > 0.0);
  CHECK(IdentityGallery::HandledRemainingSec(
      kept, original.handled_epoch_sec + cfg.handled_ttl_sec,
      cfg.handled_ttl_sec) == 0.0);  // 原截止时刻不顺延
  if (!restart) {
    CHECK(!gallery->bind_global(id, old_cloud.global_person_id, 1, old_generation));
    CHECK(gallery->get(id, kept) &&
          kept.global_person_id == today_cloud.global_person_id);
  }
}

static void test_cloud_update_does_not_renew_expired_handled() {
  IdentityGallery::Config cfg;
  cfg.handled_ttl_sec = 1.0;
  auto day = std::make_shared<std::string>("2026-10-20");
  IdentityGallery g(cfg, [day] { return *day; });
  const auto id = g.create(unit(0), "a", 100.0);
  CHECK(g.set_handled_by_track("a"));
  IdentityRecord original;
  CHECK(g.get(id, original));
  *day = "2026-10-21";
  CHECK(g.check_daily_reset());
  const auto view = blend(0.9, 0, 0.435889894, 1);
  CHECK(g.upsert_global("today-a", view, 0, 1, 200.0) == id);
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  // 原资格真正到期后，再收到同编号模板更新也不能重新获得资格。
  CHECK(g.upsert_global("today-a", view, 0, 2, 201.0) == id);
  const auto hit = g.search(view, "new-track");
  CHECK(hit.hit && hit.local_reid_id == id);
  IdentityRecord expired;
  CHECK(g.get(hit.local_reid_id, expired));
  CHECK(expired.handled_epoch_sec == original.handled_epoch_sec);
  CHECK(IdentityGallery::HandledRemainingSec(
      expired, IdentityGallery::WallEpochSec(), cfg.handled_ttl_sec) == 0.0);
}

static void test_cloud_appearance_association_boundary() {
  IdentityGallery g(IdentityGallery::Config{});
  const auto handled_id = g.create(unit(0), "a", 100.0);
  CHECK(g.set_handled_by_track("a"));
  // 无 track 证据的云端更新不能采用较低的 track 兜底阈值（0.50）。
  const auto below_threshold = blend(0.6, 0, 0.8, 1);
  CHECK(g.upsert_global("other-person", below_threshold, 0, 1, 101.0) != handled_id);
  // 不能只搜索已核验/未绑定记录而忽略更相似、已绑定的另一身份。
  const auto closer = blend(0.9, 0, 0.435889894, 2);
  const auto closer_id = g.create(closer, "b", 102.0);
  CHECK(g.bind_global(closer_id, "already-bound", 0));
  const auto new_id = g.upsert_global("new-cloud-id", closer, 0, 1, 103.0);
  CHECK(new_id != handled_id && new_id != closer_id);
  IdentityRecord result;
  CHECK(g.get(new_id, result) && !result.handled && result.handled_epoch_sec == 0.0);
  CHECK(g.get(handled_id, result) && result.global_person_id.empty());
}

static void test_gallery_persist() {
  const std::string path = "test_gallery_persist.bin";
  std::remove(path.c_str());
  std::remove((path + ".tmp").c_str());
  const IdentityGallery::PersistMeta meta{"clipreid_vitb16_after", "1.0.0"};
  auto day = std::make_shared<std::string>("2026-10-07");
  IdentityGallery::Config cfg;

  // 无文件（首次启动）
  IdentityGallery g1(cfg, [day] { return *day; });
  CHECK(g1.LoadFromFile(path, meta).status ==
        IdentityGallery::LoadStatus::kMissing);
  CHECK(!g1.dirty());

  // 造状态：2 个新建身份（1 个合并过、绑定全局）+ 1 个云端扇出身份
  const std::string id1 = g1.create(unit(0), "t1", 100.0);
  const std::string id2 = g1.create(unit(1), "t2", 101.0);
  g1.merge(id1, blend(1.0, 0, 0.1, 1), "t1", 102.0);
  g1.bind_global(id1, "global-person-00042", 1);
  const std::string id3 =
      g1.upsert_global("global-person-00043", unit(4), 2, 7, 103.0);
  CHECK(g1.set_handled_by_track("t1"));  // id1 已处置（handled 随库持久化）
  CHECK(g1.dirty());
  CHECK(g1.SaveToFile(path, meta));
  CHECK(!g1.dirty());  // 落盘成功后脏标记清零

  // “重启”：同日新库加载，二进制回环应逐位还原
  IdentityGallery g2(cfg, [day] { return *day; });
  const auto lr = g2.LoadFromFile(path, meta);
  CHECK(lr.status == IdentityGallery::LoadStatus::kOk);
  CHECK(lr.records == 3);
  CHECK(!g2.dirty());
  IdentityRecord before, after;
  CHECK(g1.get(id1, before) && g2.get(id1, after));
  CHECK(before.tmpl == after.tmpl);
  CHECK(after.merge_count == before.merge_count);
  CHECK(after.global_person_id == "global-person-00042");
  CHECK(after.match_type == 1);
  // 时间戳不逐位还原：加载时统一重整到本次开机的单调时钟基线（R17），
  // 同一条记录内 first==last；此后命中 merge 会用当前时钟刷新
  CHECK_NEAR(after.first_seen, after.last_seen, 1e-9);
  CHECK(after.last_seen > 0.0);
  CHECK(g2.get(id2, after) && after.merge_count == 1);
  CHECK(g2.get(id3, after) && after.merge_count == 7 &&
        after.match_type == 2);
  CHECK(g2.get(id1, after) && after.handled);    // handled 随持久化还原
  CHECK(g2.get(id2, after) && !after.handled);

  // 恢复后：特征命中照常；track 映射不落盘（重启后语义失效，只能走特征阈值）
  CHECK(g2.search(blend(1.0, 0, 0.1, 1), "zz").hit);
  CHECK(g2.search(blend(1.0, 0, 0.1, 1), "zz").local_reid_id == id1);
  const std::vector<float> f060 = blend(3.0, 0, 4.0, 2);
  CHECK(!g2.search(f060, "t1").hit);  // cos≈0.6 低于阈值，track 兜底已不存在

  // 序号续号：不重用已持久化身份的 local_reid_id
  CHECK(g2.create(unit(7), "t9", 200.0) == "local-person-00004");

  // 跨天文件：不再整份作废——仍在忽略期内的免核验记录（id1，带标记）
  // 跨天采纳，其余当日事实（id2/id3，仅见过/云端扇出）按每日清零丢弃；
  // 采纳记录剥离云端当日关联（id1 原绑定 global-person-00042，云端编号
  // 次日复用，保留会被 upsert_global 错绑到新人）
  *day = "2026-10-08";
  IdentityGallery g3(cfg, [day] { return *day; });
  {
    const auto lr3 = g3.LoadFromFile(path, meta);
    CHECK(lr3.status == IdentityGallery::LoadStatus::kOk);
    CHECK(lr3.adopted_cross_day);
    CHECK(g3.size() == 1);
    IdentityRecord cross;
    CHECK(g3.get(id1, cross) && cross.handled && cross.handled_epoch_sec > 0.0);
    CHECK(cross.global_person_id.empty() && cross.match_type == 255);
    CHECK(!g3.get(id2, cross) && !g3.get(id3, cross));
    CHECK(g3.search(unit(0), "zz").local_reid_id == id1);  // 外观命中延续
    // 复用编号扇出：新建独立身份，不覆盖采纳记录、不继承免核验
    CHECK(g3.upsert_global("global-person-00042", unit(2), 1, 1, 200.0) != id1);
    CHECK(g3.get(id1, cross) && cross.handled && cross.global_person_id.empty());
    // 序号从采纳记录的最大后缀续起，不与沿用 ID 撞号（upsert 已用掉 00002）
    CHECK(g3.create(unit(7), "t-new", 200.0) == "local-person-00003");
  }
  *day = "2026-10-07";

  // 模型/版本不符拒载
  const IdentityGallery::PersistMeta bad_meta{"osnet_x1", "1.0.0"};
  IdentityGallery g4(cfg, [day] { return *day; });
  CHECK(g4.LoadFromFile(path, bad_meta).status ==
        IdentityGallery::LoadStatus::kModelMismatch);
  CHECK(g4.size() == 0);

  // 损坏文件拒载：空文件 / 随机字节 / 截断半截
  {
    std::ofstream(path, std::ios::binary | std::ios::trunc).close();
    IdentityGallery g5(cfg, [day] { return *day; });
    CHECK(g5.LoadFromFile(path, meta).status ==
          IdentityGallery::LoadStatus::kCorrupt);

    std::ofstream(path, std::ios::binary | std::ios::trunc) << "garbage!!!";
    IdentityGallery g6(cfg, [day] { return *day; });
    CHECK(g6.LoadFromFile(path, meta).status ==
          IdentityGallery::LoadStatus::kCorrupt);

    // 有效文件被截断一半（模拟写盘中途断电，原子替换失效的极端情形）
    CHECK(g1.SaveToFile(path, meta));
    std::ifstream rd(path, std::ios::binary);
    const std::string full((std::istreambuf_iterator<char>(rd)),
                           std::istreambuf_iterator<char>());
    rd.close();
    std::ofstream(path, std::ios::binary | std::ios::trunc)
        << full.substr(0, full.size() / 2);
    IdentityGallery g7(cfg, [day] { return *day; });
    CHECK(g7.LoadFromFile(path, meta).status ==
          IdentityGallery::LoadStatus::kCorrupt);
    CHECK(g7.size() == 0);

    // 版本不符（等价于旧 v1 文件）按损坏拒绝，不做迁移
    CHECK(g1.SaveToFile(path, meta));
    {
      std::fstream f(path, std::ios::in | std::ios::out | std::ios::binary);
      f.seekp(8);  // magic(8B) 之后即 version(u32)
      const char v1[4] = {1, 0, 0, 0};
      f.write(v1, 4);
    }
    IdentityGallery g8(cfg, [day] { return *day; });
    CHECK(g8.LoadFromFile(path, meta).status ==
          IdentityGallery::LoadStatus::kCorrupt);
    CHECK(g8.size() == 0);
  }

  // 跨天文件中已到期的免核验标记：不采纳（资格按墙钟跨天判定，过期即弃）
  {
    IdentityGallery::Config ttl_cfg;
    ttl_cfg.handled_ttl_sec = 0.3;
    const std::string p_exp = "test_gallery_persist_cross_expired.bin";
    std::remove(p_exp.c_str());
    std::remove((p_exp + ".tmp").c_str());
    IdentityGallery ge(ttl_cfg, [day] { return *day; });
    const std::string ide = ge.create(unit(6), "te", 100.0);
    CHECK(ge.set_handled_by_track("te"));
    CHECK(ge.SaveToFile(p_exp, meta));
    std::this_thread::sleep_for(std::chrono::milliseconds(400));  // 停机越过忽略时长
    *day = "2026-10-09";
    IdentityGallery gx(ttl_cfg, [day] { return *day; });
    const auto lrx = gx.LoadFromFile(p_exp, meta);
    CHECK(lrx.status == IdentityGallery::LoadStatus::kOk && lrx.adopted_cross_day);
    CHECK(gx.size() == 0);  // 唯一记录已到期：整份丢弃
    *day = "2026-10-07";
    std::remove(p_exp.c_str());
    std::remove((p_exp + ".tmp").c_str());
  }

  // R17：整机重启（单调时钟归零）后，恢复的旧时间戳不得让新人被错误淘汰
  {
    const std::string p2 = "test_gallery_persist_r17.bin";
    std::remove(p2.c_str());
    std::remove((p2 + ".tmp").c_str());
    IdentityGallery::Config cap1;
    cap1.max_identities = 1;
    IdentityGallery go(cap1, [day] { return *day; });
    const std::string ida = go.create(unit(2), "t-old", 10000.0);  // 旧开机期的时间戳
    CHECK(go.SaveToFile(p2, meta));
    IdentityGallery gr(cap1, [day] { return *day; });
    const auto lr17 = gr.LoadLatest(p2, meta);
    CHECK(lr17.status == IdentityGallery::LoadStatus::kOk);
    CHECK(!lr17.from_tmp);
    const std::string idb = gr.create(unit(6), "t-new", 1e12);  // 新记录时间戳更大
    CHECK(idb != ida);
    CHECK(gr.search(unit(6), "t-new").hit);  // 新人存活
    CHECK(!gr.search(unit(2), "t-old").hit); // 容量淘汰的是恢复的旧身份
    std::remove(p2.c_str());
    std::remove((p2 + ".tmp").c_str());
  }

  // R16：正式文件缺失但残留有效 .tmp（替换被中断）→ LoadLatest 恢复并提升
  {
    const std::string p3 = "test_gallery_persist_r16.bin";
    const std::string tmp3 = p3 + ".tmp";
    std::remove(p3.c_str());
    std::remove(tmp3.c_str());
    IdentityGallery gs(cfg, [day] { return *day; });
    gs.create(unit(5), "t1", 1.0);
    CHECK(gs.SaveToFile(p3, meta));
    CHECK(gs.SaveToFile(tmp3, meta));  // 把 .tmp 当目标再存一份，模拟替换前的中间态
    std::remove(p3.c_str());           // 模拟替换中断后正式文件缺失
    IdentityGallery gl(cfg, [day] { return *day; });
    const auto lr16 = gl.LoadLatest(p3, meta);
    CHECK(lr16.status == IdentityGallery::LoadStatus::kOk);
    CHECK(lr16.from_tmp);
    CHECK(gl.search(unit(5), "zz").hit);  // 从 .tmp 恢复的内容可用
    std::remove(p3.c_str());
    std::remove(tmp3.c_str());
  }
  std::remove(path.c_str());
  std::remove((path + ".tmp").c_str());
}

static void test_gallery_handled() {
  IdentityGallery::Config cfg;
  IdentityGallery g(cfg);
  const std::string id1 = g.create(unit(0), "t1", 0.0);
  const std::string id2 = g.create(unit(1), "t2", 0.0);

  // 未标记前：命中 handled=false（仅见过 ≠ 已处置）
  IdentityRecord rec;
  CHECK(g.get(id1, rec) && !rec.handled);

  // 按 track 绑定标记（任务侧 handled 下发的主路径）
  CHECK(g.set_handled_by_track("t1"));
  CHECK(g.get(id1, rec) && rec.handled);
  CHECK(g.get(id2, rec) && !rec.handled);   // 不误伤他人
  CHECK(!g.set_handled_by_track("t-none")); // 无绑定

  // 外观兜底：绑定过期后按特征定位（须过身份阈值 0.70）
  g.prune_tracks(1e9, 0.0);                 // 清空全部 track 映射
  CHECK(g.set_handled_by_feature(unit(1))); // 与 id2 cos=1.0
  CHECK(g.get(id2, rec) && rec.handled);
  CHECK(!g.set_handled_by_feature(unit(7))); // 与两人均无关，宁可不标

  // 任务 reset：清除全部 handled 标记（身份与模板保留）
  CHECK(g.clear_handled() == 2);
  CHECK(g.get(id1, rec) && !rec.handled);
  CHECK(g.get(id2, rec) && !rec.handled);
  CHECK(g.size() == 2);                      // 身份本体未被清除
  CHECK(g.clear_handled() == 0);
}

// "重复核验忽略时长"（平台配置项）：免核验资格按置位时刻+TTL 在读取侧判定，
// 到期后 handled 不再免核验（外观事实/身份记录保留）；持久化 v3 恢复时
// 置位墙钟随库持久化，跨重启按剩余量精确倒计时（停机期间照常计入）；
// v2 旧文件的 handled 标记按旧处置语义作废。
static void test_gallery_handled_ttl() {
  const auto near = [](double a, double b) {
    const double d = a - b;
    return d < 1e-6 && d > -1e-6;
  };

  IdentityGallery::Config cfg;
  cfg.handled_ttl_sec = 60.0;
  IdentityGallery g(cfg);
  const std::string id1 = g.create(unit(0), "t1", 0.0);
  const std::string id2 = g.create(unit(1), "t2", 0.0);

  IdentityRecord rec;
  CHECK(g.set_handled_by_track("t1"));
  CHECK(g.get(id1, rec) && rec.handled && rec.handled_epoch_sec > 0.0);
  CHECK(near(IdentityGallery::HandledRemainingSec(rec, rec.handled_epoch_sec, 60.0), 60.0));
  CHECK(near(IdentityGallery::HandledRemainingSec(rec, rec.handled_epoch_sec + 30.0, 60.0), 30.0));
  // 到期：剩余 0（免核验资格失效，is_seen 外观事实不受影响）
  CHECK(IdentityGallery::HandledRemainingSec(rec, rec.handled_epoch_sec + 60.0, 60.0) == 0.0);
  CHECK(IdentityGallery::HandledRemainingSec(rec, rec.handled_epoch_sec + 61.0, 60.0) == 0.0);
  // TTL=0（平台配置"关闭免核验"）：已标记也不免核验
  CHECK(IdentityGallery::HandledRemainingSec(rec, rec.handled_epoch_sec, 0.0) == 0.0);
  // 未标记记录恒 0
  IdentityRecord rec2;
  CHECK(g.get(id2, rec2) && rec2.handled_epoch_sec == 0.0);
  CHECK(IdentityGallery::HandledRemainingSec(rec2, rec.handled_epoch_sec, 60.0) == 0.0);

  // 持久化 v3：标记连同置位墙钟恢复，剩余量按原置位时刻延续（≈完整 TTL）
  const std::string path = "test_gallery_ttl.bin";
  std::remove(path.c_str());
  std::remove((path + ".tmp").c_str());
  const IdentityGallery::PersistMeta meta{"m", "1"};
  CHECK(g.SaveToFile(path, meta));
  {
    IdentityGallery g2(cfg);
    CHECK(g2.LoadFromFile(path, meta).status == IdentityGallery::LoadStatus::kOk);
    IdentityRecord after;
    CHECK(g2.get(id1, after) && after.handled && after.handled_epoch_sec > 0.0);
    // 墙钟连续：剩余量跨重启按原置位时刻延续（不再"重新起算"）
    const double rem = IdentityGallery::HandledRemainingSec(
        after, IdentityGallery::WallEpochSec(), 60.0);
    CHECK(rem > 59.0 && rem <= 60.000001);
  }
  // v2 旧文件（版本=2，记录无尾部 handled_epoch_sec 字段）：手工按 v2 布局
  // 精确构造一条 handled=1 的记录——身份与外观记录保留，标记按旧处置
  // 语义（跟丢即处置）作废
  {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    const auto w32 = [&f](const uint32_t v) {
      f.write(reinterpret_cast<const char*>(&v), 4);
    };
    const auto wstr = [&f, &w32](const std::string& s) {
      w32(static_cast<uint32_t>(s.size()));
      f.write(s.data(), static_cast<std::streamsize>(s.size()));
    };
    f.write("REIDLB01", 8);
    w32(2);  // version = v2
    wstr(IdentityGallery::today_string());
    wstr(meta.model_name);
    wstr(meta.model_version);
    w32(1);  // seq
    w32(1);  // count = 1 条记录
    wstr("local-person-00042");
    w32(1);  // merge_count
    const uint8_t match_type = 255;
    const uint8_t handled = 1;
    f.write(reinterpret_cast<const char*>(&match_type), 1);
    f.write(reinterpret_cast<const char*>(&handled), 1);
    wstr("");  // global_person_id
    w32(1);    // dim
    const float tmpl = 1.0F;
    f.write(reinterpret_cast<const char*>(&tmpl), 4);
    const double seen = 100.0;
    f.write(reinterpret_cast<const char*>(&seen), 8);  // first_seen
    f.write(reinterpret_cast<const char*>(&seen), 8);  // last_seen
    //（v2 记录到此为止：无 handled_epoch_sec）
  }
  IdentityGallery g3(cfg);
  CHECK(g3.LoadFromFile(path, meta).status == IdentityGallery::LoadStatus::kOk);
  CHECK(g3.size() == 1);
  IdentityRecord legacy;
  CHECK(g3.get("local-person-00042", legacy));
  CHECK(!legacy.handled && legacy.handled_epoch_sec == 0.0);  // 标记作废
  CHECK(legacy.tmpl.size() == 1);                            // 外观记录保留
  std::remove(path.c_str());
  std::remove((path + ".tmp").c_str());

  // 跨重启到期：保存时仍有效、恢复时已越过忽略时长的标记不再免核验
  //（停机期间照常计入——不依赖"到期后再保存一次"，防抖/退出保存都不会
  // 因时间流逝触发）
  {
    IdentityGallery::Config cross_cfg;
    cross_cfg.handled_ttl_sec = 0.3;
    IdentityGallery gc(cross_cfg);
    const std::string idc = gc.create(unit(4), "tc", 0.0);
    CHECK(gc.set_handled_by_track("tc"));
    const std::string pc = "test_gallery_ttl_cross.bin";
    std::remove(pc.c_str());
    CHECK(gc.SaveToFile(pc, meta));  // 保存时标记有效
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    IdentityGallery gr(cross_cfg);
    CHECK(gr.LoadFromFile(pc, meta).status == IdentityGallery::LoadStatus::kOk);
    IdentityRecord cross;
    CHECK(gr.get(idc, cross) && !cross.handled && cross.handled_epoch_sec == 0.0);
    std::remove(pc.c_str());
    std::remove((pc + ".tmp").c_str());
  }

  // 已到期的标记按未标记落盘（重启不会以完整忽略时长复活过期资格）：
  // id1 上面的标记在 ttl=60 下仍有效——另用短 ttl 库验证过期不落盘
  {
    IdentityGallery::Config short_cfg;
    short_cfg.handled_ttl_sec = 0.2;
    IdentityGallery gs(short_cfg);
    const std::string ids = gs.create(unit(3), "ts", 0.0);
    CHECK(gs.set_handled_by_track("ts"));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));  // 越过 0.2s
    const std::string p2 = "test_gallery_ttl_expired.bin";
    std::remove(p2.c_str());
    CHECK(gs.SaveToFile(p2, meta));
    IdentityGallery gr(short_cfg);
    CHECK(gr.LoadFromFile(p2, meta).status == IdentityGallery::LoadStatus::kOk);
    IdentityRecord expired;
    CHECK(gr.get(ids, expired) && !expired.handled && expired.handled_epoch_sec == 0.0);
    std::remove(p2.c_str());
    std::remove((p2 + ".tmp").c_str());
  }

  // 跨午夜保留（需求方确认：免核验资格从核验通过时刻起完整持续配置
  // 时长，不因自然日切换提前到期）：带有效标记的记录连同模板保留，
  // 无标记身份照旧清零，序号从保留的最大后缀续起不撞号；忽略期内
  // 再遇该人 -> 仍 seen+handled（免核验延续）
  {
    auto rday = std::make_shared<std::string>("2026-10-20");
    IdentityGallery gm(cfg, [rday] { return *rday; });
    const std::string idm = gm.create(unit(5), "tm", 0.0);
    CHECK(gm.set_handled_by_track("tm"));
    CHECK(gm.bind_global(idm, "global-person-00001", 1));  // 昨日的云端绑定
    const std::string idn = gm.create(unit(6), "tn", 0.0);  // 仅见过
    CHECK(gm.size() == 2);

    *rday = "2026-10-21";
    CHECK(gm.check_daily_reset());
    CHECK(gm.size() == 1);                       // 仅保留带有效标记的记录
    IdentityRecord kept;
    CHECK(gm.get(idm, kept) && kept.handled);    // 标记与模板保留
    CHECK(kept.global_person_id.empty() && kept.match_type == 255);  // 云端当日关联剥离
    CHECK(kept.last_seen > 0.0 && kept.last_seen < 1e9);  // 单调时钟口径（非墙钟 1.8e9 量级）
    CHECK(!gm.get(idn, kept));                   // 无标记身份清零
    const auto so = gm.search(unit(5), "any");
    CHECK(so.hit && so.local_reid_id == idm);    // 外观仍命中同人
    CHECK(gm.create(unit(7), "t-new", 0.0) == "local-person-00002");  // 序号续起不撞号

    // 云端编号次日复用：同一 global-person-00001 分配给新人并扇出模板——
    // 因绑定已剥离，upsert_global 新建独立身份，不覆盖旧身份、不继承免核验
    const std::string id_new =
        gm.upsert_global("global-person-00001", unit(7), 1, 1, 0.0);
    CHECK(id_new != idm);
    IdentityRecord rec_new;
    CHECK(gm.get(id_new, rec_new) && !rec_new.handled);      // 新人无免核验资格
    CHECK(rec_new.global_person_id == "global-person-00001");
    CHECK(gm.get(idm, kept) && kept.handled && kept.global_person_id.empty());
  }

  // 跨日在途云端响应：迟到回绑被日期代次拒绝（防清理后的编号复用链复发）。
  // 顺序复刻评审场景：昨日发起请求（记录代次）→ 午夜保留式清理（代次+1、
  // 剥离绑定）→ 昨日响应迟到回绑（旧代次，拒绝）→ 编号复用安全
  {
    auto cday = std::make_shared<std::string>("2026-11-01");
    IdentityGallery gc2(cfg, [cday] { return *cday; });
    const std::string ida = gc2.create(unit(2), "ta", 0.0);
    CHECK(gc2.set_handled_by_track("ta"));
    const uint64_t gen_before = gc2.day_generation();

    *cday = "2026-11-02";
    CHECK(gc2.check_daily_reset());
    CHECK(gc2.day_generation() == gen_before + 1);           // 清理递增代次
    IdentityRecord after_flip;
    CHECK(gc2.get(ida, after_flip) && after_flip.handled &&
          after_flip.global_person_id.empty());

    // 昨日响应迟到：旧代次回绑被拒，绑定保持剥离状态
    CHECK(!gc2.bind_global(ida, "global-person-00009", 1, gen_before));
    CHECK(gc2.get(ida, after_flip) && after_flip.global_person_id.empty());
    // 今日新请求（当前代次）绑定正常；0=不校验（当日同步路径/测试直连）
    CHECK(gc2.bind_global(ida, "global-person-00009", 1, gc2.day_generation()));
    CHECK(gc2.get(ida, after_flip) && after_flip.global_person_id == "global-person-00009");
    CHECK(gc2.bind_global(ida, "global-person-00010", 1, 0));
    CHECK(gc2.get(ida, after_flip) && after_flip.global_person_id == "global-person-00010");
    // 绑定失败不影响记录其余字段（handled 资格仍在）
    CHECK(after_flip.handled);
  }
}

int main() {
  test_gallery_basic();
  test_track_assoc();
  test_merge_running_mean();
  test_bind_global();
  test_daily_reset();
  test_track_ttl();
  test_evict();
  test_threshold_respected();
  test_tracker_basic();
  test_tracker_timeout_and_late();
  test_tracker_fire_and_forget();
  test_tracker_sweep_unresponded();
  test_cloud_library();
  test_cloud_library_daily_reset();
  test_gallery_upsert_global();
  test_cross_day_cloud_identity(false);
  test_cross_day_cloud_identity(true);
  test_cloud_update_does_not_renew_expired_handled();
  test_cloud_appearance_association_boundary();
  test_gallery_persist();
  test_gallery_handled();
  test_gallery_handled_ttl();

  std::printf("%d/%d passed\n", g_total - g_fail, g_total);
  return g_fail == 0 ? 0 : 1;
}

static std::string id_of(const IdentityGallery& g, const std::string& track) {
  return g.search(blend(3.0, 0, 4.0, 2), track).local_reid_id;
}
