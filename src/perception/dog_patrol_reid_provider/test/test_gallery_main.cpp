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

  // 跨天文件作废（每日清零语义保持）：次日开机，库构造时即取到新日期
  *day = "2026-10-08";
  IdentityGallery g3(cfg, [day] { return *day; });
  CHECK(g3.LoadFromFile(path, meta).status ==
        IdentityGallery::LoadStatus::kWrongDay);
  CHECK(g3.size() == 0);
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
  test_gallery_persist();
  test_gallery_handled();

  std::printf("%d/%d passed\n", g_total - g_fail, g_total);
  return g_fail == 0 ? 0 : 1;
}

static std::string id_of(const IdentityGallery& g, const std::string& track) {
  return g.search(blend(3.0, 0, 4.0, 2), track).local_reid_id;
}
