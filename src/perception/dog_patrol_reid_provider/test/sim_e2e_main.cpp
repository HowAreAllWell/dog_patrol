// -*- coding: utf-8 -*-
// 端到端逻辑仿真：用产品库代码（IdentityGallery / CloudLibrary / RequestTracker）
// 镜像 reid_node/reid_cloud_proxy 的编排顺序，不依赖 ROS/OpenCV，Windows g++
// 即可运行。特征用低维合成向量模拟（逻辑与维度无关）。
//
// 编译运行：
//   g++ -std=c++17 -Iinclude test/sim_e2e_main.cpp src/reid_gallery.cpp src/cloud_library.cpp -o sim_e2e && ./sim_e2e
//
// 覆盖剧本：
//   S1 狗A首遇P1：端侧未命中->建身份->云端search_and_upsert->新建全局ID
//   S2 狗A再遇P1(轻微变化)：端侧命中立即应答，模板增量<防抖阈值不回推
//   S3 狗A遇P1(衣着变化)：命中+模板移动超阈值->回推->云端合并->扇出狗B
//   S4 狗B遇P1：B 的端侧库已有扇出条目->直接命中（跨狗去重）
//   S5 狗B首遇P2：云端新建->扇出狗A
//   S6 狗A重启(换新库)：端侧丢失->云端命中找回（is_seen 仍正确）
//   S6b 狗A关机充电(当日库落盘)->重启恢复：端侧直接命中（不依赖云端）
//   S7 人脸结论落云端库(set_match_type)：后续命中透传 match_type=白
//   S8 云端超时降级+迟到绑定（RequestTracker）
//   S9 跨天：当日事实清零（无有效免核验标记时全清）、序号重计
//   S10 handled语义(R06-A)：处置完成换新track再遇 -> seen+handled 透传
//   S11 在途请求跨日：代次随身份解析固定，登记/响应晚于跨日清理 ->
//       昨日编号迟到回绑被拒（评审竞态回归）
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "dog_patrol_reid_provider/cloud_library.hpp"
#include "dog_patrol_reid_provider/reid_gallery.hpp"
#include "dog_patrol_reid_provider/request_tracker.hpp"

using dog_patrol_reid_provider::CloudLibrary;
using dog_patrol_reid_provider::CloudOutcome;
using dog_patrol_reid_provider::CloudRecord;
using dog_patrol_reid_provider::IdentityGallery;
using dog_patrol_reid_provider::IdentityRecord;
using dog_patrol_reid_provider::PendingCloud;
using dog_patrol_reid_provider::RequestTracker;

static int g_fail = 0, g_total = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    ++g_total;                                                    \
    if (!(cond)) {                                                \
      ++g_fail;                                                   \
      std::printf("    FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

namespace {
constexpr size_t kD = 16;

std::vector<float> unit(size_t i) {
  std::vector<float> v(kD, 0.0F);
  v[i] = 1.0F;
  return v;
}
// normalize(u_i + noise*u_j)：与 u_i 的余弦 = 1/sqrt(1+noise^2)
std::vector<float> noisy(size_t i, double noise, size_t j) {
  std::vector<float> v(kD, 0.0F);
  v[i] = 1.0F;
  v[j] = static_cast<float>(noise);
  dog_patrol_reid_provider::l2_normalize(v);
  return v;
}

const char* mt_name(uint8_t mt) {
  switch (mt) {
    case 0: return "陌生人";
    case 1: return "白名单";
    case 2: return "不确定";
    default: return "未知";
  }
}

// ---- 仿真世界：狗集合 + 共享云端库（day_provider 统一注入便于跨天测试）----
struct World {
  float threshold = 0.70F;
  float update_min_delta = 0.01F;
  double now = 0.0;
  CloudLibrary cloud;
  std::map<std::string, IdentityGallery*> dogs;  // device_id -> 端侧库

  explicit World(std::function<std::string()> day_provider)
      : cloud(CloudLibrary::Config{}, std::move(day_provider)) {}
};

struct EncResult {
  bool is_seen = false;
  bool via_cloud = false;      // 端侧未命中、云端命中（对应日志 CLOUD_HIT）
  bool pushed_cloud = false;   // 本次是否回推了云端（并触发扇出）
  bool handled = false;        // 命中身份是否已处置（R06-A：业务结论经外观延续）
  std::string local_id, global_id;
  uint8_t match_type = 255;
};

// 镜像 reid_cloud_proxy.fan_out：库变化后把权威模板推给其他狗（跳过来源）
void fan_out(World& w, const std::string& origin, const std::string& gid) {
  CloudRecord crec;
  if (!w.cloud.get(gid, crec)) {
    return;
  }
  for (auto& kv : w.dogs) {
    if (kv.first == origin || kv.second == nullptr) {
      continue;
    }
    kv.second->upsert_global(gid, crec.tmpl, crec.match_type, crec.merge_count,
                             w.now);
  }
}

// 镜像 reid_node.on_dedup 的决策序列（端侧检索->命中合并/未命中建身份->
// 云端 upsert->绑定全局ID->扇出）
EncResult encounter(World& w, IdentityGallery& g, const std::string& dog,
                    const std::vector<float>& feat, const std::string& track) {
  g.check_daily_reset();
  w.cloud.check_daily_reset();
  ++w.now;
  EncResult r;
  const auto sr = g.search(feat, track);
  if (sr.hit) {
    const float delta = g.merge(sr.local_reid_id, feat, track, w.now);
    IdentityRecord rec;
    g.get(sr.local_reid_id, rec);
    r.is_seen = true;
    r.local_id = sr.local_reid_id;
    r.global_id = rec.global_person_id;
    r.match_type = rec.match_type;
    r.handled = rec.handled;
    if (delta >= w.update_min_delta) {
      // 回推的是合并后的模板（与 publish_upsert 一致）
      const auto up = w.cloud.upsert(rec.tmpl, dog, w.now);
      r.pushed_cloud = true;
      fan_out(w, dog, up.global_person_id);
    }
    return r;
  }
  r.local_id = g.create(feat, track, w.now);
  const auto up = w.cloud.upsert(feat, dog, w.now);  // search_and_upsert
  CloudRecord crec;
  uint8_t mt = 255;
  if (w.cloud.get(up.global_person_id, crec)) {
    mt = crec.match_type;
  }
  g.bind_global(r.local_id, up.global_person_id, mt);  // 命中/新建都绑定
  r.global_id = up.global_person_id;
  r.match_type = mt;
  r.is_seen = up.is_match;
  r.via_cloud = up.is_match;
  r.pushed_cloud = true;
  fan_out(w, dog, up.global_person_id);
  return r;
}

void report(const char* step, const char* desc, const EncResult& r) {
  std::printf("[%s] %s\n    -> is_seen=%d handled=%d 路径=%s local=%s global=%s "
              "match_type=%s 回推云端=%d\n",
              step, desc, r.is_seen ? 1 : 0, r.handled ? 1 : 0,
              r.via_cloud ? "云端命中" : (r.is_seen ? "端侧命中" : "新人"),
              r.local_id.c_str(), r.global_id.c_str(), mt_name(r.match_type),
              r.pushed_cloud ? 1 : 0);
}
}  // namespace

int main() {
  auto day = std::make_shared<std::string>("2026-09-28");
  auto mk_gallery = [&day] {
    IdentityGallery::Config cfg;
    return new IdentityGallery(cfg, [day] { return *day; });
  };
  World w([day] { return *day; });

  auto* ga = mk_gallery();
  auto* gb = mk_gallery();
  w.dogs["m20-001"] = ga;
  w.dogs["m20-002"] = gb;

  const std::string g1 = "global-person-00001";
  const std::string g2 = "global-person-00002";

  // S1 狗A首遇P1
  EncResult r = encounter(w, *ga, "m20-001", unit(0), "t-a1");
  report("S1", "狗A(m20-001) 首遇 P1(track=t-a1)", r);
  CHECK(!r.is_seen);
  CHECK(r.global_id == g1);
  CHECK(w.cloud.size() == 1);

  // S2 轻微变化再遇：端侧命中、不回推
  r = encounter(w, *ga, "m20-001", noisy(0, 0.08, 1), "t-a1");
  report("S2", "狗A 再遇 P1(轻微视角变化 cos≈0.997)", r);
  CHECK(r.is_seen && !r.via_cloud);
  CHECK(!r.pushed_cloud);
  {
    CloudRecord crec;
    CHECK(w.cloud.get(g1, crec) && crec.merge_count == 1);  // 云端没动
  }

  // S3 衣着变化：回推 + 云端合并 + 扇出狗B
  r = encounter(w, *ga, "m20-001", noisy(0, 0.55, 1), "t-a1");
  report("S3", "狗A 遇 P1(衣着变化 cos≈0.88)", r);
  CHECK(r.is_seen && r.pushed_cloud);
  {
    CloudRecord crec;
    CHECK(w.cloud.get(g1, crec) && crec.merge_count == 2);
  }
  CHECK(gb->size() == 1);  // 扇出落地：狗B 端侧库有了 P1

  // S4 狗B遇P1：B 自己的端侧库直接命中（跨狗去重成立）
  r = encounter(w, *gb, "m20-002", noisy(0, 0.08, 2), "t-b1");
  report("S4", "狗B(m20-002) 遇 P1(track=t-b1)——B 只靠扇出条目就认出", r);
  CHECK(r.is_seen && !r.via_cloud);
  CHECK(r.global_id == g1);

  // S5 狗B首遇P2：云端新建并扇出狗A
  r = encounter(w, *gb, "m20-002", unit(3), "t-b2");
  report("S5", "狗B 首遇 P2(track=t-b2)", r);
  CHECK(!r.is_seen && r.global_id == g2);
  CHECK(ga->size() == 2);  // A 也拿到了 P2

  // S6 狗A重启（换新端侧库，云端还在）：P2 靠云端命中找回
  w.dogs.erase("m20-001");
  delete ga;
  auto* ga2 = mk_gallery();
  w.dogs["m20-001"] = ga2;
  r = encounter(w, *ga2, "m20-001", noisy(3, 0.10, 4), "t-a9");
  report("S6", "狗A 重启后遇 P2(track=t-a9)——端侧丢失、云端找回", r);
  CHECK(r.is_seen && r.via_cloud);
  CHECK(r.global_id == g2);

  // S6b 狗A关机充电（当日库落盘）-> 重启恢复：端侧直接命中，不依赖云端
  const std::string lib_file = "sim_reid_library.bin";
  std::remove(lib_file.c_str());
  CHECK(ga2->SaveToFile(lib_file, {"clipreid_vitb16_after", "1.0.0"}));
  w.dogs.erase("m20-001");
  delete ga2;
  ga2 = mk_gallery();
  CHECK(ga2->LoadFromFile(lib_file, {"clipreid_vitb16_after", "1.0.0"})
            .status == IdentityGallery::LoadStatus::kOk);
  std::remove(lib_file.c_str());
  w.dogs["m20-001"] = ga2;
  r = encounter(w, *ga2, "m20-001", noisy(3, 0.10, 4), "t-a9");
  report("S6b", "狗A 再次重启但当日库已从盘恢复 -> 遇 P2 端侧直接命中", r);
  CHECK(r.is_seen && !r.via_cloud);
  CHECK(r.global_id == g2);

  // S7 人脸结论落云端库：之后所有狗的命中都透传 match_type=白名单
  CHECK(w.cloud.set_match_type(g1, 1));  // 模拟人脸核验通过落库
  auto* ga3 = mk_gallery();              // 再重启一只全新库的狗
  w.dogs.erase("m20-001");
  delete ga2;
  w.dogs["m20-001"] = ga3;
  r = encounter(w, *ga3, "m20-001", unit(0), "t-a10");
  report("S7", "人脸结论已落库后，狗A(再次重启) 遇 P1", r);
  CHECK(r.is_seen && r.via_cloud);
  CHECK(r.match_type == 1);

  // S8 云端超时降级 + 迟到绑定
  RequestTracker tr;
  const std::string lid = ga3->create(unit(7), "t-a11", w.now);
  tr.add("req-late", lid, "t-a11", w.now + 3.0, w.now + 63.0);
  CloudOutcome polled;
  CHECK(!tr.poll("req-late", polled));  // 3s 内无响应
  CHECK(tr.mark_timeout("req-late"));   // 按超时应答（is_seen=false 已返回）
  CloudOutcome late;
  late.succeeded = true;
  late.is_match = true;
  late.global_person_id = "global-person-00003";
  late.match_type = 0;
  PendingCloud info;
  CHECK(tr.on_response("req-late", late, info));
  CHECK(info.timed_out_reported);
  CHECK(ga3->bind_global(info.local_reid_id, late.global_person_id,
                         late.match_type));  // 迟到绑定仍生效
  std::printf("[S8] 云端超时降级为 NEW 应答；迟到响应绑定 %s -> %s\n",
              info.local_reid_id.c_str(), late.global_person_id.c_str());
  {
    IdentityRecord rec;
    CHECK(ga3->get(lid, rec) && rec.global_person_id == "global-person-00003");
  }

  // S9 跨天：当日事实清零、序号重计（本剧本人员无有效免核验标记）
  *day = "2026-09-29";
  r = encounter(w, *ga3, "m20-001", unit(0), "t-a1");
  report("S9", "次日狗A 遇 P1——全部清零，重新当新人", r);
  CHECK(!r.is_seen);
  CHECK(r.global_id == g1);  // 云端序号也归零重计
  CHECK(w.cloud.size() == 1);

  // S10 handled 语义（R06-A）：处置完成的人换新 track 再遇 -> seen 且 handled；
  // 处置结论由任务侧下发（镜像 set_handled_by_track），ReID 只按外观延续
  w.dogs.erase("m20-001");
  auto* ga5 = mk_gallery();
  w.dogs["m20-001"] = ga5;
  r = encounter(w, *ga5, "m20-001", unit(5), "t-x1");
  report("S10a", "狗A(新库) 首遇 P3(track=t-x1)——仅见过，未处置", r);
  CHECK(!r.is_seen && !r.handled);
  CHECK(ga5->set_handled_by_track("t-x1"));  // supervisor 判处置完成并下发
  ga5->prune_tracks(w.now + 1e6, 0.0);       // P3 离开：track 映射过期
  r = encounter(w, *ga5, "m20-001", unit(5), "t-x2");
  report("S10b", "P3 处置完成后换新 track 再遇——seen 且 handled 透传", r);
  CHECK(r.is_seen && !r.via_cloud && r.handled);
  delete ga5;

  // S11 在途请求跨日（评审竞态回归）：代次必须在解析身份的临界区固定。
  // 竞态时序：昨日解析身份并发布请求 -> 跨日清理（代次+1、剥离绑定）->
  // 登记沿用解析时快照（不得重读当前值）-> 昨日编号迟到响应按快照拒绝回绑
  {
    auto day2 = std::make_shared<std::string>("2026-09-30");
    IdentityGallery::Config cfg;
    IdentityGallery gy(cfg, [day2] { return *day2; });
    const std::string lid = gy.create(unit(9), "t-z1", w.now);
    CHECK(gy.set_handled_by_track("t-z1"));
    const uint64_t gen_req = gy.day_generation();  // 解析身份时固定（持锁快照）

    *day2 = "2026-10-01";  // 发布与登记之间发生跨日清理（竞态窗口）
    CHECK(gy.check_daily_reset());
    CHECK(gy.day_generation() == gen_req + 1);
    IdentityRecord rz;
    CHECK(gy.get(lid, rz) && rz.handled && rz.global_person_id.empty());

    RequestTracker tr2;
    tr2.add("req-cross-day", lid, "t-z1", 0.0, w.now + 60.0, gen_req);
    CloudOutcome oc2;
    oc2.succeeded = true;
    oc2.is_match = true;
    oc2.global_person_id = "global-person-00009";  // 昨日编号（云端每日清零复用）
    oc2.match_type = 0;
    PendingCloud info2;
    CHECK(tr2.on_response("req-cross-day", oc2, info2));
    CHECK(info2.day_generation == gen_req);
    CHECK(!gy.bind_global(info2.local_reid_id, oc2.global_person_id,
                          oc2.match_type, info2.day_generation));  // 拒绝回绑
    CHECK(gy.get(lid, rz) && rz.global_person_id.empty() && rz.handled);
    CHECK(gy.bind_global(lid, "global-person-00009", 0,
                         gy.day_generation()));  // 今日请求正常绑定
    std::printf("[S11] 在途请求跨日：解析时代次登记，昨日编号迟到回绑被拒"
                "（handled 资格保留）\n");
  }

  delete ga3;
  delete gb;
  std::printf("\n%d/%d passed — 逻辑链路(端侧判定/云端合并/扇出/重启找回/"
              "迟到绑定/跨天清零/跨日在途拒绝回绑)全部按预期\n",
              g_total - g_fail, g_total);
  return g_fail == 0 ? 0 : 1;
}
