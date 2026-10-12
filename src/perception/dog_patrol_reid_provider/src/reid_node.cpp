// -*- coding: utf-8 -*-
// perception_reid 节点：ReID 行人去重（新架构，设计见
// shorttermreid/docs/reid_dedup_interface_draft.md 与
// 任务总控调度模块-设计 v1.1）。
//
// 输入：
//   /perception/reid/dedup (service ReIDDedup)  检测节点问询：这个人今天是否见过
//   /reid/cloud/response (ReIDResponse)         云端代理的响应（request_id 关联）
// 输出：
//   /reid/cloud/request (ReIDRequest)           云端检索/保存请求（文件 URL 载荷）
//   /perception/reid/status (std_msgs/String)   1Hz JSON 状态（调试可观测性）
//
// 端侧库：单身份库 + 运行均值模板，每日清零；命中立即应答（云端后台回推），
// 未命中发 search_and_upsert 并限时等待（默认 3s）超时降级为仅端侧结论。
// 执行器必须用 MultiThreadedExecutor：service 等待云端期间响应回调要能并发执行。
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "dog_patrol_reid_provider/msg/re_id_library_update.hpp"
#include "dog_patrol_reid_provider/msg/re_id_handled.hpp"
#include "dog_patrol_reid_provider/msg/re_id_request.hpp"
#include "dog_patrol_reid_provider/msg/re_id_response.hpp"
#include "dog_patrol_reid_provider/srv/re_id_dedup.hpp"

#include "dog_patrol_reid_provider/feature_extractor.hpp"
#include "dog_patrol_reid_provider/reid_gallery.hpp"
#include "dog_patrol_reid_provider/request_tracker.hpp"

namespace {

using dog_patrol_reid_provider::CloudOutcome;
using dog_patrol_reid_provider::FeatureExtractor;
using dog_patrol_reid_provider::IdentityGallery;
using dog_patrol_reid_provider::IdentityRecord;
using dog_patrol_reid_provider::PendingCloud;
using dog_patrol_reid_provider::RequestTracker;
using dog_patrol_reid_provider::SearchOutcome;
using dog_patrol_reid_provider::msg::ReIDHandled;
using dog_patrol_reid_provider::msg::ReIDLibraryUpdate;
using dog_patrol_reid_provider::msg::ReIDRequest;
using dog_patrol_reid_provider::msg::ReIDResponse;
using dog_patrol_reid_provider::srv::ReIDDedup;

double steady_now() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// 平台中立的 UTC 时间（Orin/Linux 与 Windows 各走标准安全接口）
std::tm gmtime_utc(std::time_t t) {
  std::tm tm_buf{};
#ifdef _WIN32
  gmtime_s(&tm_buf, &t);
#else
  gmtime_r(&t, &tm_buf);
#endif
  return tm_buf;
}

std::string iso_utc(std::time_t t, uint32_t nanosec) {
  const std::tm tm_buf = gmtime_utc(t);
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03uZ",
                tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
                nanosec / 1000000u);
  return buf;
}

std::string iso_utc_now() {
  const auto dur = std::chrono::system_clock::now().time_since_epoch();
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur);
  const auto nsec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(dur - sec);
  return iso_utc(static_cast<std::time_t>(sec.count()),
                 static_cast<uint32_t>(nsec.count()));
}

std::string gen_request_id() {
  static std::atomic<uint32_t> ctr{1};
  const uint32_t r = std::random_device{}();
  char buf[32];
  std::snprintf(buf, sizeof(buf), "req-reid-%04x-%04x", r & 0xFFFFu,
                ctr.fetch_add(1) & 0xFFFFu);
  return buf;
}

// 响应取值（srv 只留注释不声明常量，节点内用本地常量保持可读）
constexpr uint8_t kStatusOk = 0;
constexpr uint8_t kStatusError = 1;

}  // namespace

namespace dog_patrol_reid_provider {

class ReidNode : public rclcpp::Node {
 public:
  ReidNode() : Node("perception_reid") {
    // ---- 接口参数 ----
    service_name_ = declare_parameter<std::string>(
        "service_name", "/perception/reid/dedup");
    status_topic_ = declare_parameter<std::string>("status_topic",
                                                   "/perception/reid/status");
    cloud_request_topic_ =
        declare_parameter<std::string>("cloud_request_topic", "/reid/cloud/request");
    cloud_response_topic_ =
        declare_parameter<std::string>("cloud_response_topic", "/reid/cloud/response");
    library_update_topic_ = declare_parameter<std::string>(
        "library_update_topic", "/reid/cloud/library_update");
    handled_topic_ = declare_parameter<std::string>(
        "handled_topic", "/perception/reid/handled");
    apply_cloud_updates_ =
        declare_parameter<bool>("cloud.apply_library_updates", true);
    cloud_qos_depth_ = declare_parameter<int>("cloud_qos_depth", 200);

    // ---- 云端交互 ----
    cloud_enable_ = declare_parameter<bool>("cloud.enable", true);
    cloud_timeout_ms_ = declare_parameter<int>("cloud.timeout_ms", 3000);
    cloud_grace_sec_ = declare_parameter<double>("cloud.grace_sec", 60.0);
    cloud_max_results_ = declare_parameter<int>("cloud.max_results", 3);
    device_id_ = declare_parameter<std::string>("device_id", "");
    api_version_ = declare_parameter<std::string>("api_version", "1.0");
    model_name_ =
        declare_parameter<std::string>("model_name", "clipreid_vitb16_after");
    model_version_ = declare_parameter<std::string>("model_version", "1.0.0");
    data_dir_ = declare_parameter<std::string>("data_dir", "/data/reid");

    // ---- 模型 ----
    FeatureExtractor::Config ecfg;
    ecfg.model_path = declare_parameter<std::string>("model_path", "");
    ecfg.device = declare_parameter<std::string>("device", "cuda");
    ecfg.input_height = declare_parameter<int>("input_height", 256);
    ecfg.input_width = declare_parameter<int>("input_width", 128);
    extractor_ = FeatureExtractor::Create(ecfg);
    if (!extractor_) {
      RCLCPP_ERROR(get_logger(), "特征提取器创建失败（device=%s, model_path=%s）",
                   ecfg.device.c_str(),
                   ecfg.model_path.empty() ? "<空>" : ecfg.model_path.c_str());
    } else {
      // 启动即预加载（一次性成本：TRT engine 反序列化+预热约 1s，CPU 后端
      // 秒级），消除首次问询的冷启动。失败不致命：首次问询会再试。
      const double t0 = steady_now();
      if (extractor_->load()) {
        RCLCPP_INFO(get_logger(), "模型预加载完成（%.2fs）", steady_now() - t0);
      } else {
        RCLCPP_WARN(get_logger(), "模型预加载失败，将在首次问询时重试");
      }
      last_use_time_ = steady_now();
    }

    // ---- 端侧库 ----
    IdentityGallery::Config gcfg;
    threshold_ = declare_parameter<double>("threshold", 0.70);
    gcfg.threshold = static_cast<float>(threshold_);
    gcfg.track_assoc_threshold = static_cast<float>(
        declare_parameter<double>("track_assoc_threshold", 0.50));
    gcfg.max_identities =
        static_cast<size_t>(declare_parameter<int>("max_identities", 10000));
    gallery_ = std::make_unique<IdentityGallery>(gcfg);
    track_ttl_sec_ = declare_parameter<double>("track_ttl_sec", 300.0);
    update_min_delta_ = declare_parameter<double>("update_min_delta", 0.01);
    min_det_conf_ = declare_parameter<double>("min_det_conf", 0.0);

    // ---- 当日库持久化（应对每天多次开关机：重启恢复当日库，跨天作废）----
    persist_enable_ = declare_parameter<bool>("persist.enable", true);
    persist_save_interval_sec_ =
        declare_parameter<double>("persist.save_interval_sec", 10.0);
    last_persist_ = steady_now();
    if (persist_enable_) {
      std::error_code ec;
      std::filesystem::create_directories(data_dir_, ec);  // 已存在则无操作
      if (ec) {
        RCLCPP_WARN(get_logger(), "data_dir %s 创建失败（%s），当日库将无法持久化",
                    data_dir_.c_str(), ec.message().c_str());
      }
      persist_path_ = data_dir_ + "/library.bin";
      const IdentityGallery::PersistMeta meta{model_name_, model_version_};
      std::lock_guard<std::mutex> lk(gallery_mtx_);
      const auto lr = gallery_->LoadLatest(persist_path_, meta);
      switch (lr.status) {
        case IdentityGallery::LoadStatus::kOk:
          RCLCPP_INFO(get_logger(),
                      "已从 %s 恢复当日库：%zu 个身份%s（重启不清零，跨天自动作废）",
                      persist_path_.c_str(), lr.records,
                      lr.from_tmp ? "，自残留临时文件提升" : "");
          break;
        case IdentityGallery::LoadStatus::kMissing:
          break;  // 首次启动，无文件属正常
        case IdentityGallery::LoadStatus::kWrongDay:
          RCLCPP_INFO(get_logger(),
                      "库文件 %s 为其他日期（每日清零设计），从空库开始",
                      persist_path_.c_str());
          break;
        case IdentityGallery::LoadStatus::kModelMismatch:
          RCLCPP_WARN(get_logger(),
                      "库文件 %s 模型/版本不符，不加载，从空库开始",
                      persist_path_.c_str());
          break;
        case IdentityGallery::LoadStatus::kCorrupt:
          RCLCPP_WARN(get_logger(), "库文件 %s 损坏，不加载，从空库开始",
                      persist_path_.c_str());
          break;
      }
    }

    // ---- 载荷与模型驻留 ----
    thumb_width_ = declare_parameter<int>("thumb_width", 96);
    thumb_height_ = declare_parameter<int>("thumb_height", 192);
    thumb_quality_ = declare_parameter<int>("thumb_quality", 60);
    model_idle_unload_sec_ =
        declare_parameter<double>("model_idle_unload_sec", 0.0);
    last_use_time_ = steady_now();

    // ---- 接口 ----
    srv_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cloud_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    dedup_srv_ = create_service<ReIDDedup>(
        service_name_,
        [this](const std::shared_ptr<ReIDDedup::Request> req,
               std::shared_ptr<ReIDDedup::Response> resp) {
          on_dedup(req, resp);
        },
        // Humble 带 callback group 的重载要求 rmw_qos_profile_t
        rclcpp::ServicesQoS().get_rmw_qos_profile(), srv_cb_group_);

    rclcpp::QoS cloud_qos(rclcpp::KeepLast(
        static_cast<size_t>(cloud_qos_depth_)));
    cloud_qos.reliable();
    cloud_pub_ = create_publisher<ReIDRequest>(cloud_request_topic_, cloud_qos);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cloud_cb_group_;
    cloud_sub_ = create_subscription<ReIDResponse>(
        cloud_response_topic_, cloud_qos,
        [this](const ReIDResponse::SharedPtr msg) { on_cloud_response(msg); },
        sub_opts);

    if (apply_cloud_updates_) {
      // 扇出订阅：与代理发布侧一致 Transient Local，晚加入/重启可补最近若干条
      rclcpp::QoS upd_qos(
          rclcpp::KeepLast(static_cast<size_t>(cloud_qos_depth_)));
      upd_qos.reliable();
      upd_qos.transient_local();
      rclcpp::SubscriptionOptions upd_opts;
      upd_opts.callback_group = cloud_cb_group_;
      update_sub_ = create_subscription<ReIDLibraryUpdate>(
          library_update_topic_, upd_qos,
          [this](const ReIDLibraryUpdate::SharedPtr msg) {
            on_library_update(msg);
          },
          upd_opts);
    }

    // 任务侧 handled 结论下发（R06-A：业务处置资格与外观库分开维护）
    rclcpp::SubscriptionOptions handled_opts;
    handled_opts.callback_group = cloud_cb_group_;
    handled_sub_ = create_subscription<ReIDHandled>(
        handled_topic_, rclcpp::QoS(10),
        [this](const ReIDHandled::SharedPtr msg) { on_handled(msg); },
        handled_opts);

    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_,
                                                          rclcpp::QoS(10));
    tick_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this] { on_tick(); });

    RCLCPP_INFO(get_logger(),
                "perception_reid 就绪（service=%s, model=%s, device=%s, "
                "threshold=%.2f, cloud=%s(timeout %dms), data_dir=%s）",
                service_name_.c_str(),
                ecfg.model_path.empty() ? "<未配置!>" : ecfg.model_path.c_str(),
                ecfg.device.c_str(), gcfg.threshold,
                cloud_enable_ ? "on" : "off", cloud_timeout_ms_,
                data_dir_.c_str());
    if (device_id_.empty()) {
      RCLCPP_WARN(get_logger(),
                  "device_id 未配置：云端请求将带空 device_id，多狗部署前必须设置");
    }
  }

  ~ReidNode() override {
    if (!persist_enable_ || !gallery_) {
      return;
    }
    // 正常退出的兜底落盘；突然断电由周期落盘兜底（最多丢一个防抖窗口）
    std::lock_guard<std::mutex> lk(gallery_mtx_);
    if (gallery_->dirty()) {
      const bool ok =
          gallery_->SaveToFile(persist_path_, {model_name_, model_version_});
      std::fprintf(stderr, "[perception_reid] 退出落盘%s（%s）\n",
                   ok ? "完成" : "失败", persist_path_.c_str());
    }
  }

 private:
  struct PayloadResult {
    bool ok = false;
    std::string thumb_url;
    std::string emb_url;
    std::string err;
    size_t dim = 0;
  };

  // ---------------------------------------------------------------- service

  void on_dedup(const std::shared_ptr<ReIDDedup::Request> req,
                std::shared_ptr<ReIDDedup::Response> resp) {
    const double now = steady_now();
    resp->status = kStatusError;
    resp->track_id = req->track_id;  // 回显对齐
    resp->is_seen = false;

    if (req->track_id.empty()) {
      resp->error_message = "track_id 为空";
      return;
    }
    cv::Mat crop;
    if (!to_mat(req->crop, crop)) {
      resp->error_message = "crop 无效（需 BGR8、非空、步长一致）";
      return;
    }

    std::vector<float> feat;
    {
      std::lock_guard<std::mutex> lk(model_mtx_);
      if (!extractor_) {
        resp->error_message = "特征提取器不可用（创建失败，见启动日志）";
        return;
      }
      if (!extractor_->loaded() && !extractor_->load()) {
        resp->error_message = "模型不可用（看 model_path/device 参数）";
        return;
      }
      feat = extractor_->extract(crop);
    }
    last_use_time_ = steady_now();
    if (feat.empty()) {
      resp->error_message = "特征提取失败";
      return;
    }
    const bool quality_ok =
        req->det_conf >= static_cast<float>(min_det_conf_);

    // ---- 端侧检索 ----
    SearchOutcome sr;
    {
      std::lock_guard<std::mutex> lk(gallery_mtx_);
      gallery_->check_daily_reset();
      sr = gallery_->search(feat, req->track_id);
    }

    if (sr.hit) {
      on_edge_hit(*req, *resp, sr, feat, crop, quality_ok, now);
      return;
    }
    on_edge_miss(*req, *resp, feat, crop, quality_ok, now);
  }

  // 端侧命中：并入均值模板，立即应答；模板移动超过阈值才后台回推云端
  void on_edge_hit(const ReIDDedup::Request& req, ReIDDedup::Response& resp,
                   const SearchOutcome& sr, const std::vector<float>& feat,
                   const cv::Mat& crop, bool quality_ok, double now) {
    float delta = -1.0F;
    {
      std::lock_guard<std::mutex> lk(gallery_mtx_);
      if (quality_ok) {
        delta = gallery_->merge(sr.local_reid_id, feat, req.track_id, now);
      }
    }
    IdentityRecord rec;
    {
      std::lock_guard<std::mutex> lk(gallery_mtx_);
      gallery_->get(sr.local_reid_id, rec);
    }
    resp.status = kStatusOk;
    resp.is_seen = true;
    resp.handled = rec.handled;  // 已处置结论随命中透传（换 track/重启仍有效）

    if (cloud_enable_ && quality_ok && delta >= 0.0F &&
        delta >= update_min_delta_) {
      publish_upsert(sr.local_reid_id, req.track_id, rec.tmpl, crop, req, now);
    }
    RCLCPP_INFO(get_logger(),
                "[dedup] track=%s -> EDGE_HIT %s (sim=%.3f%s, merges=%u, "
                "handled=%u)",
                req.track_id.c_str(), sr.local_reid_id.c_str(), sr.similarity,
                sr.via_track ? ", 经track兜底" : "", rec.merge_count,
                rec.handled ? 1u : 0u);
  }

  // 端侧未命中：建身份；云端可用则 search_and_upsert 限时等待。
  // 对检测节点只暴露 is_seen；端侧/云端命中之分仅体现在节点日志里
  void on_edge_miss(const ReIDDedup::Request& req, ReIDDedup::Response& resp,
                    const std::vector<float>& feat, const cv::Mat& crop,
                    bool quality_ok, double now) {
    if (!quality_ok) {
      // 低置信度：不建身份不查询（is_seen 保持 false）
      resp.status = kStatusOk;
      RCLCPP_INFO(get_logger(),
                  "[dedup] track=%s -> NEW（det_conf=%.2f 低于阈值，未建身份）",
                  req.track_id.c_str(), req.det_conf);
      return;
    }

    std::string local_id;
    {
      std::lock_guard<std::mutex> lk(gallery_mtx_);
      local_id = gallery_->create(feat, req.track_id, now);
    }
    resp.status = kStatusOk;

    if (!cloud_enable_) {
      RCLCPP_INFO(get_logger(), "[dedup] track=%s -> NEW %s（云端关闭）",
                  req.track_id.c_str(), local_id.c_str());
      return;
    }

    // ---- 云端 search_and_upsert ----
    const std::string rid = gen_request_id();
    const PayloadResult pl = write_payload(rid, crop, feat);
    if (!pl.ok) {
      RCLCPP_WARN(get_logger(),
                  "[dedup] 云端请求未发（%s），track=%s 降级为仅端侧结论",
                  pl.err.c_str(), req.track_id.c_str());
      return;
    }
    cloud_pub_->publish(
        make_cloud_request(rid, "search_and_upsert", req.track_id, local_id,
                           pl, req));
    const double deadline = now + cloud_timeout_ms_ / 1000.0;
    tracker_.add(rid, local_id, req.track_id, deadline,
                 deadline + cloud_grace_sec_);

    CloudOutcome oc;
    bool got = false;
    while (steady_now() < deadline) {
      if (tracker_.poll(rid, oc)) {
        got = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!got) {
      got = tracker_.poll(rid, oc);  // 最后一次兜底（恰在截止后到达）
    }

    const char* cloud_state = "not_sent";
    float sim = -1.0F;
    if (got) {
      cloud_state = oc.succeeded ? "returned" : "returned_failed";
      if (oc.succeeded && oc.is_match && !oc.global_person_id.empty()) {
        resp.is_seen = true;  // 其他狗今天见过（绑定已在响应回调完成）
        sim = oc.similarity;
      } else if (!oc.succeeded) {
        RCLCPP_WARN(get_logger(),
                    "[dedup] 云端返回失败（%s），track=%s 按新人处理",
                    oc.error_code.c_str(), req.track_id.c_str());
      }
    } else {
      tracker_.mark_timeout(rid);
      cloud_state = "timeout";
      // 迟到响应仍会经回调绑定全局身份，但不再影响本次结论
    }
    RCLCPP_INFO(get_logger(),
                "[dedup] track=%s -> %s %s (cloud=%s, sim=%.3f)",
                req.track_id.c_str(), resp.is_seen ? "CLOUD_HIT" : "NEW",
                local_id.c_str(), cloud_state, sim);
  }

  // ---------------------------------------------------------------- 云端

  void publish_upsert(const std::string& local_id, const std::string& track_id,
                      const std::vector<float>& tmpl, const cv::Mat& crop,
                      const ReIDDedup::Request& svc_req, double now) {
    const std::string rid = gen_request_id();
    const PayloadResult pl = write_payload(rid, crop, tmpl);
    if (!pl.ok) {
      RCLCPP_WARN(get_logger(), "[upsert] 载荷未写（%s），跳过本次回推",
                  pl.err.c_str());
      return;
    }
    cloud_pub_->publish(
        make_cloud_request(rid, "upsert", track_id, local_id, pl, svc_req));
    tracker_.add(rid, local_id, track_id, /*await_deadline=*/0.0,
                 now + cloud_grace_sec_);  // fire-and-forget，仅收迟到绑定
  }

  ReIDRequest make_cloud_request(const std::string& rid,
                                 const std::string& op,
                                 const std::string& track_id,
                                 const std::string& local_id,
                                 const PayloadResult& pl,
                                 const ReIDDedup::Request& svc) const {
    ReIDRequest m;
    m.version = api_version_;
    m.message_id = rid + "-m";
    m.timestamp = iso_utc_now();
    m.device_id = device_id_;
    m.node_name = get_fully_qualified_name();
    m.request_id = rid;
    m.task_id = svc.task_id;
    m.event_id = svc.event_id;
    m.track_id = track_id;
    m.local_reid_id = local_id;
    m.operation = op;
    m.thumbnail_url = pl.thumb_url;
    m.embedding_url = pl.emb_url;
    m.embedding_dimension = static_cast<uint32_t>(pl.dim);
    m.embedding_dtype = "float32";
    m.model_name = model_name_;
    m.model_version = model_version_;
    m.max_results = static_cast<uint32_t>(cloud_max_results_);
    m.min_similarity = static_cast<float>(threshold_);
    const std::string ts =
        svc.header.stamp.sec > 0
            ? iso_utc(svc.header.stamp.sec, svc.header.stamp.nanosec)
            : iso_utc_now();
    std::string other = "{";
    if (!svc.camera_id.empty()) {
      other += "\"camera_id\":\"" + svc.camera_id + "\",";
    }
    other += "\"capture_timestamp\":\"" + ts + "\"}";
    m.other_info = other;
    return m;
  }

  void on_cloud_response(const ReIDResponse::SharedPtr msg) {
    CloudOutcome oc;
    oc.succeeded = msg->status == "succeeded";
    oc.is_match = msg->is_match;
    oc.similarity = msg->similarity;
    oc.global_person_id = msg->global_person_id;
    oc.match_type = msg->match_type;
    oc.error_code = msg->error_code;

    PendingCloud info;
    if (!tracker_.on_response(msg->request_id, oc, info)) {
      RCLCPP_WARN(get_logger(), "[cloud] 未知/重复响应 request_id=%s，忽略",
                  msg->request_id.c_str());
      return;
    }
    if (oc.succeeded && !oc.global_person_id.empty()) {
      // 命中=绑定已有全局身份；未命中=云端本次新建的身份 ID，同样绑定
      //（后续 upsert/扇出跳过、跨狗关联都靠它）
      std::lock_guard<std::mutex> lk(gallery_mtx_);
      if (gallery_->bind_global(info.local_reid_id, oc.global_person_id,
                                oc.match_type)) {
        RCLCPP_INFO(get_logger(),
                    "[cloud] %s 绑定全局身份 %s（match_type=%u, sim=%.3f%s）",
                    info.local_reid_id.c_str(), oc.global_person_id.c_str(),
                    oc.match_type, oc.similarity,
                    info.timed_out_reported ? ", 迟到绑定" : "");
      }
    } else if (!oc.succeeded) {
      RCLCPP_WARN(get_logger(),
                  "[cloud] 响应失败 request_id=%s err=%s/%s",
                  msg->request_id.c_str(), msg->error_code.c_str(),
                  msg->error_message.c_str());
    }
  }

  // 任务侧 handled 结论落地（R06-A）：优先按 track 绑定定位身份；绑定过期时
  // 用消息附带的 crop 提特征兜底（需过身份阈值，避免误标他人）
  void on_handled(const ReIDHandled::SharedPtr msg) {
    if (msg->clear_all) {
      size_t n = 0;
      {
        std::lock_guard<std::mutex> lk(gallery_mtx_);
        gallery_->check_daily_reset();
        n = gallery_->clear_handled();
      }
      RCLCPP_INFO(get_logger(), "[handled] 任务 reset：清除 handled 标记 %zu 个", n);
      return;
    }
    if (msg->track_id.empty()) {
      return;
    }
    bool ok = false;
    const char* how = "track绑定";
    {
      std::lock_guard<std::mutex> lk(gallery_mtx_);
      gallery_->check_daily_reset();
      ok = gallery_->set_handled_by_track(msg->track_id);
    }
    if (!ok) {
      cv::Mat crop;
      if (!to_mat(msg->crop, crop) || crop.empty()) {
        RCLCPP_WARN(get_logger(),
                    "[handled] track=%s 无绑定且无兜底 crop，未标记",
                    msg->track_id.c_str());
        return;
      }
      std::vector<float> feat;
      {
        std::lock_guard<std::mutex> lk(model_mtx_);
        if (!extractor_->loaded() && !extractor_->load()) {
          RCLCPP_WARN(get_logger(), "[handled] 模型不可用，特征兜底失败");
          return;
        }
        feat = extractor_->extract(crop);
      }
      last_use_time_ = steady_now();
      if (feat.empty()) {
        RCLCPP_WARN(get_logger(), "[handled] 兜底特征提取失败");
        return;
      }
      how = "特征检索";
      {
        std::lock_guard<std::mutex> lk(gallery_mtx_);
        ok = gallery_->set_handled_by_feature(feat);
      }
    }
    if (ok) {
      RCLCPP_INFO(get_logger(), "[handled] track=%s 已标记处置（按%s）",
                  msg->track_id.c_str(), how);
    } else {
      RCLCPP_WARN(get_logger(),
                  "[handled] track=%s 按%s未定位到可信身份，未标记",
                  msg->track_id.c_str(), how);
    }
  }

  // 云端扇出落地：其他狗贡献的权威模板进端侧库（跳过自己/跨日/模型不符）
  void on_library_update(const ReIDLibraryUpdate::SharedPtr msg) {
    if (msg->device_id == device_id_) {
      return;  // 自己触发的更新，本地视角已有
    }
    if (msg->day != IdentityGallery::today_string()) {
      RCLCPP_INFO(get_logger(), "[library_update] 丢弃跨日更新 %s（本地 %s）",
                  msg->day.c_str(), IdentityGallery::today_string().c_str());
      return;
    }
    if (msg->model_name != model_name_ || msg->model_version != model_version_) {
      RCLCPP_WARN(get_logger(), "[library_update] 模型不匹配(%s/%s)，丢弃",
                  msg->model_name.c_str(), msg->model_version.c_str());
      return;
    }
    if (msg->embedding.empty()) {
      return;
    }
    const double now = steady_now();
    std::lock_guard<std::mutex> lk(gallery_mtx_);
    gallery_->check_daily_reset();
    const std::string local_id = gallery_->upsert_global(
        msg->global_person_id, msg->embedding, msg->match_type,
        msg->merge_count, now);
    RCLCPP_INFO(get_logger(),
                "[library_update] %s -> %s（来自 %s, match_type=%u, merges=%u）",
                msg->global_person_id.c_str(), local_id.c_str(),
                msg->device_id.c_str(), msg->match_type, msg->merge_count);
  }

  // ---------------------------------------------------------------- 载荷

  PayloadResult write_payload(const std::string& rid, const cv::Mat& crop,
                              const std::vector<float>& feat) const {
    namespace fs = std::filesystem;
    PayloadResult r;
    std::error_code ec;
    const fs::path dir = fs::path(data_dir_) / rid;
    fs::create_directories(dir, ec);
    if (ec) {
      r.err = "mkdir: " + ec.message();
      return r;
    }
    cv::Mat thumb;
    cv::resize(crop, thumb, cv::Size(thumb_width_, thumb_height_), 0, 0,
               cv::INTER_AREA);
    std::string ext = ".webp";
    std::vector<uchar> buf;
    try {
      const std::vector<int> par{cv::IMWRITE_WEBP_QUALITY, thumb_quality_};
      cv::imencode(ext, thumb, buf, par);
    } catch (const cv::Exception&) {
      // 本机 OpenCV 无 WEBP 时退回 JPEG（URL 扩展名与实际文件保持一致）
      ext = ".jpg";
      buf.clear();
      try {
        const std::vector<int> par{
            cv::IMWRITE_JPEG_QUALITY,
            thumb_quality_ < 50 ? thumb_quality_ : 50};
        cv::imencode(ext, thumb, buf, par);
      } catch (const cv::Exception& e) {
        r.err = std::string("encode: ") + e.what();
        return r;
      }
    }
    const fs::path thumb_p = dir / ("thumbnail" + ext);
    const fs::path emb_p = dir / "embedding.bin";
    {
      std::ofstream f(thumb_p, std::ios::binary);
      f.write(reinterpret_cast<const char*>(buf.data()),
              static_cast<std::streamsize>(buf.size()));
      if (!f) {
        r.err = "write thumb";
        return r;
      }
    }
    {
      std::ofstream f(emb_p, std::ios::binary);
      f.write(reinterpret_cast<const char*>(feat.data()),
              static_cast<std::streamsize>(feat.size() * sizeof(float)));
      if (!f) {
        r.err = "write embedding";
        return r;
      }
    }
    if (static_cast<uintmax_t>(buf.size()) +
            static_cast<uintmax_t>(feat.size() * sizeof(float)) >
        10240ull) {
      fs::remove_all(dir, ec);  // 超限不发（接口约定 3.3）
      r.err = "PAYLOAD_TOO_LARGE";
      return r;
    }
    r.ok = true;
    r.dim = feat.size();
    r.thumb_url = "file://" + thumb_p.generic_string();
    r.emb_url = "file://" + emb_p.generic_string();
    return r;
  }

  static bool to_mat(const sensor_msgs::msg::Image& img, cv::Mat& out) {
    if (img.encoding != "bgr8" || img.width == 0 || img.height == 0 ||
        img.step < img.width * 3 ||
        img.data.size() <
            static_cast<size_t>(img.step) * static_cast<size_t>(img.height)) {
      return false;
    }
    out = cv::Mat(static_cast<int>(img.height), static_cast<int>(img.width),
                  CV_8UC3, const_cast<uint8_t*>(img.data.data()),
                  static_cast<size_t>(img.step))
              .clone();
    return true;
  }

  // ---------------------------------------------------------------- 维护

  void on_tick() {
    const double now = steady_now();
    {
      std::lock_guard<std::mutex> lk(gallery_mtx_);
      if (gallery_->check_daily_reset()) {
        RCLCPP_INFO(get_logger(),
                    "[tick] 跨天清零完成（云端库由云端侧服务同样清零）");
      }
      gallery_->prune_tracks(now, track_ttl_sec_);
      if (persist_enable_ && gallery_->dirty() &&
          now - last_persist_ >= persist_save_interval_sec_) {
        if (gallery_->SaveToFile(persist_path_, {model_name_, model_version_})) {
          last_persist_ = now;
        } else {
          RCLCPP_WARN(get_logger(),
                      "[tick] 当日库落盘失败（%s），下个周期重试",
                      persist_path_.c_str());
        }
      }
    }
    const std::vector<std::string> removed = tracker_.sweep(now);
    if (!removed.empty()) {
      RCLCPP_INFO(get_logger(), "[tick] 清理过期云端请求 %zu 条",
                  removed.size());
    }
    {
      std::lock_guard<std::mutex> lk(model_mtx_);
      if (extractor_ && extractor_->loaded() && model_idle_unload_sec_ > 0.0 &&
          now - last_use_time_ > model_idle_unload_sec_) {
        extractor_->unload();
        RCLCPP_INFO(get_logger(), "[tick] 空闲超过 %.0fs，模型卸载",
                    model_idle_unload_sec_);
      }
    }
    publish_status(now);
  }

  void publish_status(double now) {
    size_t lib = 0;
    bool loaded = false;
    {
      std::lock_guard<std::mutex> lk(gallery_mtx_);
      lib = gallery_->size();
    }
    {
      std::lock_guard<std::mutex> lk(model_mtx_);
      loaded = extractor_ && extractor_->loaded();
    }
    std_msgs::msg::String st;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "{\"ts\":%.1f,\"library\":%zu,\"pending_cloud\":%zu,"
                  "\"model_loaded\":%u,\"cloud_enabled\":%u,\"idle_sec\":%.0f}",
                  now, lib, tracker_.size(), loaded ? 1u : 0u,
                  cloud_enable_ ? 1u : 0u, now - last_use_time_);
    st.data = buf;
    status_pub_->publish(st);
  }

  // ---- 成员 ----
  std::string service_name_, status_topic_, cloud_request_topic_,
      cloud_response_topic_, library_update_topic_, handled_topic_;
  int cloud_qos_depth_ = 200;
  bool cloud_enable_ = true;
  bool apply_cloud_updates_ = true;
  int cloud_timeout_ms_ = 3000;
  double cloud_grace_sec_ = 60.0;
  int cloud_max_results_ = 3;
  std::string device_id_, api_version_, model_name_, model_version_, data_dir_;
  double threshold_ = 0.70;
  double track_ttl_sec_ = 300.0;
  double update_min_delta_ = 0.01;
  double min_det_conf_ = 0.0;
  int thumb_width_ = 96, thumb_height_ = 192, thumb_quality_ = 60;
  double model_idle_unload_sec_ = 300.0;
  bool persist_enable_ = true;
  double persist_save_interval_sec_ = 10.0;
  double last_persist_ = 0.0;
  std::string persist_path_;
  std::atomic<double> last_use_time_{0.0};

  mutable std::mutex gallery_mtx_;
  mutable std::mutex model_mtx_;
  std::unique_ptr<FeatureExtractor> extractor_;
  std::unique_ptr<IdentityGallery> gallery_;
  RequestTracker tracker_;

  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;
  rclcpp::CallbackGroup::SharedPtr cloud_cb_group_;
  rclcpp::Service<ReIDDedup>::SharedPtr dedup_srv_;
  rclcpp::Publisher<ReIDRequest>::SharedPtr cloud_pub_;
  rclcpp::Subscription<ReIDResponse>::SharedPtr cloud_sub_;
  rclcpp::Subscription<ReIDLibraryUpdate>::SharedPtr update_sub_;
  rclcpp::Subscription<ReIDHandled>::SharedPtr handled_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace dog_patrol_reid_provider

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // 必须 MultiThreadedExecutor：service 内等待云端响应期间，
  // 响应回调（另一 callback group）需要并发执行，否则互相等死
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<dog_patrol_reid_provider::ReidNode>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
