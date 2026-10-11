// -*- coding: utf-8 -*-
// reid_cloud_proxy 节点：云端请求代理 + 全局人员库（我们维护的"云端"侧，
// 现场部署时与狗同网运行；真正的跨网云端化属后续工作）。
//
// 输入：
//   /reid/cloud/request (ReIDRequest)        各狗的检索/保存请求
// 输出：
//   /reid/cloud/response (ReIDResponse)      按 request_id 关联应答
//   /reid/cloud/library_update (ReIDLibraryUpdate)
//       每次库变化（合并/新建）后扇出权威模板，其他狗的端侧库落地
//   /reid/cloud_proxy/status (std_msgs/String)  1Hz JSON 状态
//
// 全局库当天有效、每日清零（与端侧一致）。模板为所有设备贡献的运行均值。
// 单线程执行器即可：节点内部不等待任何外部应答，无自锁风险。
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "dog_patrol_reid_provider/msg/match.hpp"
#include "dog_patrol_reid_provider/msg/re_id_library_update.hpp"
#include "dog_patrol_reid_provider/msg/re_id_request.hpp"
#include "dog_patrol_reid_provider/msg/re_id_response.hpp"

#include "dog_patrol_reid_provider/cloud_library.hpp"

namespace {

using dog_patrol_reid_provider::CloudLibrary;
using dog_patrol_reid_provider::CloudRecord;
using dog_patrol_reid_provider::msg::Match;
using dog_patrol_reid_provider::msg::ReIDLibraryUpdate;
using dog_patrol_reid_provider::msg::ReIDRequest;
using dog_patrol_reid_provider::msg::ReIDResponse;

double steady_now() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::tm gmtime_utc(std::time_t t) {
  std::tm tm_buf{};
#ifdef _WIN32
  gmtime_s(&tm_buf, &t);
#else
  gmtime_r(&t, &tm_buf);
#endif
  return tm_buf;
}

std::string iso_utc_now() {
  const auto dur = std::chrono::system_clock::now().time_since_epoch();
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur);
  const auto nsec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(dur - sec);
  const std::tm tm_buf = gmtime_utc(static_cast<std::time_t>(sec.count()));
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03uZ",
                tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
                static_cast<unsigned>(nsec.count()) / 1000000u);
  return buf;
}

std::string gen_message_id() {
  static std::atomic<uint32_t> ctr{1};
  const uint32_t r = std::random_device{}();
  char buf[32];
  std::snprintf(buf, sizeof(buf), "msg-cloud-%04x-%04x", r & 0xFFFFu,
                ctr.fetch_add(1) & 0xFFFFu);
  return buf;
}

// "file:///data/reid/x/embedding.bin" -> "/data/reid/x/embedding.bin"
std::string path_from_url(const std::string& url) {
  const std::string prefix = "file://";
  if (url.rfind(prefix, 0) == 0) {
    return url.substr(prefix.size());
  }
  return url;
}

}  // namespace

namespace dog_patrol_reid_provider {

class ReidCloudProxyNode : public rclcpp::Node {
 public:
  ReidCloudProxyNode() : Node("reid_cloud_proxy") {
    request_topic_ =
        declare_parameter<std::string>("request_topic", "/reid/cloud/request");
    response_topic_ =
        declare_parameter<std::string>("response_topic", "/reid/cloud/response");
    update_topic_ = declare_parameter<std::string>(
        "library_update_topic", "/reid/cloud/library_update");
    status_topic_ = declare_parameter<std::string>("status_topic",
                                                   "/reid/cloud_proxy/status");
    qos_depth_ = declare_parameter<int>("qos_depth", 200);

    api_version_ = declare_parameter<std::string>("api_version", "1.0");
    model_name_ =
        declare_parameter<std::string>("model_name", "clipreid_vitb16_after");
    model_version_ = declare_parameter<std::string>("model_version", "1.0.0");

    CloudLibrary::Config lcfg;
    lcfg.threshold =
        static_cast<float>(declare_parameter<double>("threshold", 0.70));
    lcfg.max_persons =
        static_cast<size_t>(declare_parameter<int>("max_persons", 20000));
    library_ = std::make_unique<CloudLibrary>(lcfg);

    // 请求/响应按负责人约定：Reliable / Volatile / KeepLast(200)
    rclcpp::QoS req_qos(rclcpp::KeepLast(static_cast<size_t>(qos_depth_)));
    req_qos.reliable();
    req_sub_ = create_subscription<ReIDRequest>(
        request_topic_, req_qos,
        [this](const ReIDRequest::SharedPtr msg) { on_request(msg); });
    resp_pub_ = create_publisher<ReIDResponse>(response_topic_, req_qos);

    // 扇出：Transient Local——重启/晚加入的端侧节点能补到最近 depth 条更新
    //（无全量同步服务前的近似恢复手段；发布/订阅两侧 durability 必须一致）
    rclcpp::QoS upd_qos(rclcpp::KeepLast(static_cast<size_t>(qos_depth_)));
    upd_qos.reliable();
    upd_qos.transient_local();
    update_pub_ = create_publisher<ReIDLibraryUpdate>(update_topic_, upd_qos);

    status_pub_ =
        create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));
    tick_timer_ = create_wall_timer(std::chrono::seconds(1),
                                    [this] { on_tick(); });

    RCLCPP_INFO(get_logger(),
                "reid_cloud_proxy 就绪（req=%s, update=%s, model=%s/%s, "
                "threshold=%.2f）",
                request_topic_.c_str(), update_topic_.c_str(),
                model_name_.c_str(), model_version_.c_str(), lcfg.threshold);
  }

 private:
  void on_request(const ReIDRequest::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(lib_mtx_);
    library_->check_daily_reset();
    ++req_count_;

    ReIDResponse resp;
    resp.version = api_version_;
    resp.message_id = gen_message_id();
    resp.timestamp = iso_utc_now();
    resp.device_id = msg->device_id;   // 回给发起设备
    resp.node_name = get_fully_qualified_name();
    resp.request_id = msg->request_id;  // 关联键，原样返回
    resp.similarity = -1.0F;
    resp.match_type = 255;
    resp.error_code = "";
    resp.error_message = "";
    resp.other_info = "{}";

    // ---- 校验 ----
    const bool model_ok = msg->model_name == model_name_ &&
                          msg->model_version == model_version_;
    if (!model_ok) {
      fail(resp, "MODEL_MISMATCH",
           "期望 " + model_name_ + "/" + model_version_ + "，收到 " +
               msg->model_name + "/" + msg->model_version);
    } else if (msg->embedding_dtype != "float32") {
      fail(resp, "UNSUPPORTED_DTYPE",
          "当前只支持 float32，收到 " + msg->embedding_dtype);
    } else if (msg->operation != "search" && msg->operation != "upsert" &&
               msg->operation != "search_and_upsert") {
      fail(resp, "BAD_OPERATION", "未知 operation: " + msg->operation);
    } else {
      std::vector<float> feat;
      std::string err;
      if (!read_embedding(msg->embedding_url, msg->embedding_dimension, feat,
                          err)) {
        fail(resp, "PAYLOAD_UNREADABLE", err);
      } else {
        handle_operation(*msg, feat, resp);
      }
    }
    resp_pub_->publish(resp);
    if (resp.status != "succeeded") {
      RCLCPP_WARN(get_logger(), "[proxy] request_id=%s -> %s(%s)",
                  msg->request_id.c_str(), resp.status.c_str(),
                  resp.error_code.c_str());
    }
  }

  void handle_operation(const ReIDRequest& msg, const std::vector<float>& feat,
                        ReIDResponse& resp) {
    const double now = steady_now();
    const float min_sim =
        msg.min_similarity > 0.0F ? msg.min_similarity : threshold_;
    const size_t max_results =
        msg.max_results > 0 ? static_cast<size_t>(msg.max_results) : 3;

    std::string gid;
    bool is_match = false;
    if (msg.operation != "search") {
      // upsert / search_and_upsert：命中并入、未命中新建，两种都返回有效 gid
      const CloudLibrary::UpsertResult r =
          library_->upsert(feat, msg.device_id, now);
      gid = r.global_person_id;
      is_match = r.is_match;
      ++fanout_count_;
    }

    // matches 列表（upsert 后再检索，模板已含本次贡献）
    const std::vector<dog_patrol_reid_provider::CloudSearchHit> hits =
        library_->search(feat, min_sim, max_results);
    resp.status = "succeeded";
    resp.is_match = is_match;
    resp.global_person_id = gid;
    if (!hits.empty()) {
      resp.similarity = hits.front().similarity;
      for (const auto& h : hits) {
        Match m;
        m.person_id = h.global_person_id;
        m.similarity = h.similarity;
        resp.matches.push_back(m);
      }
      if (gid.empty()) {  // 纯 search：取最优命中
        resp.global_person_id = hits.front().global_person_id;
        resp.is_match = true;
      }
    }
    if (!gid.empty()) {
      CloudRecord rec;
      if (library_->get(gid, rec)) {
        resp.match_type = rec.match_type;
      }
      if (msg.operation != "search") {
        fan_out(gid, msg.device_id, now);
      }
    }
  }

  // 库变化后扇出权威模板（云端合并结果，含所有狗的贡献）
  void fan_out(const std::string& gid, const std::string& origin_device,
               double now) {
    CloudRecord rec;
    if (!library_->get(gid, rec)) {
      return;
    }
    ReIDLibraryUpdate u;
    u.version = api_version_;
    u.message_id = gen_message_id();
    u.timestamp = iso_utc_now();
    u.device_id = origin_device;  // 接收方跳过自己
    u.day = library_->today_string();
    u.global_person_id = gid;
    u.embedding = rec.tmpl;
    u.merge_count = rec.merge_count;
    u.match_type = rec.match_type;
    u.model_name = model_name_;
    u.model_version = model_version_;
    u.other_info = "{}";
    update_pub_->publish(u);
    (void)now;
  }

  static void fail(ReIDResponse& resp, const std::string& code,
                   const std::string& message) {
    resp.status = "rejected";
    resp.is_match = false;
    resp.similarity = -1.0F;
    resp.global_person_id = "";
    resp.error_code = code;
    resp.error_message = message;
  }

  static bool read_embedding(const std::string& url, uint32_t dim,
                             std::vector<float>& feat, std::string& err) {
    const std::string path = path_from_url(url);
    if (path.empty()) {
      err = "embedding_url 为空";
      return false;
    }
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
      err = "文件不存在: " + path;
      return false;
    }
    std::ifstream f(path, std::ios::binary);
    if (!f) {
      err = "无法打开: " + path;
      return false;
    }
    if (dim == 0) {
      err = "embedding_dimension 为 0";
      return false;
    }
    feat.assign(dim, 0.0F);
    f.read(reinterpret_cast<char*>(feat.data()),
           static_cast<std::streamsize>(dim * sizeof(float)));
    if (static_cast<size_t>(f.gcount()) != dim * sizeof(float)) {
      err = "文件长度与维度不符: " + path;
      feat.clear();
      return false;
    }
    return true;
  }

  void on_tick() {
    {
      std::lock_guard<std::mutex> lk(lib_mtx_);
      if (library_->check_daily_reset()) {
        RCLCPP_INFO(get_logger(), "[proxy] 跨天清零完成");
      }
    }
    std_msgs::msg::String st;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "{\"ts\":%.1f,\"library\":%zu,\"requests\":%u,"
                  "\"fanouts\":%u}",
                  steady_now(), library_->size(), req_count_.load(),
                  fanout_count_.load());
    st.data = buf;
    status_pub_->publish(st);
  }

  // ---- 成员 ----
  std::string request_topic_, response_topic_, update_topic_, status_topic_;
  int qos_depth_ = 200;
  std::string api_version_, model_name_, model_version_;
  float threshold_ = 0.70F;

  mutable std::mutex lib_mtx_;
  std::unique_ptr<CloudLibrary> library_;
  std::atomic<uint32_t> req_count_{0};
  std::atomic<uint32_t> fanout_count_{0};

  rclcpp::Subscription<ReIDRequest>::SharedPtr req_sub_;
  rclcpp::Publisher<ReIDResponse>::SharedPtr resp_pub_;
  rclcpp::Publisher<ReIDLibraryUpdate>::SharedPtr update_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace dog_patrol_reid_provider

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dog_patrol_reid_provider::ReidCloudProxyNode>());
  rclcpp::shutdown();
  return 0;
}
