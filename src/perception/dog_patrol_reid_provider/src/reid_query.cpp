// -*- coding: utf-8 -*-
// 轻量 C++ 联调客户端：不依赖 Python 绑定（rosidl python 生成器在部分
// 环境不可用时，ros2 service call / sim_detection_client.py 均失效）。
// 三种模式（-p mode:=...）：
//   query   读图问询 /perception/reid/dedup，打印 is_seen/handled
//   handled 下发一条 handled 结论（-p track_id:=...，绑定过期需带 crop 时用 python 版）
//   clear   下发 clear_all（任务 reset 语义）
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "rclcpp/rclcpp.hpp"

#include "dog_patrol_reid_provider/msg/re_id_handled.hpp"
#include "dog_patrol_reid_provider/srv/re_id_dedup.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("reid_query");
  const std::string mode = node->declare_parameter<std::string>("mode", "query");
  const std::string image = node->declare_parameter<std::string>("image", "");
  const std::string track_id = node->declare_parameter<std::string>("track_id", "");
  const std::string service_name =
      node->declare_parameter<std::string>("service", "/perception/reid/dedup");
  const std::string handled_topic =
      node->declare_parameter<std::string>("handled_topic", "/perception/reid/handled");
  const double timeout = node->declare_parameter<double>("timeout", 10.0);

  using dog_patrol_reid_provider::msg::ReIDHandled;
  using dog_patrol_reid_provider::srv::ReIDDedup;

  if (mode == "handled" || mode == "clear") {
    auto pub = node->create_publisher<ReIDHandled>(handled_topic, 10);
    ReIDHandled msg;
    msg.header.stamp = node->now();
    if (mode == "handled") {
      if (track_id.empty()) {
        std::fprintf(stderr, "[reid_query] handled 模式需要 -p track_id:=...\n");
        return 2;
      }
      msg.track_id = track_id;
    } else {
      msg.clear_all = true;
    }
    // 等待订阅方完成发现再发布
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    pub->publish(msg);
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    std::printf("[reid_query] 已发布 %s（track_id=%s）\n", mode.c_str(),
                msg.track_id.c_str());
    rclcpp::shutdown();
    return 0;
  }

  if (mode != "query") {
    std::fprintf(stderr, "[reid_query] 未知 mode: %s（query|handled|clear）\n",
                 mode.c_str());
    return 2;
  }
  if (image.empty() || track_id.empty()) {
    std::fprintf(stderr, "[reid_query] query 模式需要 -p image:=... 与 -p track_id:=...\n");
    return 2;
  }
  const cv::Mat img = cv::imread(image, cv::IMREAD_COLOR);
  if (img.empty()) {
    std::fprintf(stderr, "[reid_query] 无法读取图片: %s\n", image.c_str());
    return 2;
  }

  auto cli = node->create_client<ReIDDedup>(service_name);
  if (!cli->wait_for_service(std::chrono::duration<double>(timeout))) {
    std::fprintf(stderr, "[reid_query] 服务不可用: %s\n", service_name.c_str());
    return 3;
  }
  auto req = std::make_shared<ReIDDedup::Request>();
  req->track_id = track_id;
  req->header.stamp = node->now();
  req->crop.height = img.rows;
  req->crop.width = img.cols;
  req->crop.encoding = "bgr8";
  req->crop.step = static_cast<std::uint32_t>(img.cols * 3);
  req->crop.data.assign(img.data, img.data + img.total() * img.elemSize());

  auto fut = cli->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, fut,
          std::chrono::duration<double>(timeout)) != rclcpp::FutureReturnCode::SUCCESS) {
    std::fprintf(stderr, "[reid_query] 服务应答超时（>%.1fs）\n", timeout);
    return 4;
  }
  const auto resp = fut.get();
  std::printf("track_id : %s\n"
              "status   : %u (0=OK 1=ERROR)\n"
              "is_seen  : %s   <- 今天是否见过\n"
              "handled  : %s   <- 今天是否已处置（免处置需 is_seen 且 handled）\n",
              resp->track_id.c_str(), static_cast<unsigned>(resp->status),
              resp->is_seen ? "True" : "False", resp->handled ? "True" : "False");
  if (!resp->error_message.empty()) {
    std::printf("error    : %s\n", resp->error_message.c_str());
  }
  rclcpp::shutdown();
  return 0;
}
