#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模拟人体检测节点，端到端联调 ReID 去重系统（不改动任何现有代码）。

在 Orin（ROS 2 Humble，先 colcon build 本包并 source）上使用：

  终端1  云端代理：
    ros2 launch dog_patrol_reid_provider reid_cloud_proxy.launch.py
  终端2  端侧节点（model_path 配好）：
    ros2 launch dog_patrol_reid_provider reid.launch.py
  终端3  观察云端链路（请求/响应/扇出/状态）：
    python3 scripts/sim_detection_client.py --observe
  终端4  模拟检测节点送图：
    python3 scripts/sim_detection_client.py --image xxx.jpg --track-id track-1
    同一人再送一张（应答应为 is_seen=true）：
    python3 scripts/sim_detection_client.py --image yyy.jpg --track-id track-1
    同一 track 周期刷新 / 换人换 track_id 都可以随意重复调用。

两只狗仿真（验证扇出）：第二只狗用错开的参数再起一个端侧节点——

    ros2 run dog_patrol_reid_provider reid_node --ros-args \
      -r __node:=perception_reid_2 \
      -p device_id:=m20-002 \
      -p service_name:=/perception/reid/dedup2 \
      -p status_topic:=/perception/reid/status2 \
      -p data_dir:=/data/reid2 \
      -p model_path:=/abs/path/reid_clipreid_vitb16_after.onnx

  然后把图发给第二只狗（--service /perception/reid/dedup2），观察：
  狗1送新人 -> 云端新建 -> 狗2 的 [library_update] 日志 -> 狗2 再送同人
  端侧直接命中（is_seen=true）。
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from dog_patrol_reid_provider.msg import ReIDLibraryUpdate, ReIDRequest, ReIDResponse
from dog_patrol_reid_provider.srv import ReIDDedup

MATCH_TYPE = {0: '陌生人(黑)', 1: '白名单', 2: '不确定', 255: '未知'}


def load_image_msg(path):
    """读图为 sensor_msgs/Image（BGR8），不依赖 cv_bridge。"""
    import cv2
    from sensor_msgs.msg import Image
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit(f'无法读取图片: {path}')
    msg = Image()
    msg.height, msg.width = img.shape[0], img.shape[1]
    msg.encoding = 'bgr8'
    msg.step = img.shape[1] * 3
    msg.data = img.tobytes()
    return msg


class Observer(Node):
    """订阅云端链路三个话题，打印一行摘要。"""

    def __init__(self):
        super().__init__('reid_sim_observer')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        upd_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ReIDRequest, '/reid/cloud/request',
                                 self.on_req, qos)
        self.create_subscription(ReIDResponse, '/reid/cloud/response',
                                 self.on_resp, qos)
        self.create_subscription(ReIDLibraryUpdate, '/reid/cloud/library_update',
                                 self.on_upd, upd_qos)

    def on_req(self, m):
        print(f'[req ] {m.request_id} op={m.operation} track={m.track_id} '
              f'local={m.local_reid_id} dim={m.embedding_dimension} '
              f'来自 {m.device_id} thumb={m.thumbnail_url}')

    def on_resp(self, m):
        print(f'[resp] {m.request_id} status={m.status} is_match={m.is_match} '
              f'global={m.global_person_id or "-"} sim={m.similarity:.3f} '
              f'match_type={MATCH_TYPE.get(m.match_type, m.match_type)}')

    def on_upd(self, m):
        print(f'[扇出] {m.global_person_id} 来源={m.device_id} day={m.day} '
              f'merges={m.merge_count} match_type='
              f'{MATCH_TYPE.get(m.match_type, m.match_type)} dim={len(m.embedding)}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', help='人体裁剪图路径（jpg/png）')
    ap.add_argument('--track-id', default='track-sim-1')
    ap.add_argument('--service', default='/perception/reid/dedup')
    ap.add_argument('--det-conf', type=float, default=0.9)
    ap.add_argument('--camera-id', default='')
    ap.add_argument('--task-id', default='')
    ap.add_argument('--event-id', default='')
    ap.add_argument('--timeout', type=float, default=10.0,
                    help='等服务上线/应答的超时（秒）')
    ap.add_argument('--observe', action='store_true', help='只观察云端链路话题')
    args = ap.parse_args()

    rclpy.init()
    try:
        if args.observe:
            rclpy.spin(Observer())
            return

        if not args.image:
            ap.error('需要 --image（或 --observe 观察模式）')

        node = Node('reid_sim_detection')
        cli = node.create_client(ReIDDedup, args.service)
        print(f'等待服务 {args.service} ...')
        if not cli.wait_for_service(timeout_sec=args.timeout):
            raise SystemExit(f'服务不可用: {args.service}')

        req = ReIDDedup.Request()
        req.header.stamp = node.get_clock().now().to_msg()
        req.track_id = args.track_id
        req.crop = load_image_msg(args.image)
        req.det_conf = args.det_conf
        req.camera_id = args.camera_id
        req.task_id = args.task_id
        req.event_id = args.event_id

        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)
        if not fut.done():
            raise SystemExit(f'服务应答超时（>{args.timeout}s）')
        resp = fut.result()
        print(f'track_id   : {resp.track_id}')
        print(f'status     : {resp.status} (0=OK 1=ERROR)')
        print(f'is_seen    : {resp.is_seen}  <- 今天是否见过')
        print(f'handled    : {resp.handled}   <- 今天是否已处置（免处置需 is_seen 且 handled）')
        if resp.error_message:
            print(f'error      : {resp.error_message}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
