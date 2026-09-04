#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import multiprocessing
import rclpy
from drdds.msg import NavCmd

def subscriber_domain_42(queue):
    # 接收端强制运行在我们指定的独立 Domain，动态从 device_parameters.yaml 中加载
    domain_id = "42"
    try:
        import yaml
        yaml_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../config/device_parameters.yaml"))
        if os.path.exists(yaml_path):
            with open(yaml_path, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f)
                if config_data and "/**" in config_data:
                    params = config_data["/**"].get("ros__parameters", {})
                    if "ros_domain_id" in params:
                        domain_id = str(params["ros_domain_id"])
    except Exception as e:
        print(f"Failed to read ros_domain_id in bridge, using default 42: {e}")
    os.environ["ROS_DOMAIN_ID"] = domain_id
    rclpy.init()
    node = rclpy.create_node("nav_cmd_bridge_sub")
    
    def cb(msg):
        # 为了避免跨进程序列化复杂 ROS 消息报错，我们提取核心字段打包为字典
        data = {
            'stamp_sec': msg.header.stamp.sec,
            'stamp_nanosec': msg.header.stamp.nanosec,
            'frame_id': msg.header.frame_id,
            'x_vel': msg.data.x_vel,
            'y_vel': msg.data.y_vel,
            'yaw_vel': msg.data.yaw_vel
        }
        queue.put(data)
        
    node.create_subscription(NavCmd, "/NAV_CMD", cb, 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def publisher_domain_0(queue):
    # 发送端强制运行在 Domain 0（或者狗的实际 Domain）
    os.environ["ROS_DOMAIN_ID"] = "0"
    rclpy.init()
    node = rclpy.create_node("nav_cmd_bridge_pub")
    pub = node.create_publisher(NavCmd, "/NAV_CMD", 10)
    
    while rclpy.ok():
        try:
            # 阻塞获取队列数据，超时 0.01 秒以便让 ROS 事件循环运行
            data = queue.get(timeout=0.01)
            msg = NavCmd()
            msg.header.stamp.sec = data['stamp_sec']
            msg.header.stamp.nanosec = data['stamp_nanosec']
            msg.header.frame_id = data['frame_id']
            msg.data.x_vel = data['x_vel']
            msg.data.y_vel = data['y_vel']
            msg.data.yaw_vel = data['yaw_vel']
            pub.publish(msg)
        except Exception:
            # 队列为空或超时，属于正常情况
            pass
        rclpy.spin_once(node, timeout_sec=0)
        
    node.destroy_node()
    rclpy.shutdown()

def main():
    queue = multiprocessing.Queue()
    
    p_sub = multiprocessing.Process(target=subscriber_domain_42, args=(queue,))
    p_pub = multiprocessing.Process(target=publisher_domain_0, args=(queue,))
    
    p_sub.start()
    p_pub.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down bridge...")
        p_sub.terminate()
        p_pub.terminate()
        p_sub.join()
        p_pub.join()

if __name__ == "__main__":
    main()
