import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

import numpy as np
import math

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        self.get_logger().debug("Global Planner Node Started")

        self.declare_parameter('plan_period', 2.0)
        self.plan_period = max(0.5, float(self.get_parameter('plan_period').value))
        self.request_in_flight = False

        # 创建动作客户端
        self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.publisher_ = self.create_publisher(Path, 'global_path', 20)

        # 订阅 goal_pose 话题，动态获取用户设定的目标点
        self.goal_xy = None
        self.goal_yaw = 0.0
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_pose_callback, 10
        )
        self.timer = self.create_timer(self.plan_period, self.plan_and_publish_path)

    def goal_pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        self.goal_xy = np.array([x, y])
        self.goal_yaw = yaw
        self.get_logger().info(f"Global Planner received goal: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f}°")

    def plan_and_publish_path(self):
        if self.goal_xy is None:
            return
        if self.request_in_flight:
            return
        if not self.compute_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().debug("ComputePathToPose server not available yet.")
            return

        goal_msg = ComputePathToPose.Goal()
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = float(self.goal_xy[0])
        target_pose.pose.position.y = float(self.goal_xy[1])
        target_pose.pose.orientation.z = math.sin(self.goal_yaw * 0.5)
        target_pose.pose.orientation.w = math.cos(self.goal_yaw * 0.5)

        goal_msg.goal = target_pose
        goal_msg.use_start = False  # 使用导航栈提供的当前位置

        self.get_logger().debug("Sending global path planning request...")

        self.request_in_flight = True
        send_goal_future = self.compute_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.request_in_flight = False
            self.get_logger().error(f'Goal request failed: {exc}')
            return
        if not goal_handle.accepted:
            self.request_in_flight = False
            self.get_logger().error('Goal rejected by server')
            return

        self.get_logger().debug('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.request_in_flight = False
        result = future.result().result
        if result is None:
            self.get_logger().error('No path returned')
            return

        path_msg = result.path
        path_msg.header.frame_id = "map"
        self.get_logger().debug("publish global path")
        self.publisher_.publish(path_msg)
        self.get_logger().debug(f"Published global path with {len(path_msg.poses)} poses")

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
