from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'mapping'
    pkg_share = get_package_share_directory(pkg_name)
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    nav_root = os.environ.get(
        'DOG_PATROL_NAV_ROOT', os.path.join(ws_root, 'src', 'navigation', 'fast_livo_dog'))
    pgo_log_dir = os.path.join(nav_root, 'data') + '/'
    if not os.path.exists(pgo_log_dir):
        pgo_log_dir = os.path.join(ws_root, 'src', 'data') + '/'
    
    # 预加载 libusb 动态库，避免与前端节点的 PCL 动态链接冲突
    import platform
    if platform.machine() == "aarch64":
        os.environ['LD_PRELOAD'] = '/lib/aarch64-linux-gnu/libusb-1.0.so.0'
    else:
        os.environ['LD_PRELOAD'] = '/lib/x86_64-linux-gnu/libusb-1.0.so.0'
    
    # RViz 路径
    rviz_config_file = os.path.join(
        get_package_share_directory('mapping'), 'rviz_cfg', 'mapping.rviz')

    nav_config_file_path = os.path.join(nav_root, 'config', 'nav_parameters.yaml')
    if not os.path.exists(nav_config_file_path):
        nav_config_file_path = os.path.join(ws_root, 'src', 'navigation', 'fast_livo_dog', 'config', 'nav_parameters.yaml')

    device_config_file_path = os.path.join(nav_root, 'config', 'device_parameters.yaml')
    if not os.path.exists(device_config_file_path):
        device_config_file_path = os.path.join(ws_root, 'src', 'navigation', 'fast_livo_dog', 'config', 'device_parameters.yaml')

    map_save_dir_arg = DeclareLaunchArgument(
        'map_save_dir', 
        default_value=pgo_log_dir, 
        description='Directory to save maps'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='是否启动 RViz')

    return LaunchDescription([
        map_save_dir_arg,
        use_sim_time_arg,
        rviz_arg,
        Node(
            package='mapping',
            executable='lidar_loop_node',
            name='lidar_loop_node',
            output='screen',
            parameters=[
                nav_config_file_path,
                {'use_sim_time': use_sim_time},
                {'save_directory': LaunchConfiguration('map_save_dir')},
                {'keyframe_meter_gap': 0.5},        # 调密关键帧，增加回环机会
                {'keyframe_deg_gap': 5.0},
                {'sc_dist_thres': 0.15},             # 雷达回环灵敏度
                {'loopFitnessScoreThreshold': 0.20}, # 收紧 ICP 验证严苛度至 0.20，平衡回环数量与配准质量
                {'mapviz_filter_size': 0.1}         # 3D PCD 地图保存时的下采样分辨率
            ]
        ),
        Node(
            package='mapping',
            executable='visual_loop_node',
            name='visual_loop_node',
            output='screen',
            parameters=[
                device_config_file_path,
                {'use_sim_time': use_sim_time},
                {'save_directory': LaunchConfiguration('map_save_dir')},
                {'orb_nfeatures': 500},             # 视觉特征点数量
                {'clahe_clip_limit': 3.0},          # 图像增强强度
                {'min_loop_id_diff': 30},           # 两边统一为 30 帧
                {'lowe_ratio': 0.75},               # 匹配点筛选比例
                {'min_good_matches': 30}            # 视觉回环触发阈值
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ])
