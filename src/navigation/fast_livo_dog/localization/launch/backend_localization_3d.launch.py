import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'localization'
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
    
    map_save_dir_arg = DeclareLaunchArgument(
        'map_save_dir', 
        default_value=pgo_log_dir, 
        description='Directory containing map and SC database'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) clock if true'
    )
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='是否启动 RViz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. 地图发布节点 (加载 PCD 地图发给 RViz 和 ICP)
    map_publisher_node = Node(
        package=pkg_name,
        executable='map_publisher_cpp',
        name='map_publisher',
        output='screen',
        parameters=[{
            'map_path': [LaunchConfiguration('map_save_dir'), 'map_3d.pcd'], 
            'frame_id': 'map',
            'use_sim_time': use_sim_time
        }]
    )

    # Load configuration file path for parameter sharing
    config_file_path = os.path.join(nav_root, 'config', 'device_parameters.yaml')
    if not os.path.exists(config_file_path):
        config_file_path = os.path.join(ws_root, 'src', 'navigation', 'fast_livo_dog', 'config', 'device_parameters.yaml')

    # 2. SC 全局唤醒节点
    lidar_locator_node = Node(
        package=pkg_name,
        executable='lidar_global_locator',
        name='lidar_global_locator',
        output='screen',
        parameters=[config_file_path, {
            'sc_db_path': [LaunchConfiguration('map_save_dir'), 'sc_database.txt'],
            'match_threshold': 0.15, # SC 查表相似度阈值（极其收紧以确保首帧初筛的绝对正确性）
            'use_sim_time': use_sim_time
        }]
    )

    # 3. 视觉全局唤醒节点
    visual_locator_node = Node(
        package=pkg_name,
        executable='visual_global_locator',
        name='visual_global_locator',
        output='screen',
        parameters=[config_file_path, {
            'visual_db_path': [LaunchConfiguration('map_save_dir'), 'visual_descriptors.bin'],
            'pose_db_path': [LaunchConfiguration('map_save_dir'), 'sc_database.txt'],
            'min_good_matches': 20,
            'use_sim_time': use_sim_time
        }]
    )



    # 4. ICP 实时重定位节点
    global_loc_node = Node(
        package=pkg_name,
        executable='global_localization_cpp',
        name='global_localization',
        output='screen',
        parameters=[config_file_path, {
            'use_sim_time': use_sim_time,
            'aruco.landmarks_file': [LaunchConfiguration('map_save_dir'), 'aruco_landmarks.yaml'],
        }]
    )

    # 5. TF 坐标融合节点 (高频里程计 + 低频重定位补偿)
    transform_fusion_node = Node(
        package=pkg_name,
        executable='transform_fusion_cpp',
        name='transform_fusion',
        output='screen',
        remappings=[('/global_path', '/localization/trajectory')],
        parameters=[config_file_path, {'use_sim_time': use_sim_time}]
    )

    # 6. RViz (使用 3D 定位专用的 localization_3d.rviz)
    rvz_config_file = os.path.join(get_package_share_directory(pkg_name), 'rviz_cfg', 'localization_3d.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rvz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=use_sim_time),
        map_save_dir_arg,
        use_sim_time_arg,
        rviz_arg,
        map_publisher_node,
        lidar_locator_node,
        visual_locator_node,
        global_loc_node,
        transform_fusion_node,
        rviz_node 
    ])
