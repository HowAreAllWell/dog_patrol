import sys
import os
import importlib.util
import subprocess

# Keep ROS Python nodes and colcon on the same runtime as the deployed
# perception dependencies. This is explicit because a previously generated
# ROS console script may still contain /usr/bin/python3 in its shebang.
preferred_venv = os.environ.get("DOG_PATROL_VENV", "/mnt/nvme/venv/m20_nav")
preferred_python = os.path.join(preferred_venv, "bin", "python3")
if os.path.isfile(preferred_python):
    os.environ["DOG_PATROL_VENV"] = preferred_venv
    os.environ["DOG_PATROL_PYTHON"] = preferred_python
    os.environ["VIRTUAL_ENV"] = preferred_venv
    os.environ["PATH"] = (
        f"{os.path.dirname(preferred_python)}:{os.environ.get('PATH', '')}"
    )

# 强制选用 FastDDS 作为通信中间件，以利用 C++ 节点之间高带宽的高速共享内存 (SHM) 通信
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

# Tracking in the merged system consumes the navigation camera image topic,
# while the deployed perception binary still links the Hik SDK at runtime.
def _find_workspace_root():
    configured_root = os.environ.get("DOG_PATROL_WORKSPACE_ROOT", "").strip()
    candidates = []
    if configured_root:
        candidates.append(os.path.abspath(configured_root))

    current = os.path.abspath(os.path.dirname(__file__))
    while current and current != os.path.dirname(current):
        candidates.append(current)
        current = os.path.dirname(current)

    for candidate in candidates:
        package_manifest = os.path.join(
            candidate,
            "src",
            "orchestration",
            "robot_console",
            "package.xml",
        )
        if os.path.isfile(package_manifest):
            return candidate

    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))


app_workspace_root = _find_workspace_root()
os.environ.setdefault("DOG_PATROL_WORKSPACE_ROOT", app_workspace_root)


def _bootstrap_workspace_environment():
    """Load this workspace when app.py is executed directly from source."""
    if importlib.util.find_spec("dog_patrol_interfaces") is not None:
        return

    setup_path = os.path.join(app_workspace_root, "install", "setup.bash")
    bootstrap_marker = "DOG_PATROL_WORKSPACE_BOOTSTRAPPED"
    if os.environ.get(bootstrap_marker) == app_workspace_root:
        raise ModuleNotFoundError(
            "dog_patrol_interfaces is still unavailable after loading "
            f"{setup_path}; rebuild dog_patrol_interfaces and robot_console"
        )
    if not os.path.isfile(setup_path):
        raise ModuleNotFoundError(
            "dog_patrol_interfaces is unavailable and this workspace has no "
            f"{setup_path}; run colcon build before starting robot_console"
        )

    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    ros_setup_path = f"/opt/ros/{ros_distro}/setup.bash"
    if not os.path.isfile(ros_setup_path):
        raise ModuleNotFoundError(
            "dog_patrol_interfaces is unavailable and the ROS setup file does "
            f"not exist: {ros_setup_path}"
        )

    result = subprocess.run(
        [
            "bash",
            "-c",
            'source "$1" && source "$2" && env -0',
            "dog-patrol-console-bootstrap",
            ros_setup_path,
            setup_path,
        ],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        diagnostic = result.stderr.decode(errors="replace").strip()
        raise RuntimeError(
            f"failed to load {setup_path}: {diagnostic or 'unknown setup error'}"
        )

    environment = os.environ.copy()
    for entry in result.stdout.split(b"\0"):
        if not entry or b"=" not in entry:
            continue
        key, value = entry.split(b"=", 1)
        environment[os.fsdecode(key)] = os.fsdecode(value)
    environment[bootstrap_marker] = app_workspace_root

    # Re-exec is required so the dynamic loader also sees the interface package
    # library path produced by the sourced colcon environment.
    os.execve(
        sys.executable,
        [sys.executable, os.path.abspath(__file__), *sys.argv[1:]],
        environment,
    )


_bootstrap_workspace_environment()

os.environ.setdefault(
    "DOG_PATROL_ASSETS_ROOT",
    os.path.join(
        app_workspace_root,
        "src",
        "perception",
        "dog_patrol_perception_assets_20260813",
    ),
)
mvs_lib = "/opt/MVS/lib/aarch64"
ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
if mvs_lib not in ld_library_path.split(":"):
    os.environ["LD_LIBRARY_PATH"] = (
        f"{mvs_lib}:{ld_library_path}" if ld_library_path else mvs_lib
    )

# 强制使用独立的 ROS_DOMAIN_ID 进行通信隔离，防止与机器狗底层内部节点或局域网内其他设备产生 /tf 冲突
domain_id = "42"
try:
    import yaml
    current_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.abspath(os.path.join(current_dir, "../../../.."))
    nav_root = os.environ.get(
        "DOG_PATROL_NAV_ROOT",
        os.path.join(workspace_root, "src", "navigation", "fast_livo_dog"),
    )
    yaml_path = os.path.join(nav_root, "config", "device_parameters.yaml")
    if os.path.exists(yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            config_data = yaml.safe_load(f)
            if config_data and "/**" in config_data:
                params = config_data["/**"].get("ros__parameters", {})
                if "ros_domain_id" in params:
                    domain_id = str(params["ros_domain_id"])
except Exception as e:
    print(f"Failed to read ros_domain_id from config, using default 42: {e}")

os.environ["ROS_DOMAIN_ID"] = domain_id

# 为 PyQt UI 进程指定专属的 FastDDS 配置，强制其仅使用 UDPv4 传输，从而避开 ARM64 架构下 Python C-bindings 订阅 C++ 共享内存的 Bug
current_dir = os.path.dirname(os.path.abspath(__file__))
nav_root = os.environ.get(
    "DOG_PATROL_NAV_ROOT",
    os.path.abspath(
        os.path.join(current_dir, "../../../..", "src", "navigation", "fast_livo_dog")
    ),
)
config_path = os.path.join(nav_root, "config", "fastdds_udp_only.xml")
if os.path.exists(config_path):
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = config_path


from PyQt5.QtWidgets import QApplication

# 导入界面与后台解耦模块
# pyrefly: ignore [missing-import]
try:
    from .main_window import RobotMainWindow
    from .ros2_node import Ros2BackendThread
except ImportError:
    from main_window import RobotMainWindow
    from ros2_node import Ros2BackendThread

def main():
    # 1. 启动 Qt 应用
    from PyQt5.QtCore import Qt
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    
    # 2. 实例化界面 (View)
    window = RobotMainWindow()
    
    # 3. 实例化并启动后台 ROS2 通信线程 (Model)
    ros_thread = Ros2BackendThread()
    
    # 4. 连接后台 ROS 2 线程与前端 GUI 实例 of 跨线程信号槽
    ros_thread.signals.pose_updated.connect(window.update_hud)
    ros_thread.signals.log_msg.connect(window.append_log)
    
    # 连接定位状态更新信号，确保地图撤销和显示逻辑同步
    ros_thread.signals.status_updated.connect(window.handle_localization_status)
    ros_thread.signals.odom_distance_updated.connect(window.update_odom_distance)
    ros_thread.signals.ui_clear.connect(window.ui_clear_callback)
    ros_thread.signals.task_state_updated.connect(window.update_task_state)
    ros_thread.signals.task_command_result.connect(window.handle_task_command_result)
    
    # 重置槽连接：当前端触发重置信号时，通知后台清空历史 TF 缓存
    window.sig_reset_tf.connect(ros_thread.reset_tf)
    # 停止导航槽连接：通知后台发送空 /global_path 消息清除 RViz 全局路径残影
    window.sig_clear_nav.connect(ros_thread.clear_nav_viz)
    window.sig_task_command.connect(ros_thread.send_task_command)
    
    # 启动后台监听线程
    ros_thread.start()
    
    # 显示主界面
    window.show()
    
    # 5. 挂起主循环，程序结束时安全清理
    exit_code = app.exec_()
    
    # 关闭时挂起并回收后台线程
    ros_thread.stop()
    sys.exit(exit_code)

if __name__ == '__main__':
    if "--check-environment" in sys.argv[1:]:
        from dog_patrol_interfaces.msg import PatrolTaskState
        from dog_patrol_interfaces.srv import PatrolTaskCommand

        print(
            "robot_console environment OK: "
            f"TASK_RUNNING={PatrolTaskState.TASK_RUNNING}, "
            f"START={PatrolTaskCommand.Request.START}"
        )
    else:
        main()
