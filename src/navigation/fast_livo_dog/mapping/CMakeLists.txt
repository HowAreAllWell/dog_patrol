cmake_minimum_required(VERSION 3.8)
project(mapping)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if($ENV{ROS_DISTRO} MATCHES "humble")
  add_compile_definitions(ROS_HUMBLE)
elseif($ENV{ROS_DISTRO} MATCHES "foxy")
  add_compile_definitions(ROS_FOXY)
endif()
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============== 查找依赖 ==============
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(fast_gicp REQUIRED)
find_package(scancontext REQUIRED)
find_package(dbow3 REQUIRED)

set(ros2_dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  cv_bridge
)

# ============== 头文件路径 ==============
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${dbow3_INCLUDE_DIRS}
)

# ==========================================================
# 1. 核心雷达闭环与后端建图
# ==========================================================
add_executable(lidar_loop_node 
  src/lidar_loop_node.cpp
  src/fg_manager.cpp
  src/loop_detector.cpp
  src/map_paving_engine.cpp
)
target_compile_options(lidar_loop_node PRIVATE ${OpenMP_CXX_FLAGS})
target_include_directories(lidar_loop_node PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(lidar_loop_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam -Wl,--no-as-needed ${LIBUSB_LIBRARIES} -Wl,--as-needed)
ament_target_dependencies(lidar_loop_node ${ros2_dependencies} scancontext fast_gicp)

# ==========================================================
# 2. 视觉回环检测节点
# ==========================================================
add_executable(visual_loop_node src/visual_loop_node.cpp)
target_link_libraries(visual_loop_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${dbow3_LIBRARIES})
ament_target_dependencies(visual_loop_node ${ros2_dependencies})

# ============== 安装 ==============
install(TARGETS 
  lidar_loop_node
  visual_loop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz_cfg DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# ============== ament 包导出 ==============
ament_package()
