cmake_minimum_required(VERSION 3.8)
project(localization)

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

add_compile_definitions(PCL_NO_PRECOMPILE)

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
find_package(yaml-cpp REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
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
  ${dbow3_INCLUDE_DIRS}
)

# ==========================================================
# 2. 地图发布节点
# ==========================================================
add_executable(map_publisher_cpp src/map_publisher.cpp)
ament_target_dependencies(map_publisher_cpp ${ros2_dependencies})
target_include_directories(map_publisher_cpp PUBLIC ${PCL_INCLUDE_DIRS})
target_include_directories(map_publisher_cpp PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(map_publisher_cpp ${PCL_LIBRARIES} -Wl,--no-as-needed ${LIBUSB_LIBRARIES} -Wl,--as-needed)

# ==========================================================
# 3. TF 融合节点 (高频里程计 + 低频重定位)
# ==========================================================
add_executable(transform_fusion_cpp src/transform_fusion.cpp)
ament_target_dependencies(transform_fusion_cpp ${ros2_dependencies})
target_include_directories(transform_fusion_cpp PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(transform_fusion_cpp -Wl,--no-as-needed ${LIBUSB_LIBRARIES} -Wl,--as-needed)

# ==========================================================
# 4. ICP 实时重定位节点
# ==========================================================
add_executable(global_localization_cpp src/global_localization.cpp)
ament_target_dependencies(global_localization_cpp ${ros2_dependencies} fast_gicp)
target_include_directories(global_localization_cpp PUBLIC ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_include_directories(global_localization_cpp PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_compile_options(global_localization_cpp PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(global_localization_cpp ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Eigen3::Eigen yaml-cpp ${OpenMP_CXX_FLAGS} -Wl,--no-as-needed ${LIBUSB_LIBRARIES} -Wl,--as-needed)

# ==========================================================
# 5. 基于 SC 的自动初始化节点 (完美替代 publish_initial_pose)
# ==========================================================
add_executable(lidar_global_locator src/lidar_global_locator.cpp)
target_include_directories(lidar_global_locator PUBLIC ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_include_directories(lidar_global_locator PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(lidar_global_locator ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} -Wl,--no-as-needed ${LIBUSB_LIBRARIES} -Wl,--as-needed)
ament_target_dependencies(lidar_global_locator ${ros2_dependencies} scancontext)

# ==========================================================
# 6. 基于视觉的自动初始化节点 (新增)
# ==========================================================
add_executable(visual_global_locator src/visual_global_locator.cpp)
target_include_directories(visual_global_locator PUBLIC ${OpenCV_INCLUDE_DIRS} ${dbow3_INCLUDE_DIRS})
target_include_directories(visual_global_locator PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(visual_global_locator ${OpenCV_LIBRARIES} ${dbow3_LIBRARIES} -Wl,--no-as-needed ${LIBUSB_LIBRARIES} -Wl,--as-needed)
ament_target_dependencies(visual_global_locator ${ros2_dependencies})

# ==========================================================
# 7. 平面里程计与 TF 投影桥接节点 (2D 定位降维)
# ==========================================================
add_executable(odom_bridge_node src/odom_bridge.cpp)
ament_target_dependencies(odom_bridge_node ${ros2_dependencies})

# ============== 安装 ==============
install(TARGETS 
  map_publisher_cpp 
  transform_fusion_cpp 
  global_localization_cpp
  lidar_global_locator
  visual_global_locator
  odom_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz_cfg DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# ============== ament 包导出 ==============
ament_package()
