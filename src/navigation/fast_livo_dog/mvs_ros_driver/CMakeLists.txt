cmake_minimum_required(VERSION 3.8)
project(mvs_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  link_directories(/opt/MVS/lib/aarch64/)
  message(STATUS "Running on aarch64")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  link_directories(/opt/MVS/lib/64/)
  message(STATUS "Running on x86_64")
endif()

include_directories(
  /opt/MVS/include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(grabImgWithTrigger src/grab_trigger.cpp)

ament_target_dependencies(grabImgWithTrigger
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(grabImgWithTrigger
  ${OpenCV_LIBS}
  MvCameraControl
)

# 安装可执行文件
install(TARGETS
  grabImgWithTrigger
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 和 config 文件夹（如果你后续需要用到）
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)

ament_package()