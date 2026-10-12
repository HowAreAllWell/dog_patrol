cmake_minimum_required(VERSION 3.10)
project(dog_patrol_reid_provider)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dog_patrol_interfaces REQUIRED)
find_package(dog_patrol_perception_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core dnn imgproc imgcodecs)

# TensorRT 后端（可选）：找到 libnvinfer + CUDA 时编入 feature_extractor_trt.cpp，
# 节点参数 device:=tensorrt + model_path:=*.engine 启用；找不到则只编 OpenCV DNN 后端。
option(REID_BUILD_TENSORRT "Build the TensorRT feature-extractor backend" ON)
set(REID_TRT_SOURCES "")
set(REID_TRT_DEFINITIONS "")
set(REID_TRT_LIBS "")
if(REID_BUILD_TENSORRT)
  find_path(NVINFER_INCLUDE_DIR NvInfer.h
    PATHS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu
          /usr/include /usr/local/include)
  find_library(NVINFER_LIBRARY NAMES nvinfer
    PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib)
  find_package(CUDAToolkit QUIET)
  if(NVINFER_INCLUDE_DIR AND NVINFER_LIBRARY AND CUDAToolkit_FOUND)
    set(REID_TRT_SOURCES src/feature_extractor_trt.cpp)
    set(REID_TRT_DEFINITIONS REID_HAS_TENSORRT)
    set(REID_TRT_LIBS ${NVINFER_LIBRARY} CUDA::cudart)
    set(REID_TRT_INCLUDES ${NVINFER_INCLUDE_DIR})
    message(STATUS "TensorRT backend enabled: ${NVINFER_LIBRARY}")
  else()
    message(STATUS "TensorRT 未找到（NvInfer.h/libnvinfer/CUDA），仅构建 OpenCV DNN 后端")
  endif()
endif()

# 本包接口：检测节点问询 service + 云端请求/响应/库扇出
# （请求/响应为负责人文档 v1.1 逐字定义，ReIDResponse 按字段表补上了 msg 定义
#   遗漏的 match_type；ReIDLibraryUpdate 为我们的扇出扩展接口）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Match.msg"
  "msg/ReIDRequest.msg"
  "msg/ReIDResponse.msg"
  "msg/ReIDLibraryUpdate.msg"
  "msg/ReIDHandled.msg"
  "srv/ReIDDedup.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# ---- 端侧节点 ----
add_executable(reid_node
  src/reid_node.cpp
  src/reid_gallery.cpp
  src/feature_extractor.cpp
  ${REID_TRT_SOURCES}
)
target_include_directories(reid_node PRIVATE include ${REID_TRT_INCLUDES})
target_compile_definitions(reid_node PRIVATE ${REID_TRT_DEFINITIONS})
ament_target_dependencies(reid_node
  rclcpp
  std_msgs
  sensor_msgs
  dog_patrol_interfaces
  dog_patrol_perception_interfaces
)
target_link_libraries(reid_node ${OpenCV_LIBS} ${REID_TRT_LIBS})

# 云端请求代理 + 全局库节点（不依赖 OpenCV）
add_executable(reid_cloud_proxy
  src/reid_cloud_proxy_node.cpp
  src/cloud_library.cpp
  src/reid_gallery.cpp
)
target_include_directories(reid_cloud_proxy PRIVATE include)
ament_target_dependencies(reid_cloud_proxy
  rclcpp
  std_msgs
)

# 同包使用自己生成的接口（Humble 推荐写法）
rosidl_get_typesupport_target(reid_typesupport_c ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(reid_node "${reid_typesupport_c}")
target_link_libraries(reid_cloud_proxy "${reid_typesupport_c}")

# 轻量 C++ 联调客户端（不依赖 rosidl python 绑定，现场调试用）
add_executable(reid_query
  src/reid_query.cpp
)
ament_target_dependencies(reid_query
  rclcpp
  sensor_msgs
)
target_link_libraries(reid_query "${reid_typesupport_c}" ${OpenCV_LIBS})

install(TARGETS reid_node reid_cloud_proxy reid_query
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
# 检测节点对接文档与运维脚本（sim 客户端可直接 ros2 run）
install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_detection_client.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/build_trt_engine.sh scripts/validate_trt_fp16.py
        scripts/benchmark_onnx.py
  DESTINATION share/${PROJECT_NAME}/scripts)

# ---- 独立逻辑自测（不依赖 ROS/OpenCV，g++ 即可编译运行）----
add_executable(test_reid_gallery
  test/test_gallery_main.cpp
  src/reid_gallery.cpp
  src/cloud_library.cpp
)
target_include_directories(test_reid_gallery PRIVATE include)
install(TARGETS test_reid_gallery
  DESTINATION lib/${PROJECT_NAME})

# ---- 端到端逻辑仿真（真实库代码 + 镜像节点编排，多狗/云端/扇出/重启/跨天剧本）----
add_executable(sim_reid_e2e
  test/sim_e2e_main.cpp
  src/reid_gallery.cpp
  src/cloud_library.cpp
)
target_include_directories(sim_reid_e2e PRIVATE include)
install(TARGETS sim_reid_e2e
  DESTINATION lib/${PROJECT_NAME})

# 注册进 CTest：colcon test / ctest 自动执行，失败体现在 test-result 里
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(REID_TEST_PYTHON "${Python3_EXECUTABLE}" CACHE FILEPATH "Python interpreter for ReID ROS node tests")
  ament_add_pytest_test(reid_readiness_node test/test_readiness_node.py
    PYTHON_EXECUTABLE "${REID_TEST_PYTHON}" TIMEOUT 90)
  add_executable(test_reid_readiness test/test_readiness_main.cpp)
  target_include_directories(test_reid_readiness PRIVATE include)
  add_test(NAME reid_readiness COMMAND test_reid_readiness)
  add_test(NAME reid_gallery COMMAND test_reid_gallery)
  add_test(NAME sim_reid_e2e COMMAND sim_reid_e2e)
endif()

# 接口运行时依赖导出（含 Python 绑定等默认生成物；package.xml 的
# exec_depend 不等于 CMake 侧已加载 rosidl_default_generators）
ament_export_dependencies(rosidl_default_runtime)

ament_package()
