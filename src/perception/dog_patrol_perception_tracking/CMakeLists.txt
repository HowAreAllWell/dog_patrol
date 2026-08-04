cmake_minimum_required(VERSION 3.8)
project(dog_patrol_perception_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dog_patrol_interfaces REQUIRED)
find_package(dog_patrol_perception_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
option(
  TRACKING_ENABLE_ORIN_RUNTIME
  "Build the CUDA/TensorRT/Hik MVS runtime and FFmpeg recording/offline tools"
  ON)

set(CORE_SOURCES
  src/modules/yolo26_output_contract.cpp
  src/modules/det_filter.cpp
  src/modules/appearance_feature_service.cpp
  src/modules/reid_embedder.cpp
  src/modules/association_utils.cpp
  src/modules/mot_tracker_observability.cpp
  src/modules/mot_tracker.cpp
  src/modules/primary_target_manager.cpp
  src/modules/primary_target_observer.cpp
  src/modules/mission_coordinator.cpp
  src/modules/mission_frame_transaction.cpp
  src/modules/perception_config_materializer.cpp
  src/modules/perception_readiness.cpp
  src/modules/primary_recovery_debug.cpp
  src/modules/feature_update_policy.cpp
  src/modules/feature_bank_cost.cpp
  src/modules/feature_geometry_update_state.cpp
  src/modules/reliable_geometry_cost.cpp
  src/modules/assignment_cost.cpp
  src/modules/active_assignment_solver.cpp
  src/modules/inactive_recovery_solver.cpp
  src/modules/raw_continuity_decision.cpp
  src/modules/birth_candidate_decision.cpp
  src/modules/birth_candidate_store.cpp
  src/modules/birth_manager.cpp
  src/modules/phase5_birth_coordinator.cpp
  src/modules/phase4_handoff_coordinator.cpp
  src/modules/merged_single_blob_assignment_decision.cpp
  src/modules/occlusion_mode_state.cpp
  src/modules/occlusion_group_shadow_lifecycle.cpp
  src/modules/semantic_id_allocator.cpp
  src/modules/assignment_application_plan.cpp
  src/modules/assignment_application_executor.cpp
  src/modules/assignment_candidate_builder.cpp
  src/modules/active_assignment_input_collector.cpp
  src/modules/inactive_recovery_input_collector.cpp
  src/modules/unresolved_track_final_resolution_coordinator.cpp
  src/modules/raw_semantic_binding_store.cpp
  src/modules/identity_runtime_store.cpp
  src/modules/identity_runtime_lifecycle_coordinator.cpp
  src/modules/identity_runtime_mutation_applier.cpp
  src/modules/identity_assignment_engine_adapter.cpp
  src/modules/identity_assignment_frame_transaction.cpp
  src/modules/identity_observation_projection.cpp
  src/modules/identity_manager.cpp
  src/modules/runtime_monitor.cpp
  src/tools/offline_eval_input.cpp
  src/tools/identity_offline_metrics.cpp
)

add_library(${PROJECT_NAME}_core ${CORE_SOURCES})
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_core PRIVATE src/modules)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_core PUBLIC ${OpenCV_LIBS})

add_library(${PROJECT_NAME}_ros_adapter
  src/modules/mission_ros_adapter.cpp
  src/modules/target_image_ros_adapter.cpp)
target_include_directories(${PROJECT_NAME}_ros_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_ros_adapter PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_ros_adapter
  rclcpp dog_patrol_interfaces dog_patrol_perception_interfaces)
target_link_libraries(${PROJECT_NAME}_ros_adapter ${PROJECT_NAME}_core)

# Deterministic, headless transport fixture for the issue #84 deployment
# smoke. It is intentionally separate from the gtest process: a ROS CLI
# publisher/subscriber drives the same adapter boundary over DDS.
add_executable(mission_ros_adapter_smoke src/tools/mission_ros_adapter_smoke.cpp)
target_compile_features(mission_ros_adapter_smoke PUBLIC cxx_std_17)
ament_target_dependencies(mission_ros_adapter_smoke
  rclcpp dog_patrol_interfaces dog_patrol_perception_interfaces)
target_link_libraries(mission_ros_adapter_smoke ${PROJECT_NAME}_ros_adapter)

# Deployment preflight uses the same CMake-resolved OpenCV DNN runtime as the
# tracking core to validate ReID ONNX files and any referenced external data.
add_executable(validate_reid_onnx src/tools/validate_reid_onnx.cpp)
target_compile_features(validate_reid_onnx PUBLIC cxx_std_17)
target_include_directories(validate_reid_onnx PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(validate_reid_onnx ${OpenCV_LIBS})

set(INSTALL_TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_ros_adapter
  mission_ros_adapter_smoke
  validate_reid_onnx)

if(TRACKING_ENABLE_ORIN_RUNTIME)
  message(STATUS "Building the full Orin tracking runtime")
  find_package(CUDAToolkit QUIET)
  if(NOT CUDAToolkit_FOUND)
    message(FATAL_ERROR
      "TRACKING_ENABLE_ORIN_RUNTIME=ON requires the CUDA toolkit development files. "
      "Install the CUDA toolkit for the target JetPack, set CUDAToolkit_ROOT if it is "
      "installed in a non-standard location, or configure with "
      "-DTRACKING_ENABLE_ORIN_RUNTIME=OFF for the portable core build.")
  endif()

  find_package(PkgConfig QUIET)
  if(NOT PkgConfig_FOUND)
    message(FATAL_ERROR
      "TRACKING_ENABLE_ORIN_RUNTIME=ON requires pkg-config to locate FFmpeg. "
      "Install pkg-config and the FFmpeg development packages, or configure with "
      "-DTRACKING_ENABLE_ORIN_RUNTIME=OFF.")
  endif()
  pkg_check_modules(FFMPEG QUIET IMPORTED_TARGET libavcodec libavformat libavutil)
  if(NOT FFMPEG_FOUND)
    message(FATAL_ERROR
      "TRACKING_ENABLE_ORIN_RUNTIME=ON requires FFmpeg development packages "
      "(libavcodec, libavformat, and libavutil). Install their development packages, "
      "or configure with -DTRACKING_ENABLE_ORIN_RUNTIME=OFF.")
  endif()

  find_path(TENSORRT_INCLUDE_DIR NvInfer.h)
  find_library(TENSORRT_LIB nvinfer)
  if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIB)
    message(FATAL_ERROR
      "TRACKING_ENABLE_ORIN_RUNTIME=ON requires TensorRT headers and libnvinfer. "
      "Install the TensorRT development package for the target JetPack, or configure "
      "with -DTRACKING_ENABLE_ORIN_RUNTIME=OFF for the portable core build.")
  endif()

  find_path(MVS_INCLUDE_DIR MvCameraControl.h PATHS /opt/MVS/include)
  find_library(MVS_LIB MvCameraControl PATHS /opt/MVS/lib/64 /opt/MVS/lib/aarch64)
  if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIB)
    message(FATAL_ERROR
      "TRACKING_ENABLE_ORIN_RUNTIME=ON requires the Hikrobot MVS SDK. "
      "Install it under /opt/MVS (headers in include and libMvCameraControl in lib/64 "
      "or lib/aarch64), or configure with -DTRACKING_ENABLE_ORIN_RUNTIME=OFF.")
  endif()
  get_filename_component(MVS_LIB_DIR "${MVS_LIB}" DIRECTORY)

  add_library(${PROJECT_NAME}_orin_runtime
    src/modules/camera_ingest.cpp
    src/modules/preprocess_infer.cpp)
  target_include_directories(${PROJECT_NAME}_orin_runtime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${TENSORRT_INCLUDE_DIR})
  target_include_directories(${PROJECT_NAME}_orin_runtime PRIVATE ${MVS_INCLUDE_DIR})
  target_compile_features(${PROJECT_NAME}_orin_runtime PUBLIC cxx_std_17)
  target_compile_definitions(${PROJECT_NAME}_orin_runtime PRIVATE DOG_PATROL_PERCEPTION_TRACKING_ENABLE_HIK_MVS=1)
  target_link_libraries(${PROJECT_NAME}_orin_runtime PUBLIC
    ${PROJECT_NAME}_core ${TENSORRT_LIB} CUDA::cudart ${MVS_LIB})
  set_property(TARGET ${PROJECT_NAME}_orin_runtime APPEND PROPERTY BUILD_RPATH "${MVS_LIB_DIR}")
  set_property(TARGET ${PROJECT_NAME}_orin_runtime APPEND PROPERTY INSTALL_RPATH "${MVS_LIB_DIR}")

  add_library(${PROJECT_NAME}_recording
    src/modules/ffv1_mkv_writer.cpp
    src/modules/ffv1_capture_workflow.cpp
    src/modules/ffv1_capture_artifact_writer.cpp
    src/modules/visualizer_recorder.cpp)
  target_include_directories(${PROJECT_NAME}_recording PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${PROJECT_NAME}_recording PUBLIC cxx_std_17)
  target_link_libraries(${PROJECT_NAME}_recording PUBLIC
    ${PROJECT_NAME}_core ${PROJECT_NAME}_orin_runtime PkgConfig::FFMPEG)

  add_library(${PROJECT_NAME}_offline_runtime
    src/tools/offline_replay_run.cpp)
  target_include_directories(${PROJECT_NAME}_offline_runtime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${PROJECT_NAME}_offline_runtime PUBLIC cxx_std_17)
  target_link_libraries(${PROJECT_NAME}_offline_runtime PUBLIC
    ${PROJECT_NAME}_core ${PROJECT_NAME}_orin_runtime)

  add_executable(dog_patrol_perception_tracking_node src/perception_tracking_node.cpp)
  target_compile_features(dog_patrol_perception_tracking_node PUBLIC cxx_std_17)
  ament_target_dependencies(dog_patrol_perception_tracking_node
    rclcpp dog_patrol_interfaces dog_patrol_perception_interfaces)
  target_link_libraries(dog_patrol_perception_tracking_node
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_ros_adapter
    ${PROJECT_NAME}_orin_runtime
    ${PROJECT_NAME}_recording)

  add_executable(capture_ffv1 src/tools/capture_ffv1.cpp)
  target_compile_features(capture_ffv1 PUBLIC cxx_std_17)
  target_link_libraries(capture_ffv1 ${PROJECT_NAME}_recording)

  add_executable(offline_eval_recordings src/tools/offline_eval_recordings.cpp)
  target_compile_features(offline_eval_recordings PUBLIC cxx_std_17)
  target_link_libraries(offline_eval_recordings ${PROJECT_NAME}_offline_runtime)

  add_executable(benchmark_bayer_input src/tools/benchmark_bayer_input.cpp)
  target_compile_features(benchmark_bayer_input PUBLIC cxx_std_17)
  target_include_directories(benchmark_bayer_input PRIVATE ${MVS_INCLUDE_DIR})
  target_link_libraries(benchmark_bayer_input ${PROJECT_NAME}_recording)
  set_property(TARGET benchmark_bayer_input APPEND PROPERTY BUILD_RPATH "${MVS_LIB_DIR}")
  set_property(TARGET benchmark_bayer_input APPEND PROPERTY INSTALL_RPATH "${MVS_LIB_DIR}")

  list(APPEND INSTALL_TARGETS
    ${PROJECT_NAME}_orin_runtime
    ${PROJECT_NAME}_recording
    ${PROJECT_NAME}_offline_runtime
    dog_patrol_perception_tracking_node
    capture_ffv1
    offline_eval_recordings
    benchmark_bayer_input)
else()
  message(STATUS
    "Building portable tracking core and ROS adapter only "
    "(TRACKING_ENABLE_ORIN_RUNTIME=OFF)")
endif()

install(TARGETS ${INSTALL_TARGETS} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(TRACKING_ENABLE_ORIN_RUNTIME)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

ament_export_dependencies(
  dog_patrol_interfaces dog_patrol_perception_interfaces rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The portable build cannot link the live executable without Orin SDKs, but
  # it still compiles the node translation unit so mission/standalone control
  # flow remains covered by ordinary CI.
  if(NOT TRACKING_ENABLE_ORIN_RUNTIME)
    add_library(${PROJECT_NAME}_live_node_compile_check OBJECT
      src/perception_tracking_node.cpp)
    target_compile_features(${PROJECT_NAME}_live_node_compile_check PUBLIC cxx_std_17)
    ament_target_dependencies(${PROJECT_NAME}_live_node_compile_check
      rclcpp dog_patrol_interfaces dog_patrol_perception_interfaces)
    target_link_libraries(${PROJECT_NAME}_live_node_compile_check
      ${PROJECT_NAME}_core ${PROJECT_NAME}_ros_adapter)
  endif()

  if(TRACKING_ENABLE_ORIN_RUNTIME)
    add_test(
      NAME test_retired_media_cli
      COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_retired_media_cli.sh
              $<TARGET_FILE:dog_patrol_perception_tracking_node>
              $<TARGET_FILE:offline_eval_recordings>)

    ament_add_gtest(test_camera_ingest_contract test/test_camera_ingest_contract.cpp)
    if(TARGET test_camera_ingest_contract)
      target_compile_features(test_camera_ingest_contract PUBLIC cxx_std_17)
      target_include_directories(test_camera_ingest_contract PRIVATE include)
      target_link_libraries(test_camera_ingest_contract ${PROJECT_NAME}_orin_runtime)
    endif()

    ament_add_gtest(test_preprocess_infer_timing test/test_preprocess_infer_timing.cpp)
    if(TARGET test_preprocess_infer_timing)
      target_compile_features(test_preprocess_infer_timing PUBLIC cxx_std_17)
      target_include_directories(test_preprocess_infer_timing PRIVATE include)
      target_link_libraries(test_preprocess_infer_timing ${PROJECT_NAME}_orin_runtime)
    endif()

    ament_add_gtest(test_ffv1_capture_workflow test/test_ffv1_capture_workflow.cpp)
    if(TARGET test_ffv1_capture_workflow)
      target_compile_features(test_ffv1_capture_workflow PUBLIC cxx_std_17)
      target_include_directories(test_ffv1_capture_workflow PRIVATE include)
      target_link_libraries(test_ffv1_capture_workflow ${PROJECT_NAME}_recording)
    endif()

    ament_add_gtest(test_visualizer_recorder test/test_visualizer_recorder.cpp)
    if(TARGET test_visualizer_recorder)
      target_compile_features(test_visualizer_recorder PUBLIC cxx_std_17)
      target_include_directories(test_visualizer_recorder PRIVATE include)
      target_link_libraries(test_visualizer_recorder ${PROJECT_NAME}_recording)
    endif()
  endif()

  ament_add_gtest(test_primary_target_manager test/test_primary_target_manager.cpp)
  if(TARGET test_primary_target_manager)
    target_compile_features(test_primary_target_manager PUBLIC cxx_std_17)
    target_include_directories(test_primary_target_manager PRIVATE include)
    target_link_libraries(test_primary_target_manager ${PROJECT_NAME}_core)
    ament_target_dependencies(test_primary_target_manager rclcpp)
  endif()

  ament_add_gtest(test_primary_target_observer test/test_primary_target_observer.cpp)
  if(TARGET test_primary_target_observer)
    target_compile_features(test_primary_target_observer PUBLIC cxx_std_17)
    target_include_directories(test_primary_target_observer PRIVATE include)
    target_link_libraries(test_primary_target_observer ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_mission_coordinator test/test_mission_coordinator.cpp)
  if(TARGET test_mission_coordinator)
    target_compile_features(test_mission_coordinator PUBLIC cxx_std_17)
    target_include_directories(test_mission_coordinator PRIVATE include)
    target_link_libraries(test_mission_coordinator ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_mission_frame_transaction test/test_mission_frame_transaction.cpp)
  if(TARGET test_mission_frame_transaction)
    target_compile_features(test_mission_frame_transaction PUBLIC cxx_std_17)
    target_include_directories(test_mission_frame_transaction PRIVATE include)
    target_link_libraries(test_mission_frame_transaction ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_perception_readiness test/test_perception_readiness.cpp)
  if(TARGET test_perception_readiness)
    target_compile_features(test_perception_readiness PUBLIC cxx_std_17)
    target_include_directories(test_perception_readiness PRIVATE include)
    target_link_libraries(test_perception_readiness ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_perception_config_materializer test/test_perception_config_materializer.cpp)
  if(TARGET test_perception_config_materializer)
    target_compile_features(test_perception_config_materializer PUBLIC cxx_std_17)
    target_include_directories(test_perception_config_materializer PRIVATE include)
    target_link_libraries(test_perception_config_materializer ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_mission_ros_adapter test/test_mission_ros_adapter.cpp)
  if(TARGET test_mission_ros_adapter)
    target_compile_features(test_mission_ros_adapter PUBLIC cxx_std_17)
    target_include_directories(test_mission_ros_adapter PRIVATE include)
    target_link_libraries(test_mission_ros_adapter ${PROJECT_NAME}_ros_adapter)
    ament_target_dependencies(test_mission_ros_adapter
      rclcpp dog_patrol_interfaces dog_patrol_perception_interfaces)
  endif()

  ament_add_gtest(test_target_image_ros_adapter test/test_target_image_ros_adapter.cpp)
  if(TARGET test_target_image_ros_adapter)
    target_compile_features(test_target_image_ros_adapter PUBLIC cxx_std_17)
    target_include_directories(test_target_image_ros_adapter PRIVATE include)
    target_link_libraries(test_target_image_ros_adapter ${PROJECT_NAME}_ros_adapter)
    ament_target_dependencies(test_target_image_ros_adapter
      rclcpp dog_patrol_perception_interfaces)
  endif()

  add_executable(test_target_image_ros_smoke_driver
    test/target_image_ros_smoke.cpp)
  target_compile_features(test_target_image_ros_smoke_driver PUBLIC cxx_std_17)
  target_include_directories(test_target_image_ros_smoke_driver PRIVATE include)
  target_link_libraries(test_target_image_ros_smoke_driver
    ${PROJECT_NAME}_core ${PROJECT_NAME}_ros_adapter)
  ament_target_dependencies(test_target_image_ros_smoke_driver
    rclcpp dog_patrol_perception_interfaces)
  add_test(
    NAME test_target_image_ros_smoke
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_target_image_ros_smoke.sh
            $<TARGET_FILE:test_target_image_ros_smoke_driver>)
  set_tests_properties(test_target_image_ros_smoke PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=188;ROS_LOCALHOST_ONLY=1"
    TIMEOUT 20)

  add_executable(test_mission_pipeline_integration_driver
    test/mission_pipeline_integration_driver.cpp)
  target_compile_features(test_mission_pipeline_integration_driver PUBLIC cxx_std_17)
  target_include_directories(test_mission_pipeline_integration_driver PRIVATE include)
  target_link_libraries(test_mission_pipeline_integration_driver
    ${PROJECT_NAME}_core ${PROJECT_NAME}_ros_adapter)
  if(TRACKING_ENABLE_ORIN_RUNTIME)
    target_compile_definitions(test_mission_pipeline_integration_driver PRIVATE
      DOG_PATROL_PERCEPTION_TRACKING_ENABLE_ORIN_RUNTIME=1)
    target_link_libraries(test_mission_pipeline_integration_driver
      ${PROJECT_NAME}_orin_runtime)
  endif()
  ament_target_dependencies(test_mission_pipeline_integration_driver
    rclcpp dog_patrol_interfaces dog_patrol_perception_interfaces)
  add_test(
    NAME test_mission_pipeline_integration
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mission_pipeline_integration.sh
            $<TARGET_FILE:test_mission_pipeline_integration_driver>)
  set_tests_properties(test_mission_pipeline_integration PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=187;ROS_LOCALHOST_ONLY=1"
    TIMEOUT 45)

  ament_add_gtest(test_primary_recovery_debug test/test_primary_recovery_debug.cpp)
  if(TARGET test_primary_recovery_debug)
    target_compile_features(test_primary_recovery_debug PUBLIC cxx_std_17)
    target_include_directories(test_primary_recovery_debug PRIVATE include)
    target_link_libraries(test_primary_recovery_debug ${PROJECT_NAME}_core)
    ament_target_dependencies(test_primary_recovery_debug rclcpp)
  endif()

  ament_add_gtest(test_identity_assignment_engine_adapter test/test_identity_assignment_engine_adapter.cpp)
  if(TARGET test_identity_assignment_engine_adapter)
    target_compile_features(test_identity_assignment_engine_adapter PUBLIC cxx_std_17)
    target_include_directories(test_identity_assignment_engine_adapter PRIVATE include src/modules)
    target_link_libraries(test_identity_assignment_engine_adapter ${PROJECT_NAME}_core)
    ament_target_dependencies(test_identity_assignment_engine_adapter rclcpp)
  endif()

  ament_add_gtest(test_identity_assignment_frame_transaction test/test_identity_assignment_frame_transaction.cpp)
  if(TARGET test_identity_assignment_frame_transaction)
    target_compile_features(test_identity_assignment_frame_transaction PUBLIC cxx_std_17)
    target_include_directories(test_identity_assignment_frame_transaction PRIVATE include src/modules)
    target_link_libraries(test_identity_assignment_frame_transaction ${PROJECT_NAME}_core)
    ament_target_dependencies(test_identity_assignment_frame_transaction rclcpp)
  endif()

  ament_add_gtest(test_identity_manager test/test_identity_manager.cpp)
  if(TARGET test_identity_manager)
    target_compile_features(test_identity_manager PUBLIC cxx_std_17)
    target_include_directories(test_identity_manager PRIVATE include)
    target_link_libraries(test_identity_manager ${PROJECT_NAME}_core)
    ament_target_dependencies(test_identity_manager rclcpp)
  endif()

  ament_add_gtest(test_identity_observation_projection test/test_identity_observation_projection.cpp)
  if(TARGET test_identity_observation_projection)
    target_compile_features(test_identity_observation_projection PUBLIC cxx_std_17)
    target_include_directories(test_identity_observation_projection PRIVATE include src/modules)
    target_link_libraries(test_identity_observation_projection ${PROJECT_NAME}_core)
    ament_target_dependencies(test_identity_observation_projection rclcpp)
  endif()

  ament_add_gtest(test_feature_update_policy test/test_feature_update_policy.cpp)
  if(TARGET test_feature_update_policy)
    target_compile_features(test_feature_update_policy PUBLIC cxx_std_17)
    target_include_directories(test_feature_update_policy PRIVATE include)
    target_link_libraries(test_feature_update_policy ${PROJECT_NAME}_core)
    ament_target_dependencies(test_feature_update_policy rclcpp)
  endif()

  ament_add_gtest(test_feature_geometry_update_state test/test_feature_geometry_update_state.cpp)
  if(TARGET test_feature_geometry_update_state)
    target_compile_features(test_feature_geometry_update_state PUBLIC cxx_std_17)
    target_include_directories(test_feature_geometry_update_state PRIVATE include)
    target_link_libraries(test_feature_geometry_update_state ${PROJECT_NAME}_core)
    ament_target_dependencies(test_feature_geometry_update_state rclcpp)
  endif()

  ament_add_gtest(test_identity_runtime_record test/test_identity_runtime_record.cpp)
  if(TARGET test_identity_runtime_record)
    target_compile_features(test_identity_runtime_record PUBLIC cxx_std_17)
    target_include_directories(test_identity_runtime_record PRIVATE include src/modules)
    target_link_libraries(test_identity_runtime_record ${PROJECT_NAME}_core)
    ament_target_dependencies(test_identity_runtime_record rclcpp)
  endif()

  ament_add_gtest(test_identity_record_lifecycle test/test_identity_runtime_record_lifecycle.cpp)
  if(TARGET test_identity_record_lifecycle)
    target_compile_features(test_identity_record_lifecycle PUBLIC cxx_std_17)
    target_include_directories(test_identity_record_lifecycle PRIVATE include src/modules)
    target_link_libraries(test_identity_record_lifecycle ${PROJECT_NAME}_core)
    ament_target_dependencies(test_identity_record_lifecycle rclcpp)
  endif()

  ament_add_gtest(test_identity_runtime_lifecycle_coordinator test/test_identity_runtime_lifecycle_coordinator.cpp)
  if(TARGET test_identity_runtime_lifecycle_coordinator)
    target_compile_features(test_identity_runtime_lifecycle_coordinator PUBLIC cxx_std_17)
    target_include_directories(test_identity_runtime_lifecycle_coordinator PRIVATE include src/modules)
    target_link_libraries(test_identity_runtime_lifecycle_coordinator ${PROJECT_NAME}_core)
    ament_target_dependencies(test_identity_runtime_lifecycle_coordinator rclcpp)
  endif()

  ament_add_gtest(test_feature_bank_cost test/test_feature_bank_cost.cpp)
  if(TARGET test_feature_bank_cost)
    target_compile_features(test_feature_bank_cost PUBLIC cxx_std_17)
    target_include_directories(test_feature_bank_cost PRIVATE include)
    target_link_libraries(test_feature_bank_cost ${PROJECT_NAME}_core)
    ament_target_dependencies(test_feature_bank_cost rclcpp)
  endif()

  ament_add_gtest(test_reliable_geometry_cost test/test_reliable_geometry_cost.cpp)
  if(TARGET test_reliable_geometry_cost)
    target_compile_features(test_reliable_geometry_cost PUBLIC cxx_std_17)
    target_include_directories(test_reliable_geometry_cost PRIVATE include)
    target_link_libraries(test_reliable_geometry_cost ${PROJECT_NAME}_core)
    ament_target_dependencies(test_reliable_geometry_cost rclcpp)
  endif()

  ament_add_gtest(test_assignment_cost test/test_assignment_cost.cpp)
  if(TARGET test_assignment_cost)
    target_compile_features(test_assignment_cost PUBLIC cxx_std_17)
    target_include_directories(test_assignment_cost PRIVATE include src/modules)
    target_link_libraries(test_assignment_cost ${PROJECT_NAME}_core)
    ament_target_dependencies(test_assignment_cost rclcpp)
  endif()

  ament_add_gtest(test_active_assignment_solver test/test_active_assignment_solver.cpp)
  if(TARGET test_active_assignment_solver)
    target_compile_features(test_active_assignment_solver PUBLIC cxx_std_17)
    target_include_directories(test_active_assignment_solver PRIVATE include src/modules)
    target_link_libraries(test_active_assignment_solver ${PROJECT_NAME}_core)
    ament_target_dependencies(test_active_assignment_solver rclcpp)
  endif()

  ament_add_gtest(test_inactive_recovery_solver test/test_inactive_recovery_solver.cpp)
  if(TARGET test_inactive_recovery_solver)
    target_compile_features(test_inactive_recovery_solver PUBLIC cxx_std_17)
    target_include_directories(test_inactive_recovery_solver PRIVATE include src/modules)
    target_link_libraries(test_inactive_recovery_solver ${PROJECT_NAME}_core)
    ament_target_dependencies(test_inactive_recovery_solver rclcpp)
  endif()

  ament_add_gtest(test_inactive_recovery_input_collector test/test_inactive_recovery_input_collector.cpp)
  if(TARGET test_inactive_recovery_input_collector)
    target_compile_features(test_inactive_recovery_input_collector PUBLIC cxx_std_17)
    target_include_directories(test_inactive_recovery_input_collector PRIVATE include src/modules)
    target_link_libraries(test_inactive_recovery_input_collector ${PROJECT_NAME}_core)
    ament_target_dependencies(test_inactive_recovery_input_collector rclcpp)
  endif()

  ament_add_gtest(test_unresolved_track_final_resolution_coordinator test/test_unresolved_track_final_resolution_coordinator.cpp)
  if(TARGET test_unresolved_track_final_resolution_coordinator)
    target_compile_features(test_unresolved_track_final_resolution_coordinator PUBLIC cxx_std_17)
    target_include_directories(test_unresolved_track_final_resolution_coordinator PRIVATE include src/modules)
    target_link_libraries(test_unresolved_track_final_resolution_coordinator ${PROJECT_NAME}_core)
    ament_target_dependencies(test_unresolved_track_final_resolution_coordinator rclcpp)
  endif()

  ament_add_gtest(test_raw_continuity_decision test/test_raw_continuity_decision.cpp)
  if(TARGET test_raw_continuity_decision)
    target_compile_features(test_raw_continuity_decision PUBLIC cxx_std_17)
    target_include_directories(test_raw_continuity_decision PRIVATE include src/modules)
    target_link_libraries(test_raw_continuity_decision ${PROJECT_NAME}_core)
    ament_target_dependencies(test_raw_continuity_decision rclcpp)
  endif()

  ament_add_gtest(test_birth_candidate_decision test/test_birth_candidate_decision.cpp)
  if(TARGET test_birth_candidate_decision)
    target_compile_features(test_birth_candidate_decision PUBLIC cxx_std_17)
    target_include_directories(test_birth_candidate_decision PRIVATE include src/modules)
    target_link_libraries(test_birth_candidate_decision ${PROJECT_NAME}_core)
    ament_target_dependencies(test_birth_candidate_decision rclcpp)
  endif()

  ament_add_gtest(test_occlusion_mode_state test/test_occlusion_mode_state.cpp)
  if(TARGET test_occlusion_mode_state)
    target_compile_features(test_occlusion_mode_state PUBLIC cxx_std_17)
    target_include_directories(test_occlusion_mode_state PRIVATE include src/modules)
    target_link_libraries(test_occlusion_mode_state ${PROJECT_NAME}_core)
    ament_target_dependencies(test_occlusion_mode_state rclcpp)
  endif()

  ament_add_gtest(test_occlusion_group_shadow_lifecycle test/test_occlusion_group_shadow_lifecycle.cpp)
  if(TARGET test_occlusion_group_shadow_lifecycle)
    target_compile_features(test_occlusion_group_shadow_lifecycle PUBLIC cxx_std_17)
    target_include_directories(test_occlusion_group_shadow_lifecycle PRIVATE include src/modules)
    target_link_libraries(test_occlusion_group_shadow_lifecycle ${PROJECT_NAME}_core)
    ament_target_dependencies(test_occlusion_group_shadow_lifecycle rclcpp)
  endif()

  ament_add_gtest(test_birth_candidate_store test/test_birth_candidate_store.cpp)
  if(TARGET test_birth_candidate_store)
    target_compile_features(test_birth_candidate_store PUBLIC cxx_std_17)
    target_include_directories(test_birth_candidate_store PRIVATE include src/modules)
    target_link_libraries(test_birth_candidate_store ${PROJECT_NAME}_core)
    ament_target_dependencies(test_birth_candidate_store rclcpp)
  endif()

  ament_add_gtest(test_birth_manager test/test_birth_manager.cpp)
  if(TARGET test_birth_manager)
    target_compile_features(test_birth_manager PUBLIC cxx_std_17)
    target_include_directories(test_birth_manager PRIVATE include src/modules)
    target_link_libraries(test_birth_manager ${PROJECT_NAME}_core)
    ament_target_dependencies(test_birth_manager rclcpp)
  endif()

  ament_add_gtest(test_phase5_birth_coordinator test/test_phase5_birth_coordinator.cpp)
  if(TARGET test_phase5_birth_coordinator)
    target_compile_features(test_phase5_birth_coordinator PUBLIC cxx_std_17)
    target_include_directories(test_phase5_birth_coordinator PRIVATE include src/modules)
    target_link_libraries(test_phase5_birth_coordinator ${PROJECT_NAME}_core)
    ament_target_dependencies(test_phase5_birth_coordinator rclcpp)
  endif()

  ament_add_gtest(test_phase4_handoff_coordinator test/test_phase4_handoff_coordinator.cpp)
  if(TARGET test_phase4_handoff_coordinator)
    target_compile_features(test_phase4_handoff_coordinator PUBLIC cxx_std_17)
    target_include_directories(test_phase4_handoff_coordinator PRIVATE include src/modules)
    target_link_libraries(test_phase4_handoff_coordinator ${PROJECT_NAME}_core)
    ament_target_dependencies(test_phase4_handoff_coordinator rclcpp)
  endif()

  ament_add_gtest(test_merged_single_blob_assignment_decision test/test_merged_single_blob_assignment_decision.cpp)
  if(TARGET test_merged_single_blob_assignment_decision)
    target_compile_features(test_merged_single_blob_assignment_decision PUBLIC cxx_std_17)
    target_include_directories(test_merged_single_blob_assignment_decision PRIVATE include src/modules)
    target_link_libraries(test_merged_single_blob_assignment_decision ${PROJECT_NAME}_core)
    ament_target_dependencies(test_merged_single_blob_assignment_decision rclcpp)
  endif()

  ament_add_gtest(test_raw_semantic_binding test/test_raw_semantic_binding.cpp)
  if(TARGET test_raw_semantic_binding)
    target_compile_features(test_raw_semantic_binding PUBLIC cxx_std_17)
    target_include_directories(test_raw_semantic_binding PRIVATE include src/modules)
    target_link_libraries(test_raw_semantic_binding ${PROJECT_NAME}_core)
    ament_target_dependencies(test_raw_semantic_binding rclcpp)
  endif()

  ament_add_gtest(test_semantic_id_allocator test/test_semantic_id_allocator.cpp)
  if(TARGET test_semantic_id_allocator)
    target_compile_features(test_semantic_id_allocator PUBLIC cxx_std_17)
    target_include_directories(test_semantic_id_allocator PRIVATE include src/modules)
    target_link_libraries(test_semantic_id_allocator ${PROJECT_NAME}_core)
    ament_target_dependencies(test_semantic_id_allocator rclcpp)
  endif()

  ament_add_gtest(test_assignment_application_plan test/test_assignment_application_plan.cpp)
  if(TARGET test_assignment_application_plan)
    target_compile_features(test_assignment_application_plan PUBLIC cxx_std_17)
    target_include_directories(test_assignment_application_plan PRIVATE include src/modules)
    target_link_libraries(test_assignment_application_plan ${PROJECT_NAME}_core)
    ament_target_dependencies(test_assignment_application_plan rclcpp)
  endif()

  ament_add_gtest(test_assignment_application_executor test/test_assignment_application_executor.cpp)
  if(TARGET test_assignment_application_executor)
    target_compile_features(test_assignment_application_executor PUBLIC cxx_std_17)
    target_include_directories(test_assignment_application_executor PRIVATE include src/modules)
    target_link_libraries(test_assignment_application_executor ${PROJECT_NAME}_core)
    ament_target_dependencies(test_assignment_application_executor rclcpp)
  endif()

  ament_add_gtest(test_assignment_candidate_builder test/test_assignment_candidate_builder.cpp)
  if(TARGET test_assignment_candidate_builder)
    target_compile_features(test_assignment_candidate_builder PUBLIC cxx_std_17)
    target_include_directories(test_assignment_candidate_builder PRIVATE include src/modules)
    target_link_libraries(test_assignment_candidate_builder ${PROJECT_NAME}_core)
    ament_target_dependencies(test_assignment_candidate_builder rclcpp)
  endif()

  ament_add_gtest(test_active_assignment_input_collector test/test_active_assignment_input_collector.cpp)
  if(TARGET test_active_assignment_input_collector)
    target_compile_features(test_active_assignment_input_collector PUBLIC cxx_std_17)
    target_include_directories(test_active_assignment_input_collector PRIVATE include src/modules)
    target_link_libraries(test_active_assignment_input_collector ${PROJECT_NAME}_core)
    ament_target_dependencies(test_active_assignment_input_collector rclcpp)
  endif()

  ament_add_gtest(test_identity_runtime_store test/test_identity_runtime_store.cpp)
  if(TARGET test_identity_runtime_store)
    target_compile_features(test_identity_runtime_store PUBLIC cxx_std_17)
    target_include_directories(test_identity_runtime_store PRIVATE include src/modules)
    target_link_libraries(test_identity_runtime_store ${PROJECT_NAME}_core)
    ament_target_dependencies(test_identity_runtime_store rclcpp)
  endif()

  ament_add_gtest(test_mot_tracker_config test/test_mot_tracker_config.cpp)
  if(TARGET test_mot_tracker_config)
    target_compile_features(test_mot_tracker_config PUBLIC cxx_std_17)
    target_include_directories(test_mot_tracker_config PRIVATE include)
    target_link_libraries(test_mot_tracker_config ${PROJECT_NAME}_core)
    ament_target_dependencies(test_mot_tracker_config rclcpp)
  endif()

  ament_add_gtest(test_mot_tracker_observability test/test_mot_tracker_observability.cpp)
  if(TARGET test_mot_tracker_observability)
    target_compile_features(test_mot_tracker_observability PUBLIC cxx_std_17)
    target_include_directories(test_mot_tracker_observability PRIVATE include src/modules)
    target_link_libraries(test_mot_tracker_observability ${PROJECT_NAME}_core)
    ament_target_dependencies(test_mot_tracker_observability rclcpp)
  endif()

  ament_add_gtest(test_appearance_feature_service test/test_appearance_feature_service.cpp)
  if(TARGET test_appearance_feature_service)
    target_compile_features(test_appearance_feature_service PUBLIC cxx_std_17)
    target_include_directories(test_appearance_feature_service PRIVATE include)
    target_link_libraries(test_appearance_feature_service ${PROJECT_NAME}_core)
    ament_target_dependencies(test_appearance_feature_service rclcpp)
  endif()

  ament_add_gtest(test_yolo26_output_contract test/test_yolo26_output_contract.cpp)
  if(TARGET test_yolo26_output_contract)
    target_compile_features(test_yolo26_output_contract PUBLIC cxx_std_17)
    target_include_directories(test_yolo26_output_contract PRIVATE include)
    target_link_libraries(test_yolo26_output_contract ${PROJECT_NAME}_core)
    ament_target_dependencies(test_yolo26_output_contract rclcpp)
  endif()

  ament_add_gtest(test_offline_eval_schema test/test_offline_eval_schema.cpp)
  if(TARGET test_offline_eval_schema)
    target_compile_features(test_offline_eval_schema PUBLIC cxx_std_17)
    target_include_directories(test_offline_eval_schema PRIVATE include)
    target_link_libraries(test_offline_eval_schema ${PROJECT_NAME}_core)
    ament_target_dependencies(test_offline_eval_schema rclcpp)
  endif()

  ament_add_gtest(test_offline_eval_input test/test_offline_eval_input.cpp)
  if(TARGET test_offline_eval_input)
    target_compile_features(test_offline_eval_input PUBLIC cxx_std_17)
    target_include_directories(test_offline_eval_input PRIVATE include)
    target_link_libraries(test_offline_eval_input ${PROJECT_NAME}_core)
    ament_target_dependencies(test_offline_eval_input rclcpp)
  endif()

  if(TRACKING_ENABLE_ORIN_RUNTIME)
    ament_add_gtest(test_offline_replay_run test/test_offline_replay_run.cpp)
    if(TARGET test_offline_replay_run)
      target_compile_features(test_offline_replay_run PUBLIC cxx_std_17)
      target_include_directories(test_offline_replay_run PRIVATE include)
      target_link_libraries(test_offline_replay_run ${PROJECT_NAME}_offline_runtime)
      ament_target_dependencies(test_offline_replay_run rclcpp)
    endif()
  endif()

  ament_add_gtest(test_identity_offline_metrics test/test_identity_offline_metrics.cpp)
  if(TARGET test_identity_offline_metrics)
    target_compile_features(test_identity_offline_metrics PUBLIC cxx_std_17)
    target_include_directories(test_identity_offline_metrics PRIVATE include)
    target_link_libraries(test_identity_offline_metrics ${PROJECT_NAME}_core)
    ament_target_dependencies(test_identity_offline_metrics rclcpp)
  endif()
endif()

ament_package()
