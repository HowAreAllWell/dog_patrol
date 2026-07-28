cmake_minimum_required(VERSION 3.8)
project(vision_demo_host)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)

find_library(TENSORRT_LIB nvinfer REQUIRED)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h REQUIRED)

option(ENABLE_HIK_MVS "Enable Hikrobot MVS camera backend" ON)

set(HIK_MVS_AVAILABLE OFF)
if(ENABLE_HIK_MVS)
  find_path(MVS_INCLUDE_DIR MvCameraControl.h PATHS /opt/MVS/include)
  find_library(MVS_LIB MvCameraControl PATHS /opt/MVS/lib/64 /opt/MVS/lib/aarch64)
  if(MVS_INCLUDE_DIR AND MVS_LIB)
    set(HIK_MVS_AVAILABLE ON)
    message(STATUS "Hik MVS backend enabled")
    get_filename_component(MVS_LIB_DIR "${MVS_LIB}" DIRECTORY)
  else()
    message(WARNING "ENABLE_HIK_MVS=ON but the MVS SDK was not found under /opt/MVS; building without Hik MVS backend")
  endif()
endif()

set(MODULE_SOURCES
  src/modules/camera_ingest.cpp
  src/modules/preprocess_infer.cpp
  src/modules/yolo26_output_contract.cpp
  src/modules/det_filter.cpp
  src/modules/appearance_feature_service.cpp
  src/modules/reid_embedder.cpp
  src/modules/association_utils.cpp
  src/modules/mot_tracker.cpp
  src/modules/primary_target_manager.cpp
  src/modules/primary_recovery_debug.cpp
  src/modules/bearing_estimator.cpp
  src/modules/udp_json_adapter.cpp
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
  src/modules/identity_observation_projection.cpp
  src/modules/identity_manager.cpp
  src/modules/visualizer_recorder.cpp
  src/modules/runtime_monitor.cpp
  src/tools/identity_offline_metrics.cpp
)

add_library(${PROJECT_NAME}_modules ${MODULE_SOURCES})
target_include_directories(${PROJECT_NAME}_modules PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TENSORRT_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME}_modules PRIVATE src/modules)
target_compile_features(${PROJECT_NAME}_modules PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_modules rclcpp)
target_link_libraries(${PROJECT_NAME}_modules ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_modules ${TENSORRT_LIB} CUDA::cudart)
if(HIK_MVS_AVAILABLE)
  target_include_directories(${PROJECT_NAME}_modules PRIVATE ${MVS_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME}_modules ${MVS_LIB})
  target_compile_definitions(${PROJECT_NAME}_modules PRIVATE VISION_DEMO_HOST_ENABLE_HIK_MVS=1)
  set_property(TARGET ${PROJECT_NAME}_modules APPEND PROPERTY BUILD_RPATH "${MVS_LIB_DIR}")
  set_property(TARGET ${PROJECT_NAME}_modules APPEND PROPERTY INSTALL_RPATH "${MVS_LIB_DIR}")
endif()

add_executable(vision_demo_node src/main.cpp)
target_compile_features(vision_demo_node PUBLIC cxx_std_17)
ament_target_dependencies(vision_demo_node rclcpp)
target_link_libraries(vision_demo_node ${PROJECT_NAME}_modules)

add_executable(record_test_set src/tools/record_test_set.cpp)
target_compile_features(record_test_set PUBLIC cxx_std_17)
target_link_libraries(record_test_set ${PROJECT_NAME}_modules ${OpenCV_LIBS})

add_executable(offline_eval_recordings src/tools/offline_eval_recordings.cpp)
target_compile_features(offline_eval_recordings PUBLIC cxx_std_17)
target_link_libraries(offline_eval_recordings ${PROJECT_NAME}_modules ${OpenCV_LIBS})

install(TARGETS
  ${PROJECT_NAME}_modules
  vision_demo_node
  record_test_set
  offline_eval_recordings
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_camera_ingest_contract test/test_camera_ingest_contract.cpp)
  if(TARGET test_camera_ingest_contract)
    target_compile_features(test_camera_ingest_contract PUBLIC cxx_std_17)
    target_include_directories(test_camera_ingest_contract PRIVATE include)
    target_link_libraries(test_camera_ingest_contract ${PROJECT_NAME}_modules)
  endif()

  ament_add_gtest(test_primary_target_manager test/test_primary_target_manager.cpp)
  if(TARGET test_primary_target_manager)
    target_compile_features(test_primary_target_manager PUBLIC cxx_std_17)
    target_include_directories(test_primary_target_manager PRIVATE include)
    target_link_libraries(test_primary_target_manager ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_primary_target_manager rclcpp)
  endif()

  ament_add_gtest(test_primary_recovery_debug test/test_primary_recovery_debug.cpp)
  if(TARGET test_primary_recovery_debug)
    target_compile_features(test_primary_recovery_debug PUBLIC cxx_std_17)
    target_include_directories(test_primary_recovery_debug PRIVATE include)
    target_link_libraries(test_primary_recovery_debug ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_primary_recovery_debug rclcpp)
  endif()

  ament_add_gtest(test_identity_assignment_engine_adapter test/test_identity_assignment_engine_adapter.cpp)
  if(TARGET test_identity_assignment_engine_adapter)
    target_compile_features(test_identity_assignment_engine_adapter PUBLIC cxx_std_17)
    target_include_directories(test_identity_assignment_engine_adapter PRIVATE include src/modules)
    target_link_libraries(test_identity_assignment_engine_adapter ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_identity_assignment_engine_adapter rclcpp)
  endif()

  ament_add_gtest(test_identity_manager test/test_identity_manager.cpp)
  if(TARGET test_identity_manager)
    target_compile_features(test_identity_manager PUBLIC cxx_std_17)
    target_include_directories(test_identity_manager PRIVATE include)
    target_link_libraries(test_identity_manager ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_identity_manager rclcpp)
  endif()

  ament_add_gtest(test_identity_observation_projection test/test_identity_observation_projection.cpp)
  if(TARGET test_identity_observation_projection)
    target_compile_features(test_identity_observation_projection PUBLIC cxx_std_17)
    target_include_directories(test_identity_observation_projection PRIVATE include src/modules)
    target_link_libraries(test_identity_observation_projection ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_identity_observation_projection rclcpp)
  endif()

  ament_add_gtest(test_feature_update_policy test/test_feature_update_policy.cpp)
  if(TARGET test_feature_update_policy)
    target_compile_features(test_feature_update_policy PUBLIC cxx_std_17)
    target_include_directories(test_feature_update_policy PRIVATE include)
    target_link_libraries(test_feature_update_policy ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_feature_update_policy rclcpp)
  endif()

  ament_add_gtest(test_feature_geometry_update_state test/test_feature_geometry_update_state.cpp)
  if(TARGET test_feature_geometry_update_state)
    target_compile_features(test_feature_geometry_update_state PUBLIC cxx_std_17)
    target_include_directories(test_feature_geometry_update_state PRIVATE include)
    target_link_libraries(test_feature_geometry_update_state ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_feature_geometry_update_state rclcpp)
  endif()

  ament_add_gtest(test_identity_runtime_record test/test_identity_runtime_record.cpp)
  if(TARGET test_identity_runtime_record)
    target_compile_features(test_identity_runtime_record PUBLIC cxx_std_17)
    target_include_directories(test_identity_runtime_record PRIVATE include src/modules)
    target_link_libraries(test_identity_runtime_record ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_identity_runtime_record rclcpp)
  endif()

  ament_add_gtest(test_identity_record_lifecycle test/test_identity_runtime_record_lifecycle.cpp)
  if(TARGET test_identity_record_lifecycle)
    target_compile_features(test_identity_record_lifecycle PUBLIC cxx_std_17)
    target_include_directories(test_identity_record_lifecycle PRIVATE include src/modules)
    target_link_libraries(test_identity_record_lifecycle ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_identity_record_lifecycle rclcpp)
  endif()

  ament_add_gtest(test_identity_runtime_lifecycle_coordinator test/test_identity_runtime_lifecycle_coordinator.cpp)
  if(TARGET test_identity_runtime_lifecycle_coordinator)
    target_compile_features(test_identity_runtime_lifecycle_coordinator PUBLIC cxx_std_17)
    target_include_directories(test_identity_runtime_lifecycle_coordinator PRIVATE include src/modules)
    target_link_libraries(test_identity_runtime_lifecycle_coordinator ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_identity_runtime_lifecycle_coordinator rclcpp)
  endif()

  ament_add_gtest(test_feature_bank_cost test/test_feature_bank_cost.cpp)
  if(TARGET test_feature_bank_cost)
    target_compile_features(test_feature_bank_cost PUBLIC cxx_std_17)
    target_include_directories(test_feature_bank_cost PRIVATE include)
    target_link_libraries(test_feature_bank_cost ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_feature_bank_cost rclcpp)
  endif()

  ament_add_gtest(test_reliable_geometry_cost test/test_reliable_geometry_cost.cpp)
  if(TARGET test_reliable_geometry_cost)
    target_compile_features(test_reliable_geometry_cost PUBLIC cxx_std_17)
    target_include_directories(test_reliable_geometry_cost PRIVATE include)
    target_link_libraries(test_reliable_geometry_cost ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_reliable_geometry_cost rclcpp)
  endif()

  ament_add_gtest(test_assignment_cost test/test_assignment_cost.cpp)
  if(TARGET test_assignment_cost)
    target_compile_features(test_assignment_cost PUBLIC cxx_std_17)
    target_include_directories(test_assignment_cost PRIVATE include src/modules)
    target_link_libraries(test_assignment_cost ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_assignment_cost rclcpp)
  endif()

  ament_add_gtest(test_active_assignment_solver test/test_active_assignment_solver.cpp)
  if(TARGET test_active_assignment_solver)
    target_compile_features(test_active_assignment_solver PUBLIC cxx_std_17)
    target_include_directories(test_active_assignment_solver PRIVATE include src/modules)
    target_link_libraries(test_active_assignment_solver ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_active_assignment_solver rclcpp)
  endif()

  ament_add_gtest(test_inactive_recovery_solver test/test_inactive_recovery_solver.cpp)
  if(TARGET test_inactive_recovery_solver)
    target_compile_features(test_inactive_recovery_solver PUBLIC cxx_std_17)
    target_include_directories(test_inactive_recovery_solver PRIVATE include src/modules)
    target_link_libraries(test_inactive_recovery_solver ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_inactive_recovery_solver rclcpp)
  endif()

  ament_add_gtest(test_inactive_recovery_input_collector test/test_inactive_recovery_input_collector.cpp)
  if(TARGET test_inactive_recovery_input_collector)
    target_compile_features(test_inactive_recovery_input_collector PUBLIC cxx_std_17)
    target_include_directories(test_inactive_recovery_input_collector PRIVATE include src/modules)
    target_link_libraries(test_inactive_recovery_input_collector ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_inactive_recovery_input_collector rclcpp)
  endif()

  ament_add_gtest(test_unresolved_track_final_resolution_coordinator test/test_unresolved_track_final_resolution_coordinator.cpp)
  if(TARGET test_unresolved_track_final_resolution_coordinator)
    target_compile_features(test_unresolved_track_final_resolution_coordinator PUBLIC cxx_std_17)
    target_include_directories(test_unresolved_track_final_resolution_coordinator PRIVATE include src/modules)
    target_link_libraries(test_unresolved_track_final_resolution_coordinator ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_unresolved_track_final_resolution_coordinator rclcpp)
  endif()

  ament_add_gtest(test_raw_continuity_decision test/test_raw_continuity_decision.cpp)
  if(TARGET test_raw_continuity_decision)
    target_compile_features(test_raw_continuity_decision PUBLIC cxx_std_17)
    target_include_directories(test_raw_continuity_decision PRIVATE include src/modules)
    target_link_libraries(test_raw_continuity_decision ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_raw_continuity_decision rclcpp)
  endif()

  ament_add_gtest(test_birth_candidate_decision test/test_birth_candidate_decision.cpp)
  if(TARGET test_birth_candidate_decision)
    target_compile_features(test_birth_candidate_decision PUBLIC cxx_std_17)
    target_include_directories(test_birth_candidate_decision PRIVATE include src/modules)
    target_link_libraries(test_birth_candidate_decision ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_birth_candidate_decision rclcpp)
  endif()

  ament_add_gtest(test_occlusion_mode_state test/test_occlusion_mode_state.cpp)
  if(TARGET test_occlusion_mode_state)
    target_compile_features(test_occlusion_mode_state PUBLIC cxx_std_17)
    target_include_directories(test_occlusion_mode_state PRIVATE include src/modules)
    target_link_libraries(test_occlusion_mode_state ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_occlusion_mode_state rclcpp)
  endif()

  ament_add_gtest(test_occlusion_group_shadow_lifecycle test/test_occlusion_group_shadow_lifecycle.cpp)
  if(TARGET test_occlusion_group_shadow_lifecycle)
    target_compile_features(test_occlusion_group_shadow_lifecycle PUBLIC cxx_std_17)
    target_include_directories(test_occlusion_group_shadow_lifecycle PRIVATE include src/modules)
    target_link_libraries(test_occlusion_group_shadow_lifecycle ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_occlusion_group_shadow_lifecycle rclcpp)
  endif()

  ament_add_gtest(test_birth_candidate_store test/test_birth_candidate_store.cpp)
  if(TARGET test_birth_candidate_store)
    target_compile_features(test_birth_candidate_store PUBLIC cxx_std_17)
    target_include_directories(test_birth_candidate_store PRIVATE include src/modules)
    target_link_libraries(test_birth_candidate_store ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_birth_candidate_store rclcpp)
  endif()

  ament_add_gtest(test_birth_manager test/test_birth_manager.cpp)
  if(TARGET test_birth_manager)
    target_compile_features(test_birth_manager PUBLIC cxx_std_17)
    target_include_directories(test_birth_manager PRIVATE include src/modules)
    target_link_libraries(test_birth_manager ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_birth_manager rclcpp)
  endif()

  ament_add_gtest(test_phase5_birth_coordinator test/test_phase5_birth_coordinator.cpp)
  if(TARGET test_phase5_birth_coordinator)
    target_compile_features(test_phase5_birth_coordinator PUBLIC cxx_std_17)
    target_include_directories(test_phase5_birth_coordinator PRIVATE include src/modules)
    target_link_libraries(test_phase5_birth_coordinator ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_phase5_birth_coordinator rclcpp)
  endif()

  ament_add_gtest(test_phase4_handoff_coordinator test/test_phase4_handoff_coordinator.cpp)
  if(TARGET test_phase4_handoff_coordinator)
    target_compile_features(test_phase4_handoff_coordinator PUBLIC cxx_std_17)
    target_include_directories(test_phase4_handoff_coordinator PRIVATE include src/modules)
    target_link_libraries(test_phase4_handoff_coordinator ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_phase4_handoff_coordinator rclcpp)
  endif()

  ament_add_gtest(test_merged_single_blob_assignment_decision test/test_merged_single_blob_assignment_decision.cpp)
  if(TARGET test_merged_single_blob_assignment_decision)
    target_compile_features(test_merged_single_blob_assignment_decision PUBLIC cxx_std_17)
    target_include_directories(test_merged_single_blob_assignment_decision PRIVATE include src/modules)
    target_link_libraries(test_merged_single_blob_assignment_decision ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_merged_single_blob_assignment_decision rclcpp)
  endif()

  ament_add_gtest(test_raw_semantic_binding test/test_raw_semantic_binding.cpp)
  if(TARGET test_raw_semantic_binding)
    target_compile_features(test_raw_semantic_binding PUBLIC cxx_std_17)
    target_include_directories(test_raw_semantic_binding PRIVATE include src/modules)
    target_link_libraries(test_raw_semantic_binding ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_raw_semantic_binding rclcpp)
  endif()

  ament_add_gtest(test_semantic_id_allocator test/test_semantic_id_allocator.cpp)
  if(TARGET test_semantic_id_allocator)
    target_compile_features(test_semantic_id_allocator PUBLIC cxx_std_17)
    target_include_directories(test_semantic_id_allocator PRIVATE include src/modules)
    target_link_libraries(test_semantic_id_allocator ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_semantic_id_allocator rclcpp)
  endif()

  ament_add_gtest(test_assignment_application_plan test/test_assignment_application_plan.cpp)
  if(TARGET test_assignment_application_plan)
    target_compile_features(test_assignment_application_plan PUBLIC cxx_std_17)
    target_include_directories(test_assignment_application_plan PRIVATE include src/modules)
    target_link_libraries(test_assignment_application_plan ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_assignment_application_plan rclcpp)
  endif()

  ament_add_gtest(test_assignment_application_executor test/test_assignment_application_executor.cpp)
  if(TARGET test_assignment_application_executor)
    target_compile_features(test_assignment_application_executor PUBLIC cxx_std_17)
    target_include_directories(test_assignment_application_executor PRIVATE include src/modules)
    target_link_libraries(test_assignment_application_executor ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_assignment_application_executor rclcpp)
  endif()

  ament_add_gtest(test_assignment_candidate_builder test/test_assignment_candidate_builder.cpp)
  if(TARGET test_assignment_candidate_builder)
    target_compile_features(test_assignment_candidate_builder PUBLIC cxx_std_17)
    target_include_directories(test_assignment_candidate_builder PRIVATE include src/modules)
    target_link_libraries(test_assignment_candidate_builder ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_assignment_candidate_builder rclcpp)
  endif()

  ament_add_gtest(test_active_assignment_input_collector test/test_active_assignment_input_collector.cpp)
  if(TARGET test_active_assignment_input_collector)
    target_compile_features(test_active_assignment_input_collector PUBLIC cxx_std_17)
    target_include_directories(test_active_assignment_input_collector PRIVATE include src/modules)
    target_link_libraries(test_active_assignment_input_collector ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_active_assignment_input_collector rclcpp)
  endif()

  ament_add_gtest(test_identity_runtime_store test/test_identity_runtime_store.cpp)
  if(TARGET test_identity_runtime_store)
    target_compile_features(test_identity_runtime_store PUBLIC cxx_std_17)
    target_include_directories(test_identity_runtime_store PRIVATE include src/modules)
    target_link_libraries(test_identity_runtime_store ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_identity_runtime_store rclcpp)
  endif()

  ament_add_gtest(test_mot_tracker_config test/test_mot_tracker_config.cpp)
  if(TARGET test_mot_tracker_config)
    target_compile_features(test_mot_tracker_config PUBLIC cxx_std_17)
    target_include_directories(test_mot_tracker_config PRIVATE include)
    target_link_libraries(test_mot_tracker_config ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_mot_tracker_config rclcpp)
  endif()

  ament_add_gtest(test_appearance_feature_service test/test_appearance_feature_service.cpp)
  if(TARGET test_appearance_feature_service)
    target_compile_features(test_appearance_feature_service PUBLIC cxx_std_17)
    target_include_directories(test_appearance_feature_service PRIVATE include)
    target_link_libraries(test_appearance_feature_service ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_appearance_feature_service rclcpp)
  endif()

  ament_add_gtest(test_udp_json_adapter test/test_udp_json_adapter.cpp)
  if(TARGET test_udp_json_adapter)
    target_compile_features(test_udp_json_adapter PUBLIC cxx_std_17)
    target_include_directories(test_udp_json_adapter PRIVATE include)
    target_link_libraries(test_udp_json_adapter ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_udp_json_adapter rclcpp)
  endif()

  ament_add_gtest(test_yolo26_output_contract test/test_yolo26_output_contract.cpp)
  if(TARGET test_yolo26_output_contract)
    target_compile_features(test_yolo26_output_contract PUBLIC cxx_std_17)
    target_include_directories(test_yolo26_output_contract PRIVATE include)
    target_link_libraries(test_yolo26_output_contract ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_yolo26_output_contract rclcpp)
  endif()

  ament_add_gtest(test_offline_eval_schema test/test_offline_eval_schema.cpp)
  if(TARGET test_offline_eval_schema)
    target_compile_features(test_offline_eval_schema PUBLIC cxx_std_17)
    target_include_directories(test_offline_eval_schema PRIVATE include)
    target_link_libraries(test_offline_eval_schema ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_offline_eval_schema rclcpp)
  endif()

  ament_add_gtest(test_identity_offline_metrics test/test_identity_offline_metrics.cpp)
  if(TARGET test_identity_offline_metrics)
    target_compile_features(test_identity_offline_metrics PUBLIC cxx_std_17)
    target_include_directories(test_identity_offline_metrics PRIVATE include)
    target_link_libraries(test_identity_offline_metrics ${PROJECT_NAME}_modules)
    ament_target_dependencies(test_identity_offline_metrics rclcpp)
  endif()
endif()

ament_package()
