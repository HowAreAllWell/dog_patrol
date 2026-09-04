cmake_minimum_required(VERSION 3.8)
project(scancontext)

# 消除 CMake 3.12+ 引入的 PCL_ROOT 环境变量策略警告
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Add Library
add_library(scancontext src/Scancontext.cpp)

# Target Include Directories
target_include_directories(scancontext PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Target Link Libraries
target_link_libraries(scancontext
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(scancontext rclcpp cv_bridge pcl_conversions)

# Export standard dependencies
ament_export_include_directories(include)
ament_export_libraries(scancontext)
ament_export_dependencies(rclcpp cv_bridge pcl_conversions PCL OpenCV Eigen3)

# Install Headers
install(DIRECTORY include/ DESTINATION include)

# Install Library Targets
install(TARGETS scancontext
  EXPORT export_scancontext
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_scancontext)

ament_package()
