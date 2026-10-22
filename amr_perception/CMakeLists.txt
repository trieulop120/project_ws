cmake_minimum_required(VERSION 3.8)
project(amr_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# =========================================================================
# Dependencies
# =========================================================================
find_package(ament_cmake REQUIRED)

# C++ dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)

# =========================================================================
# Executable C++
# =========================================================================

# 1. Node Downsample Pointcloud
add_executable(pointcloud_downsampler src/pointcloud_downsampler_node.cpp)
ament_target_dependencies(pointcloud_downsampler rclcpp sensor_msgs pcl_conversions pcl_ros)

# 2. Node Lọc Lidar (/scan -> /scan_filtered)
add_executable(scan_filter_node src/scan_filter_node.cpp)
ament_target_dependencies(scan_filter_node rclcpp sensor_msgs)

# =========================================================================
# Install C++ Executables
# =========================================================================
install(TARGETS
  pointcloud_downsampler
  scan_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================================================
# Install Python Executables (ament_cmake pattern)
# =========================================================================
install(PROGRAMS
  amr_perception/pallet_detector_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME pallet_detector
)

# =========================================================================
# Install Launch Files
# =========================================================================
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# =========================================================================
# Install Config Files
# =========================================================================
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# =========================================================================
# Install Weights (if exists in source)
# =========================================================================
install(DIRECTORY weights
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# =========================================================================
# Export Dependencies
# =========================================================================
ament_export_dependencies(
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  geometry_msgs
  vision_msgs
  tf2_ros
  image_transport
  message_filters
)

ament_package()
