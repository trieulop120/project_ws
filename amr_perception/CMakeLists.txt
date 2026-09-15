cmake_minimum_required(VERSION 3.8)
project(amr_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Executable
add_executable(pointcloud_downsampler src/pointcloud_downsampler_node.cpp)
ament_target_dependencies(pointcloud_downsampler rclcpp sensor_msgs pcl_conversions pcl_ros)

# Install
install(TARGETS pointcloud_downsampler DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rclcpp sensor_msgs pcl_conversions pcl_ros)
ament_package()
