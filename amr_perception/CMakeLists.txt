cmake_minimum_required(VERSION 3.8)
project(amr_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

# Install directories
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Make Python scripts executable
install(PROGRAMS
  scripts/scan_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
