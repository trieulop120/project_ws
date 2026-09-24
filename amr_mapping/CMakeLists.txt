cmake_minimum_required(VERSION 3.8)
project(amr_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install maps (bao gom ca amr_map.bt bi thieu trong setup.py cu)
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})

# Install rviz files
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
