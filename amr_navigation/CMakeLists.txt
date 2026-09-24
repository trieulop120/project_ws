cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install config (nav2_params, bt/*.xml, graphs/*.{yaml,geojson})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install rviz files
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# Install console_scripts
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_navigation/yaml_to_geojson.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yaml_to_geojson
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_navigation/interactive_node_creator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME interactive_node_creator
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_navigation/route_graph_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME route_graph_publisher
)

ament_package()
