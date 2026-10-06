cmake_minimum_required(VERSION 3.8)
project(amr_web)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install console_scripts (bridge node)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_web/bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME web_bridge
)

# INSTALL RESOURCE WEB (HTML, CSS, JS)
install(DIRECTORY resources/web/
  DESTINATION share/${PROJECT_NAME}/resources/web
)

ament_package()
