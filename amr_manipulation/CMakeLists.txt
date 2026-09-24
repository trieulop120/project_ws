cmake_minimum_required(VERSION 3.8)
project(amr_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_manipulation/cmd_vel_splitter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_splitter
)

ament_package()
