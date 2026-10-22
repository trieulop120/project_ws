cmake_minimum_required(VERSION 3.8)
project(amr_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Python node: rf2o_cov_relay (sanitises the rf2o odometry for the EKF)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_localization/rf2o_cov_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rf2o_cov_relay
)

ament_package()
