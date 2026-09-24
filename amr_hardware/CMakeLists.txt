cmake_minimum_required(VERSION 3.8)
project(amr_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install launch files (exclude __pycache__)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/real_bringup.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config (udev rules)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/config/99-amr-usb.rules
  DESTINATION share/${PROJECT_NAME}/config
)

# Install standalone scripts (no entry_points needed)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/auto_detect_usb.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME auto_detect_usb
)

# Install console_scripts (entry_points from old setup.py)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_hardware/static_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME static_tf_broadcaster
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_hardware/esp32_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME esp32_bridge
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_hardware/teleop_amr.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_amr
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_hardware/cli_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cli_control
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_hardware/cmd_vel_splitter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_splitter
)

ament_package()
