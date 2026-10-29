cmake_minimum_required(VERSION 3.8)
project(amr_mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install console_scripts
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_mission_manager/mission_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_manager
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_mission_manager/mission_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_gui
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_mission_manager/pallet_mission_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pallet_mission_node
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_mission_manager/pallet_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pallet_gui
  )
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_mission_manager/mission_supervisor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_supervisor
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/amr_mission_manager/rack_tf_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rack_tf_publisher
  )

ament_package()
