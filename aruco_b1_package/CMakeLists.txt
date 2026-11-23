cmake_minimum_required(VERSION 3.8)
project(aruco_b1_package)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/aruco_b1_localizer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aruco_b1_localizer
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/anchor_policy.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME anchor_policy
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README_VI.md DESTINATION share/${PROJECT_NAME})

ament_package()
