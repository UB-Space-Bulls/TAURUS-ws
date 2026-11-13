cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch worlds config meshes
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/zed_aruco_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
