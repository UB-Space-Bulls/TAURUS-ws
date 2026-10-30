cmake_minimum_required(VERSION 3.8)
project(terrain_geometry)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# The interface and processor belong to this one ROS package. Keep the Python
# implementation private beside its executables: rosidl owns the public Python
# namespace `terrain_geometry` for generated messages. This avoids Jazzy's
# duplicate ament_python_install_package target / namespace collision.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TerrainGrid.msg"
  DEPENDENCIES std_msgs nav_msgs
)

install(DIRECTORY terrain_geometry_impl
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)
install(PROGRAMS scripts/terrain_node scripts/check_demo
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config worlds meshes docs scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(terrain_geometry_tests test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
