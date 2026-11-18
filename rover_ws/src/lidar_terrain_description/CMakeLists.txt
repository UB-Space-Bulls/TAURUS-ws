cmake_minimum_required(VERSION 3.8)
project(lidar_terrain_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY meshes launch config worlds docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/mesh_marker DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
ament_package()
