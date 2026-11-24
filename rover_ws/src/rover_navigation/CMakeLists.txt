cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(motion_guard_tests test TIMEOUT 30)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch config rviz docs worlds
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md README_FULL_STACK.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/terrain_cost_grid.py
  scripts/stationary_odometry.py
  scripts/check_terrain_integration.py
  scripts/terrain_velocity_gate.py
  scripts/test_rover_navigation.py
  scripts/fastlio_sim_bridge.py
  scripts/test_fastlio_navigation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
