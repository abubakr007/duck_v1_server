cmake_minimum_required(VERSION 3.8)
project(duck_nav2_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/twist_to_twist_stamped_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install map files
install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()