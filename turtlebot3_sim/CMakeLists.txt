cmake_minimum_required(VERSION 3.8)
project(turtlebot3_sim)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY configs launch worlds models maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
