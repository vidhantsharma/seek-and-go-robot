cmake_minimum_required(VERSION 3.8)
project(slam)

# Set C++ standard (slam_toolbox uses C++)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python executables (scripts)
install(PROGRAMS
  scripts/save_map_on_demand.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
