cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# include Eigen headers (Eigen3 sets EIGEN3_INCLUDE_DIR variable)
include_directories(include ${EIGEN3_INCLUDE_DIR})

# Install Python executables (scripts) if present
if(EXISTS "${CMAKE_SOURCE_DIR}/scripts")
  install(PROGRAMS
    scripts/save_map_on_demand.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install launch and config directories (if present)
if(EXISTS "${CMAKE_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()
if(EXISTS "${CMAKE_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

add_executable(simple_slam_node src/simple_slam_node.cpp)
ament_target_dependencies(simple_slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

target_include_directories(simple_slam_node PRIVATE ${EIGEN3_INCLUDE_DIR})

install(TARGETS simple_slam_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
