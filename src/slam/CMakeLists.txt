cmake_minimum_required(VERSION 3.8)
project(slam LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)            # provides imported target Eigen3::Eigen

# Only install headers if include/ exists
if(EXISTS "${CMAKE_SOURCE_DIR}/include")
  set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
else()
  set(INCLUDE_DIR "")
endif()

# -----------------------------------------------------------------------------
# Library (implementation)
# -----------------------------------------------------------------------------
set(SLAM_CORE_SRCS
  src/simple_slam.cpp
)

add_library(slam_core ${SLAM_CORE_SRCS})

# Provide include paths to consumers:
# - BUILD_INTERFACE: where headers live in source tree
# - INSTALL_INTERFACE: where headers will be after installation
target_include_directories(slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Link Eigen via imported target (plain signature)
target_link_libraries(slam_core Eigen3::Eigen)

# Declare dependencies for the library (ament helper uses plain signature internally)
ament_target_dependencies(slam_core
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# -----------------------------------------------------------------------------
# Executable (node)
# -----------------------------------------------------------------------------
add_executable(simple_slam_node src/simple_slam_node.cpp)

# Node needs the include path at build time; mark PRIVATE to this target
target_include_directories(simple_slam_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Link the node to the library (plain signature)
target_link_libraries(simple_slam_node slam_core)

# Runtime/compile deps for the node (plain)
ament_target_dependencies(simple_slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# -----------------------------------------------------------------------------
# Install targets, headers, scripts, launch, config
# -----------------------------------------------------------------------------
install(TARGETS
  slam_core
  simple_slam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(INCLUDE_DIR)
  install(DIRECTORY ${INCLUDE_DIR}/ DESTINATION include/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/scripts")
  install(PROGRAMS
    scripts/evaluate_trajectory_and_map.py
    scripts/save_map_on_demand.py
    scripts/save_map_runner.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS "${CMAKE_SOURCE_DIR}/config")
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  ament_add_gtest(test_icp test/test_icp.cpp)
  if(TARGET test_icp)
    target_include_directories(test_icp PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    # plain signature: link the library and Eigen target
    target_link_libraries(test_icp slam_core Eigen3::Eigen)
    ament_target_dependencies(test_icp
      rclcpp
      sensor_msgs
      nav_msgs
      geometry_msgs
      tf2
      tf2_ros
    )
  endif()

  ament_add_gtest(test_occupancy_grid test/test_occupancy_grid.cpp)
  if(TARGET test_occupancy_grid)
    target_include_directories(test_occupancy_grid PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(test_occupancy_grid slam_core Eigen3::Eigen)
    ament_target_dependencies(test_occupancy_grid
      rclcpp
      sensor_msgs
      nav_msgs
      geometry_msgs
      tf2
      tf2_ros
    )
  endif()

  ament_add_gtest(test_slam_node test/test_slam_node.cpp)
  if(TARGET test_slam_node)
    target_include_directories(test_slam_node PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(test_slam_node slam_core Eigen3::Eigen)
    ament_target_dependencies(test_slam_node
      rclcpp
      sensor_msgs
      nav_msgs
      geometry_msgs
      tf2
      tf2_ros
    )
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
