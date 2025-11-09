cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED) # provides EIGEN3_INCLUDE_DIR

# Make include/ visible to the target and add Eigen include dir
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

# Install headers (so other packages can include them)
# This will copy the contents of include/ into <install_prefix>/include/
if(EXISTS "${CMAKE_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

# Source - update if your .cpp filename is different
set(SRC_FILES
  src/simple_slam_node.cpp
)

add_executable(simple_slam_node ${SRC_FILES})

ament_target_dependencies(simple_slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Make sure the target can find your headers and Eigen
target_include_directories(simple_slam_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS simple_slam_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
