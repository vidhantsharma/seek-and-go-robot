cmake_minimum_required(VERSION 3.8)
project(warehouse_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)                # if you later add C++ nodes
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros REQUIRED)            # Gazebo integration

# NOTE: xacro is a Python package — no need for find_package(xacro)

# Install only if the directory exists (prevents symlink install errors)
set(INSTALL_DIRS
  launch
  models
  worlds
)

foreach(dir IN LISTS INSTALL_DIRS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    message(STATUS "Installing directory: ${dir}")
    install(
      DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
    )
  else()
    message(WARNING "Directory '${dir}' not found, skipping installation")
  endif()
endforeach()

# Include folder (for C++ headers if you add them later)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(
    DIRECTORY include/
    DESTINATION include/
  )
endif()

ament_package()