cmake_minimum_required(VERSION 3.8)
project(teleoperate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install only if the directory exists (prevents symlink install errors)
set(INSTALL_DIRS
  launch
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
