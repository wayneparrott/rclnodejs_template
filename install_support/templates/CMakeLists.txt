cmake_minimum_required(VERSION 3.5)
project(_PKG_)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


#include rclnodejs project specific compile and install support
include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists_rclnodejs_project.txt)


ament_package()
