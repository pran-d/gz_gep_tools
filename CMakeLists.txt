
cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)

project(gz-gep-tools)


set(CMAKE_VERBOSE_MAKEFILE TRUE)

# Find the Gazebo Transport library
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
find_package(gz-sim8 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_library(gz_transport_hw_interface SHARED
  src/joint_state_interface.cc
  src/control_over_gz.cc
)
target_include_directories(gz_transport_hw_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>    
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gz_transport_hw_interface
  gz-transport${GZ_TRANSPORT_VER}::core
  gz-sim${GZ_SIM_VER}::core)


add_executable(control_loop ./tools/control_loop.cc)
target_include_directories(control_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>    
  $<INSTALL_INTERFACE:include>)
target_link_libraries(control_loop gz_transport_hw_interface)

