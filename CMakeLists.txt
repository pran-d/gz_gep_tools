cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-gep-tools)

set(CMAKE_VERBOSE_MAKEFILE TRUE)

# Find the Gazebo Transport library
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

add_library(gz_transport_hw_interface
  src/joint_state_interface.cc
)
target_compile_features(gz_transport_hw_interface PUBLIC cxx_std_17)
target_include_directories(gz_transport_hw_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>    
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gz_transport_hw_interface gz-transport${GZ_TRANSPORT_VER}::core)


add_executable(publisher publisher.cc)
target_include_directories(publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>    
  $<INSTALL_INTERFACE:include>)
target_compile_features(publisher PUBLIC cxx_std_17)
target_link_libraries(publisher gz_transport_hw_interface)

