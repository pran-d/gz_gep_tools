cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)

# These variables have to be defined before running SETUP_PROJECT
set(PROJECT_NAME gz_gep_tools)
set(PROJECT_DESCRIPTION "Tools to control Gazebo through C++")
set(PROJECT_URL http://github.com/gepetto/gz_gep_tools)

include(cmake/base.cmake)

project(${PROJECT_NAME} CXX)

set(CMAKE_VERBOSE_MAKEFILE TRUE)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

# Find the Gazebo Transport library
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
find_package(gz-sim8 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# set(PROTO_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/msgs/gz/msgs)
# protobuf_generate_cpp(PROTO_SRCS PROTO_HDSR ${PROTO_SRC_DIR}/map_named_joint_forces.proto)
# add_library(${PROJECT_NAME}-msgs ${PROTO_SRCS} ${PROTO_HDRS})
# target_include_directories(${PROJECT_NAME}-msgs PUBLIC ${PROTOBUF_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
# target_link_libraries(${PROJECT_NAME}-msgs PUBLIC ${PROTOBUF_LIBRARIES})

add_library(
  gz_transport_hw_interface SHARED
  src/joint_state_interface.cc src/control_over_gz.cc
  src/perception_action_loop.cc src/robots_data.cc)
target_include_directories(
  gz_transport_hw_interface
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(
  gz_transport_hw_interface gz-transport${GZ_TRANSPORT_VER}::core
  gz-sim${GZ_SIM_VER}::core 
  ${PROJECT_NAME}-msgs
  )

add_executable(control_loop ./tools/control_loop.cc)
target_include_directories(
  control_loop PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)
target_link_libraries(control_loop gz_transport_hw_interface)

add_subdirectory(plugin)

install(TARGETS control_loop
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS gz_transport_hw_interface ${PROJECT_NAME}-msgs
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_package()
