#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake3 REQUIRED)

find_package(gz-msgs10 REQUIRED)
# Define a variable 'GZ_MSGS_VER' holding the version number
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

# Define a variable 'MSGS_PROTOS' listing the .proto files
set(MSGS_PROTOS
   ${CMAKE_CURRENT_SOURCE_DIR}/gz/msgs/map_named_joint_forces.proto
)

# Call 'gz_msgs_generate_messages()' to process the .proto files
gz_msgs_generate_messages(
  # The cmake target to be generated for libraries/executables to link
  TARGET msgs
  # The protobuf package to generate (Typically based on the path)
  PROTO_PACKAGE "gz.msgs"
  # The path to the base directory of the proto files
  # All import paths should be relative to this (eg gz/custom_msgs/vector3d.proto)
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  # List of proto files to process
  MSGS_PROTOS ${MSGS_PROTOS}
  # Depenency on gz-msgs
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

# From gz_msgs_generate.cmake in gz-msgs package
# TARGET in gz_msgs_generate_messages is translated in a cmake target named
# ${PROJET_NAME}-${generate_messages_TARGET}
# which means here ${PROJET_NAME}-msgs
install(TARGETS ${PROJECT_NAME}-msgs
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
