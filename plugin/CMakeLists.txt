add_subdirectory(msgs)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

set(gz_gep_tools_plugin gz-sim-apply-joints-forces)

add_library(${gz_gep_tools_plugin}-system SHARED
  ApplyJointsForces.cc)

target_link_libraries(${gz_gep_tools_plugin}-system PRIVATE
  gz-sim::gz-sim
  gz-plugin::register
  {$PROJECT_NAME}-msgs
)

# Install directories
install(TARGETS ${gz_gep_tools_plugin}-system
  DESTINATION lib
)
