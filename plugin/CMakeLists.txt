find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

# Plugin Apply Joint Forces
set(gz_gep_tools_plugin gz-sim-apply-joints-forces)

add_library(${gz_gep_tools_plugin}-system SHARED ApplyJointsForces.cc)

target_link_libraries(
  ${gz_gep_tools_plugin}-system PRIVATE gz-sim::gz-sim gz-plugin::register
                                        ${PROJECT_NAME}-msgs)

# Install directories
install(TARGETS ${gz_gep_tools_plugin}-system DESTINATION lib)

# Plugin ContactTest
set(gz_gep_tools_plugin_ct gz-sim-contact-test)

add_library(${gz_gep_tools_plugin_ct}-system SHARED ContactTest.cc)

target_link_libraries(
  ${gz_gep_tools_plugin_ct}-system
  PRIVATE gz-sim::gz-sim gz-transport${GZ_TRANSPORT_VER}::core
          gz-plugin::register)

# Install directories
install(TARGETS ${gz_gep_tools_plugin_ct}-system DESTINATION lib)

add_subdirectory(msgs)
