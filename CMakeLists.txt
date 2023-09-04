cmake_minimum_required(VERSION 3.8)
project(ardupilot_sitl_sim_interface)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 20)


############################################################################

# Dependencies

find_package(ament_cmake REQUIRED)

############################################################################

# Library

include_directories(include)

add_library(socket_example src/socket_example.cpp)
ament_export_targets(socket_example HAS_LIBRARY_TARGET)

add_library(lib_ap_json src/lib_ap_json.cpp)
ament_export_targets(lib_ap_json HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/ardupilot_sitl_sim_interface
  DESTINATION include)

install(
  TARGETS socket_example
  EXPORT socket_example
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(
  TARGETS lib_ap_json
  EXPORT lib_ap_json
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

target_link_libraries(lib_ap_json socket_example)


############################################################################

# Executables


############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

