cmake_minimum_required(VERSION 3.16)
project(kuka_supervised_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

find_path(SNAP7_INCLUDE_DIR snap7.h REQUIRED)
find_library(SNAP7_LIBRARY NAMES snap7 REQUIRED)
add_library(snap7_external UNKNOWN IMPORTED)
set_target_properties(snap7_external PROPERTIES
  IMPORTED_LOCATION "${SNAP7_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${SNAP7_INCLUDE_DIR}")

add_library(kuka_supervised_control SHARED
  src/supervisor_protocol.cpp
  src/plc_bridge.cpp
  src/rt_executor.cpp
  src/kuka_supervisor_system.cpp
)
target_include_directories(kuka_supervised_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(kuka_supervised_control PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
  snap7_external
  pthread)

pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kuka_supervised_control
  EXPORT export_kuka_supervised_control
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config plc launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_kuka_supervised_control HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib realtime_tools)
ament_package()
