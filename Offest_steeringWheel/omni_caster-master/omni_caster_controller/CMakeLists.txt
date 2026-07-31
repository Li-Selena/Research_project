cmake_minimum_required(VERSION 3.8)
project(omni_caster_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2
  tf2_msgs
  nav_msgs
  rcpputils
  geometry_msgs
  generate_parameter_library
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add omni_caster_controller library related compile commands
generate_parameter_library(omni_caster_controller_parameters
  src/omni_caster_controller.yaml
)

add_library(
  omni_caster_controller
  SHARED
  src/omni_caster_controller.cpp
  src/omni_caster_chassis_kinematics.cpp
  src/speed_limiter.cpp
  src/odometry.cpp
)

target_include_directories(omni_caster_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(omni_caster_controller omni_caster_controller_parameters)
ament_target_dependencies(omni_caster_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(omni_caster_controller PRIVATE "OMNI_CASTER_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface omni_caster_controller.xml)

install(
  TARGETS
  omni_caster_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)


ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  omni_caster_controller
)

ament_package()
