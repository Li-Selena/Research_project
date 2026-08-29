cmake_minimum_required(VERSION 3.16)
project(offset_caster_mujoco_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(offset_caster_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(mujoco CONFIG QUIET)
if(TARGET mujoco::mujoco)
  set(MUJOCO_TARGET mujoco::mujoco)
elseif(TARGET mujoco)
  set(MUJOCO_TARGET mujoco)
else()
  find_path(
    MUJOCO_INCLUDE_DIR
    mujoco/mujoco.h
    HINTS $ENV{MUJOCO_DIR}
    PATH_SUFFIXES include
  )
  find_library(
    MUJOCO_LIBRARY
    NAMES mujoco
    HINTS $ENV{MUJOCO_DIR}
    PATH_SUFFIXES lib
  )
  if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
    message(FATAL_ERROR "MuJoCo C++ SDK not found; set MUJOCO_DIR or CMAKE_PREFIX_PATH")
  endif()
  add_library(mujoco::mujoco UNKNOWN IMPORTED)
  set_target_properties(
    mujoco::mujoco
    PROPERTIES
      IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
  )
  set(MUJOCO_TARGET mujoco::mujoco)
endif()

add_library(
  caster_kinematics
  src/forward_kinematics.cpp
  src/frame_transforms.cpp
  src/inverse_kinematics.cpp
)
target_include_directories(
  caster_kinematics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(caster_kinematics PRIVATE -Wall -Wextra -Wpedantic)

add_library(motion_controller src/motion_controller.cpp)
target_include_directories(
  motion_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(motion_controller PUBLIC caster_kinematics)
target_compile_options(motion_controller PRIVATE -Wall -Wextra -Wpedantic)

add_library(
  mujoco_state_reader
  src/mujoco_imu_reader.cpp
  src/mujoco_state_reader.cpp
)
target_include_directories(
  mujoco_state_reader
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(mujoco_state_reader PUBLIC caster_kinematics ${MUJOCO_TARGET})
target_compile_options(mujoco_state_reader PRIVATE -Wall -Wextra -Wpedantic)

add_executable(inverse_kinematics_node src/inverse_kinematics_node.cpp)
target_link_libraries(inverse_kinematics_node caster_kinematics)
ament_target_dependencies(
  inverse_kinematics_node
  geometry_msgs
  rclcpp
  sensor_msgs
  trajectory_msgs
)
target_compile_options(inverse_kinematics_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mujoco_state_node src/mujoco_state_node.cpp)
target_link_libraries(
  mujoco_state_node
  caster_kinematics
  mujoco_state_reader
  ${MUJOCO_TARGET}
  glfw
)
ament_target_dependencies(
  mujoco_state_node
  builtin_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  rosgraph_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  tf2_ros
)
target_compile_options(mujoco_state_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motion_controller_node src/motion_controller_node.cpp)
target_link_libraries(motion_controller_node motion_controller)
ament_target_dependencies(
  motion_controller_node
  geometry_msgs
  nav_msgs
  offset_caster_interfaces
  rclcpp
  sensor_msgs
)
target_compile_options(motion_controller_node PRIVATE -Wall -Wextra -Wpedantic)

install(
  TARGETS caster_kinematics motion_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(
  TARGETS mujoco_state_reader inverse_kinematics_node mujoco_state_node motion_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  include(CTest)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_executable(inverse_kinematics_test test/inverse_kinematics_test.cpp)
  target_link_libraries(inverse_kinematics_test PRIVATE caster_kinematics)
  target_compile_options(inverse_kinematics_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME inverse_kinematics_test COMMAND inverse_kinematics_test)

  add_executable(forward_kinematics_test test/forward_kinematics_test.cpp)
  target_link_libraries(forward_kinematics_test PRIVATE caster_kinematics)
  target_compile_options(forward_kinematics_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME forward_kinematics_test COMMAND forward_kinematics_test)

  add_executable(frame_transforms_test test/frame_transforms_test.cpp)
  target_link_libraries(frame_transforms_test PRIVATE caster_kinematics)
  target_compile_options(frame_transforms_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME frame_transforms_test COMMAND frame_transforms_test)

  add_executable(motion_controller_test test/motion_controller_test.cpp)
  target_link_libraries(motion_controller_test PRIVATE motion_controller)
  target_compile_options(motion_controller_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME motion_controller_test COMMAND motion_controller_test)

  set(
    DEFAULT_MUJOCO_MODEL_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/../../3Dmodel/offestWheel/MJCF/offset_steering_wheel.xml"
  )
  if(EXISTS "${DEFAULT_MUJOCO_MODEL_PATH}")
    add_executable(mujoco_state_reader_test test/mujoco_state_reader_test.cpp)
    target_link_libraries(
      mujoco_state_reader_test
      PRIVATE caster_kinematics mujoco_state_reader ${MUJOCO_TARGET}
    )
    target_compile_options(mujoco_state_reader_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(
      NAME mujoco_state_reader_test
      COMMAND mujoco_state_reader_test "${DEFAULT_MUJOCO_MODEL_PATH}"
    )

    add_executable(mujoco_imu_reader_test test/mujoco_imu_reader_test.cpp)
    target_link_libraries(mujoco_imu_reader_test PRIVATE mujoco_state_reader ${MUJOCO_TARGET})
    target_compile_options(mujoco_imu_reader_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(
      NAME mujoco_imu_reader_test
      COMMAND mujoco_imu_reader_test "${DEFAULT_MUJOCO_MODEL_PATH}"
    )
  else()
    message(WARNING "MuJoCo model not found; state-reader integration test is disabled")
  endif()

  add_launch_test(test/motion_modes_integration.launch.py TIMEOUT 60)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(caster_kinematics)

ament_package()
