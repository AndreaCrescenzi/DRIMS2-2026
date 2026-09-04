cmake_minimum_required(VERSION 3.16)
project(dice_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  geometry_msgs
  tf2_ros
  tf2
)

add_library(get_face_rotation SHARED
  src/get_face_rotation.cpp
)
list(APPEND plugin_libs get_face_rotation)

add_library(compute_xy_correction SHARED
  src/compute_xy_correction.cpp
)
list(APPEND plugin_libs compute_xy_correction)

add_library(get_grasp_orientation SHARED
  src/get_grasp_orientation.cpp
)
list(APPEND plugin_libs get_grasp_orientation)

add_library(plan_rotation_path SHARED
  src/plan_rotation_path.cpp
)
list(APPEND plugin_libs plan_rotation_path)

add_library(resolve_die_orientation SHARED
  src/resolve_die_orientation.cpp
)
list(APPEND plugin_libs resolve_die_orientation)

add_library(compute_canonical_placement SHARED
  src/compute_canonical_placement.cpp
)
list(APPEND plugin_libs compute_canonical_placement)

add_library(get_tip_rotation SHARED
  src/get_tip_rotation.cpp
)
list(APPEND plugin_libs get_tip_rotation)

add_library(get_world_grasp_orientation SHARED
  src/get_world_grasp_orientation.cpp
)
list(APPEND plugin_libs get_world_grasp_orientation)

add_library(select_tip_axis SHARED
  src/select_tip_axis.cpp
)
list(APPEND plugin_libs select_tip_axis)

add_library(check_closing_axis_vertical SHARED
  src/check_closing_axis_vertical.cpp
)
list(APPEND plugin_libs check_closing_axis_vertical)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} PUBLIC ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  target_include_directories(
    ${bt_plugin} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endforeach()

install(TARGETS
  ${plugin_libs}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
