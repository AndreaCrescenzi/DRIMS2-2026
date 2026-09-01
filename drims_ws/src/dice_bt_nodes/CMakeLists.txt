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
