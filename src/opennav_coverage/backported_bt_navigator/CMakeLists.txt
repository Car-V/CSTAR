cmake_minimum_required(VERSION 3.5)
project(backported_bt_navigator)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name bt_navigator)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav_msgs
  nav2_msgs
  behaviortree_cpp_v3
  std_srvs
  nav2_util
  nav2_core
  tf2_ros
  pluginlib
)

add_library(${library_name} SHARED
  src/bt_navigator.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(nav2_navigate_to_pose_navigator SHARED src/navigators/navigate_to_pose.cpp)
ament_target_dependencies(nav2_navigate_to_pose_navigator ${dependencies})
target_compile_definitions(nav2_navigate_to_pose_navigator PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_navigate_through_poses SHARED src/navigators/navigate_through_poses.cpp)
ament_target_dependencies(nav2_navigate_through_poses ${dependencies})
target_compile_definitions(nav2_navigate_through_poses PRIVATE BT_PLUGIN_EXPORT)

pluginlib_export_plugin_description_file(backported_bt_navigator navigator_plugins.xml)
rclcpp_components_register_nodes(${library_name} "backported_bt_navigator::BtNavigator")

install(TARGETS ${library_name} nav2_navigate_to_pose_navigator nav2_navigate_through_poses
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} nav2_navigate_to_pose_navigator nav2_navigate_through_poses)
ament_export_dependencies(${dependencies})
ament_package()
