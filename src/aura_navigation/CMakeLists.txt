cmake_minimum_required(VERSION 3.8)
project(aura_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

add_library(aura_astar_planner SHARED src/map_reader.cpp)
target_compile_features(aura_astar_planner PUBLIC cxx_std_17)
target_compile_options(aura_astar_planner PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(aura_astar_planner
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d
  nav_msgs geometry_msgs tf2_ros pluginlib)

pluginlib_export_plugin_description_file(nav2_core aura_planner_plugin.xml)
install(TARGETS aura_astar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(aura_astar_planner)
ament_export_dependencies(nav2_core nav2_costmap_2d pluginlib)

ament_package()
