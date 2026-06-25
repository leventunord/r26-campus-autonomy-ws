cmake_minimum_required(VERSION 3.8)
project(pedestrian_costmap_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(pedestrian_layer SHARED
  src/pedestrian_layer.cpp
)

ament_target_dependencies(pedestrian_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d pedestrian_layer.xml)

install(TARGETS pedestrian_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES pedestrian_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(pedestrian_layer)
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
