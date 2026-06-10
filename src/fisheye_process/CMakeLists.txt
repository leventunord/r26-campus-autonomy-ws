cmake_minimum_required(VERSION 3.8)
project(fisheye_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(fisheye_stitcher_lib src/fisheye_stitcher.cpp)

add_executable(fisheye_stitch src/stitch.cpp)
target_link_libraries(fisheye_stitch fisheye_stitcher_lib ${OpenCV_LIBRARIES})
ament_target_dependencies(fisheye_stitch
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  ament_index_cpp
)

# 安装可执行文件和库
install(TARGETS
  fisheye_stitch
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装拼接所需的资源文件（MLS 网格、遮罩等）
install(DIRECTORY fisheyeStitcher/utils/
  DESTINATION share/${PROJECT_NAME}/utils
  PATTERN "*.gz"
  PATTERN "*.yml"
  PATTERN "*.jpg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
