cmake_minimum_required(VERSION 3.8)
project(hunter_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch param config
  DESTINATION share/${PROJECT_NAME})

ament_package()
