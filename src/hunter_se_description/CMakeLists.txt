cmake_minimum_required(VERSION 3.5)
project(hunter_se_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY config launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# ament package
ament_package()
