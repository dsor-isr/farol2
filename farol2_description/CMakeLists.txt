cmake_minimum_required(VERSION 3.8)
project(farol2_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install resources
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()