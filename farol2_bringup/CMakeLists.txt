cmake_minimum_required(VERSION 3.8)
project(farol2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config_default
  docs
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
