cmake_minimum_required(VERSION 3.8)
project(farol2_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  config
  docs
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
