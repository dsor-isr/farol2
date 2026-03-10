cmake_minimum_required(VERSION 3.8)
project(sim_time)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_executable(sim_clock_node
  src/sim_clock.cpp
)

add_executable(clock_checker_node
  src/clock_checker.cpp   )

target_include_directories(sim_clock_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(clock_checker_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(sim_clock_node
  rclcpp
  rosgraph_msgs
)

ament_target_dependencies(clock_checker_node
  rclcpp
  rosgraph_msgs
)

install(
  TARGETS sim_clock_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS clock_checker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()
