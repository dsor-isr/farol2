cmake_minimum_required(VERSION 3.8)
project(farol2_clock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_executable(sim_clock src/sim_clock.cpp)

ament_target_dependencies(
  sim_clock
  rclcpp
  rosgraph_msgs
)

target_include_directories(sim_clock PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(sim_clock PUBLIC cxx_std_17)

install(TARGETS
  sim_clock
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
