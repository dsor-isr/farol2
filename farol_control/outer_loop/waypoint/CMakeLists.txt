cmake_minimum_required(VERSION 3.8)
project(waypoint)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(farol_msgs REQUIRED)
find_package(farol_utils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Collect all .srv files
file(GLOB SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(waypoint_node 
  src/waypoint.cpp
  src/wp_heading.cpp
  src/wp_loose.cpp
  src/wp_standard.cpp
)

ament_target_dependencies(
  waypoint_node
  rclcpp
  std_msgs
  farol_msgs
  farol_utils
  Eigen3
  eigen3_cmake_module
)

add_dependencies(waypoint_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Link generated service types
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(waypoint_node "${cpp_typesupport_target}")

target_include_directories(waypoint_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  
target_compile_features(waypoint_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
install(TARGETS waypoint_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
