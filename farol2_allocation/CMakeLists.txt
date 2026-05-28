cmake_minimum_required(VERSION 3.8)
project(farol2_allocation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(farol2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(farol2_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Collect all .msg and .srv files
file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
file(GLOB SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)


add_executable(thruster_allocation src/thruster_allocation.cpp)
add_executable(rpm_conversion src/rpm_conversion.cpp)
add_executable(wrench_manager src/wrench_manager.cpp)


ament_target_dependencies(
  thruster_allocation
  rclcpp
  std_msgs
  geometry_msgs
  farol2_interfaces
  Eigen3
  eigen3_cmake_module
  farol2_utils
  tf2
  tf2_ros
)

ament_target_dependencies(
  rpm_conversion
  rclcpp
  std_msgs
  farol2_interfaces
)


ament_target_dependencies(
  wrench_manager
  rclcpp
  std_msgs
  farol2_interfaces
)

# Link generated service types
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(thruster_allocation "${cpp_typesupport_target}")
target_link_libraries(rpm_conversion "${cpp_typesupport_target}")


target_include_directories(thruster_allocation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(rpm_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


target_include_directories(wrench_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(thruster_allocation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_compile_features(rpm_conversion PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
target_compile_features(wrench_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  thruster_allocation
  rpm_conversion
  wrench_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
