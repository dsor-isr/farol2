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
find_package(farol2_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(static_thruster_allocator src/static_thruster_allocator.cpp)
add_library(rudder_allocator src/rudder_allocator.cpp)
add_library(thruster_rpm_converter src/thruster_rpm_converter.cpp)
add_executable(allocation_node src/allocation_node.cpp)
add_executable(wrench_manager src/wrench_manager.cpp)


ament_target_dependencies(
  static_thruster_allocator
  rclcpp
  geometry_msgs
  Eigen3
  tf2
  tf2_ros
)

ament_target_dependencies(
  rudder_allocator
  rclcpp
  farol2_interfaces
  Eigen3
  farol2_utils
)

ament_target_dependencies(
  thruster_rpm_converter
  rclcpp
  farol2_interfaces
  Eigen3
)

ament_target_dependencies(
  allocation_node
  rclcpp
  std_msgs
  geometry_msgs
  farol2_interfaces
  farol2_utils
  tf2
  tf2_ros
  Eigen3
)

target_include_directories(static_thruster_allocator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(rudder_allocator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(thruster_rpm_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(static_thruster_allocator PUBLIC c_std_99 cxx_std_17)

target_compile_features(rudder_allocator PUBLIC c_std_99 cxx_std_17)

target_compile_features(thruster_rpm_converter PUBLIC c_std_99 cxx_std_17)

target_link_libraries(allocation_node rudder_allocator static_thruster_allocator thruster_rpm_converter)
target_compile_features(allocation_node PUBLIC c_std_99 cxx_std_17)


ament_target_dependencies(
  wrench_manager
  rclcpp
  std_msgs
  farol2_interfaces
)

target_include_directories(wrench_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  
target_compile_features(wrench_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  static_thruster_allocator
  rudder_allocator
  thruster_rpm_converter
  allocation_node
  wrench_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

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
