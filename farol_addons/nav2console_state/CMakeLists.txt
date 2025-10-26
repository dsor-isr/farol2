cmake_minimum_required(VERSION 3.8)
project(nav2console_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(farol_msgs REQUIRED)
find_package(farol_utils REQUIRED)
# find_package(path_following REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

add_executable(nav2console_state_node src/nav2console_state.cpp)

ament_target_dependencies(
  nav2console_state_node
  rclcpp
  std_msgs
  farol_msgs
  farol_utils
)

target_include_directories(nav2console_state_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  
target_compile_features(nav2console_state_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
install(TARGETS
  nav2console_state_node
  DESTINATION lib/${PROJECT_NAME}
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