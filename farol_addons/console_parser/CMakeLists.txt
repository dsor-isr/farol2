cmake_minimum_required(VERSION 3.8)
project(console_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(farol_msgs REQUIRED)
find_package(farol_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(paths REQUIRED)
# find_package(path_following REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

add_executable(console_parser_node src/console_parser.cpp)

ament_target_dependencies(
  console_parser_node
  rclcpp
  std_msgs
  farol_msgs
  farol_utils
  paths
  # path_following
  Eigen3
  eigen3_cmake_module
)

target_include_directories(console_parser_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  
target_compile_features(console_parser_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
install(TARGETS
  console_parser_node
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