cmake_minimum_required(VERSION 3.8)
project(path_following)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(farol_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(waypoint REQUIRED)
find_package(paths REQUIRED)
find_package(farol_utils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Collect all .msg and .srv files
# file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
file(GLOB SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  # ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(path_following_node 
  # ROS specific details 
  src/ros/PathFollowingNode.cpp
  src/ros/PathFollowingServices.cpp
	
  # Path Following Algorithms Implementations
  src/algorithms/PathFollowing.cpp
  src/algorithms/Aguiar.cpp
  src/algorithms/Romulo.cpp
  src/algorithms/Breivik.cpp
  src/algorithms/Lapierre.cpp
  src/algorithms/Pramod.cpp
  src/algorithms/Samson.cpp
  src/algorithms/Fossen.cpp
  src/algorithms/Marcelo.cpp
  src/algorithms/RelativeHeading.cpp
)

ament_target_dependencies(
  path_following_node
  rclcpp
  std_msgs
  std_srvs
  farol_msgs
  Eigen3
  eigen3_cmake_module
  waypoint
  paths
  farol_utils
)

# Link generated service types
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(path_following_node "${cpp_typesupport_target}")

target_include_directories(path_following_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/algorithms>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ros>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  
target_compile_features(path_following_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
install(TARGETS
  path_following_node
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