cmake_minimum_required(VERSION 3.8)
project(farol2_cpf_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(farol2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(farol2_planning REQUIRED)
find_package(farol2_utils REQUIRED)
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

add_executable(cpf_controller_node 
  # ROS Nodes
  src/ros/CpfNode.cpp
  src/ros/CpfServices.cpp

  # Algorithms
  src/algorithms/CPFControl.cpp
  src/algorithms/EventTriggered.cpp
)

ament_target_dependencies(
  cpf_controller_node
  rclcpp
  std_msgs
  std_srvs
  farol2_interfaces
  Eigen3
  eigen3_cmake_module
  farol2_utils
  farol2_planning
)


# Link generated service types
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(cpf_controller_node "${cpp_typesupport_target}")

target_include_directories(cpf_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/algorithms>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ros>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  
target_compile_features(cpf_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
install(TARGETS
  cpf_controller_node
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