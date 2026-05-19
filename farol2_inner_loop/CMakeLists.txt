cmake_minimum_required(VERSION 3.8)
project(farol2_inner_loop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(farol2_interfaces REQUIRED)
find_package(farol2_allocation REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(farol2_utils REQUIRED)
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
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(farol2_utils)

# Create reusable controller library as STATIC to work with farol2_utils
# (farol2_utils is compiled as static library without PIC)
add_library(${PROJECT_NAME}_controllers STATIC
  src/controller_pi.cpp
  src/controller_pid.cpp
  src/reference_generator.cpp
)

target_link_libraries(${PROJECT_NAME}_controllers PUBLIC
  farol2_utils::farol2_utils
)

target_include_directories(${PROJECT_NAME}_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_controllers PUBLIC cxx_std_17)

# Node executable
add_executable(pid_control src/pid.cpp)

# Open Loop node executable
add_executable(open_loop src/open_loop.cpp)

ament_target_dependencies(
  open_loop
  rclcpp
  std_msgs
  std_srvs
  farol2_interfaces
  farol2_allocation
  farol2_utils
)

target_include_directories(open_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(open_loop PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  pid_control
  rclcpp
  std_msgs
  std_srvs
  farol2_interfaces
  farol2_allocation
  farol2_utils
)

# Link controller library to the executable
target_link_libraries(pid_control
  ${PROJECT_NAME}_controllers
  farol2_utils::farol2_utils
)

# Link generated service types
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(pid_control "${cpp_typesupport_target}")

target_include_directories(pid_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(pid_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS
  ${PROJECT_NAME}_controllers
  EXPORT export_${PROJECT_NAME}_controllers
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

# Install executable
install(TARGETS
  pid_control
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  open_loop
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Export controller library for downstream packages
ament_export_targets(export_${PROJECT_NAME}_controllers HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

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