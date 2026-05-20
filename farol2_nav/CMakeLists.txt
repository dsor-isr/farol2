cmake_minimum_required(VERSION 3.8)
project(farol2_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------
# Dependencies
# --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(farol2_interfaces REQUIRED)
find_package(farol2_allocation REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(farol2_utils REQUIRED)

find_library(GEOGRAPHIC_LIB GeographicLib)


# --------------------
# Interfaces (msg/srv)
# --------------------
file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
file(GLOB SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# --------------------
# Executables
# --------------------
add_executable(filter_handler src/filter_handler.cpp)
add_executable(sample_and_hold src/sample_and_hold.cpp)
add_executable(filter_node
  src/filter_node.cpp
  src/filters/pass_through.cpp
  src/filters/pos_ekf.cpp
  src/filters/mahony.cpp
)

ament_target_dependencies(filter_handler
  rclcpp
  std_msgs
  sensor_msgs
  farol2_interfaces
)

ament_target_dependencies(sample_and_hold
  rclcpp
  std_msgs
  farol2_interfaces
  farol2_allocation
  farol2_utils
)

ament_target_dependencies(filter_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  farol2_interfaces
  farol2_allocation
)


# Include dirs
target_include_directories(filter_handler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(sample_and_hold PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


# C/C++ standard
target_compile_features(filter_handler PUBLIC c_std_99 cxx_std_17)
target_compile_features(sample_and_hold PUBLIC c_std_99 cxx_std_17)
target_compile_features(filter_node PUBLIC c_std_99 cxx_std_17)

# --------------------
# Link libs (plain signature to match ament)
# --------------------
# Link this package's generated typesupport (needed if you use nav_filters msg/srv types)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(filter_handler "${cpp_typesupport_target}")
target_link_libraries(sample_and_hold "${cpp_typesupport_target}" ${GEOGRAPHIC_LIB})
target_link_libraries(filter_node "${cpp_typesupport_target}" ${GEOGRAPHIC_LIB})

# Eigen
target_link_libraries(sample_and_hold Eigen3::Eigen)
target_link_libraries(filter_node Eigen3::Eigen)

# --------------------
# Install
# --------------------
install(TARGETS
  filter_handler
  sample_and_hold
  filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --------------------
# Testing
# --------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
