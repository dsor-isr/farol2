cmake_minimum_required(VERSION 3.8)
project(vehicle_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(farol_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(control_allocation REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

# ======================
# Executables
# ======================

add_executable(auv_sim src/auv_sim.cpp)
add_executable(magic_electric_sim src/magic_electric_sim.cpp)

# ======================
# Dependencies linking
# ======================

ament_target_dependencies(
  auv_sim
  rclcpp
  std_msgs
  farol_msgs
  geometry_msgs
  rosgraph_msgs
  control_allocation
)

ament_target_dependencies(
  magic_electric_sim
  rclcpp
  std_msgs
  farol_msgs
  geometry_msgs
  rosgraph_msgs
  control_allocation
)

target_include_directories(auv_sim PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(magic_electric_sim PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(auv_sim PUBLIC cxx_std_17)
target_compile_features(magic_electric_sim PUBLIC cxx_std_17)

install(TARGETS
  auv_sim
  magic_electric_sim
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
