cmake_minimum_required(VERSION 3.8)
project(sensor_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(farol_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)




#find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Collect all .msg and .srv files
#file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
#file(GLOB SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

# Generate messages and services
#rosidl_generate_interfaces(${PROJECT_NAME}
#  ${MSG_FILES}
#  ${SRV_FILES}
#  DEPENDENCIES std_msgs geometry_msgs
#)
#ament_export_dependencies(rosidl_default_runtime)

add_executable(sim_measurements src/sim_measurements.cpp)



ament_target_dependencies(
  sim_measurements
  rclcpp
  std_msgs
  farol_msgs
)

target_include_directories(sim_measurements PRIVATE
  ${GEOGRAPHICLIB_INCLUDE_DIRS}
)

target_link_libraries(sim_measurements
  ${GEOGRAPHICLIB_LIBRARIES}
)

target_include_directories(sim_measurements PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(sim_measurements PUBLIC c_std_99 cxx_std_17)

install(TARGETS sim_measurements
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