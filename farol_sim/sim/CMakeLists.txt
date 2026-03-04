cmake_minimum_required(VERSION 3.8)
project(sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(farol_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_allocation REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)

add_library(sim_libraries INTERFACE)
target_include_directories(sim_libraries INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

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

add_executable(simulation src/simulation.cpp)
add_executable(sim_sensors src/sim_sensors.cpp)

ament_target_dependencies(
  simulation
  rclcpp
  std_msgs
  geometry_msgs
  farol_msgs
  control_allocation
)

ament_target_dependencies(
  sim_sensors
  rclcpp
  std_msgs
  geometry_msgs
  farol_msgs
  control_allocation
)

# Link generated service types
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(simulation "${cpp_typesupport_target}")
target_link_libraries(simulation Eigen3::Eigen)

if(TARGET GeographicLib::GeographicLib)
  target_link_libraries(sim_sensors GeographicLib::GeographicLib)  
  target_link_libraries(simulation GeographicLib::GeographicLib) 
else()
  target_include_directories(sim_sensors PRIVATE ${GeographicLib_INCLUDE_DIRS})
  target_include_directories(simulation PRIVATE ${GeographicLib_INCLUDE_DIRS})
  target_link_libraries(sim_sensors ${GeographicLib_LIBRARIES}) 
  target_link_libraries(simulation ${GeographicLib_LIBRARIES})    
endif()

target_include_directories(simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(sim_sensors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  
target_compile_features(simulation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
install(TARGETS
  simulation
  sim_sensors
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