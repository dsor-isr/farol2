cmake_minimum_required(VERSION 3.8)
project(farol2_motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# --------------------
# Dependencies
# --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(casadi REQUIRED)

# Find CasADi
find_path(CASADI_INCLUDE_DIR
  casadi/casadi.hpp
  HINTS $ENV{CASADI_PREFIX}/include
)

if(CASADI_INCLUDE_DIR)
  set(CASADI_INCLUDE_DIR ${CASADI_INCLUDE_DIR} ${CASADI_INCLUDE_DIR}/casadi)
  set(CASADI_FOUND_INCLUDE TRUE)
  message(STATUS "Found CasADi include dir: ${CASADI_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "Could not find CasADi include dir")
endif()

# Find CasADi library
find_library(CASADI_LIBRARY
  NAMES casadi
  HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib
)

if(CASADI_LIBRARY)
  set(CASADI_LIBRARIES ${CASADI_LIBRARY})
  message(STATUS "Found CasADi libs: ${CASADI_LIBRARIES}")
else()
  message(FATAL_ERROR "Could not find CasADi libs")
endif()

if(CASADI_FOUND_INCLUDE AND CASADI_LIBRARIES)
  set(CASADI_FOUND TRUE)
endif()

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
add_executable(${PROJECT_NAME}_node
  src/algorithms/multiple_vehicle_planner.cpp
  src/algorithms/bezier_utils.cpp
  src/ros/interactive_planner_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
)

# Include directories
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CASADI_INCLUDE_DIR}
)

# C/C++ standard
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)

# --------------------
# Link libraries
# --------------------
# Link this package's generated typesupport (for custom msg/srv types)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node
  "${cpp_typesupport_target}"
  Eigen3::Eigen
  ${CASADI_LIBRARIES}
)

# --------------------
# Install
# --------------------
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config
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
