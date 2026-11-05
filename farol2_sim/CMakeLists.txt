cmake_minimum_required(VERSION 3.16)
project(farol2_sim LANGUAGES CXX)

option(FAROL2_SIM_CORE_ONLY "Build the physics library and tests without ROS" OFF)
find_package(Eigen3 REQUIRED)
if(FAROL2_SIM_CORE_ONLY)
  set(FAROL2_UTILS_GEOMETRY_ONLY ON)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../farol2_utils
    ${CMAKE_CURRENT_BINARY_DIR}/farol2_utils_geometry)
else()
  find_package(farol2_utils REQUIRED)
endif()

add_library(farol2_sim_dynamics
  src/dynamics.cpp src/kinematics.cpp src/integrator.cpp src/sensors/sensor_kinematics.cpp
  src/thruster.cpp src/rudder.cpp)
target_compile_features(farol2_sim_dynamics PUBLIC cxx_std_17)
target_compile_options(farol2_sim_dynamics PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(farol2_sim_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(farol2_sim_dynamics PUBLIC Eigen3::Eigen farol2_utils::farol2_utils_geometry)

if(FAROL2_SIM_CORE_ONLY)
  include(CTest)
  if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    add_executable(test_dynamics test/test_dynamics.cpp)
    target_link_libraries(test_dynamics PRIVATE farol2_sim_dynamics GTest::gtest_main)
    add_test(NAME dynamics COMMAND test_dynamics)
    add_executable(test_sensor_kinematics test/test_sensor_kinematics.cpp)
    target_link_libraries(test_sensor_kinematics PRIVATE farol2_sim_dynamics GTest::gtest_main)
    add_test(NAME sensor_kinematics COMMAND test_sensor_kinematics)
  endif()
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(farol2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

add_executable(vehicle_sim src/vehicle_sim_node.cpp
  src/sensors/mounted_sensor.cpp src/sensors/noise.cpp src/sensors/gnss.cpp src/sensors/imu.cpp
  src/sensors/dvl.cpp src/sensors/depth_cell.cpp src/sensors/altimeter.cpp
  src/sensors/thruster_rpm.cpp src/sensors/control_surface_angle.cpp)
target_compile_features(vehicle_sim PRIVATE cxx_std_17)
target_compile_options(vehicle_sim PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(vehicle_sim rclcpp std_msgs farol2_interfaces geometry_msgs
  sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
target_include_directories(vehicle_sim PRIVATE include ${GEOGRAPHICLIB_INCLUDE_DIRS})
target_link_libraries(vehicle_sim farol2_sim_dynamics farol2_utils::farol2_utils ${GEOGRAPHICLIB_LIBRARIES})

add_executable(sim_clock src/sim_clock_node.cpp)
target_compile_features(sim_clock PRIVATE cxx_std_17)
target_include_directories(sim_clock PRIVATE include)
ament_target_dependencies(sim_clock rclcpp rosgraph_msgs)
target_link_libraries(sim_clock farol2_utils::farol2_utils_geometry)

install(TARGETS farol2_sim_dynamics EXPORT export_farol2_sim_dynamics
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS vehicle_sim sim_clock DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/farol2_sim DESTINATION include)
install(DIRECTORY launch docs config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
ament_export_targets(export_farol2_sim_dynamics HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 farol2_utils)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynamics test/test_dynamics.cpp)
  target_link_libraries(test_dynamics farol2_sim_dynamics)
  ament_add_gtest(test_sensor_kinematics test/test_sensor_kinematics.cpp)
  target_link_libraries(test_sensor_kinematics farol2_sim_dynamics)
  find_package(rcl_yaml_param_parser REQUIRED)
  ament_add_gtest(test_model_configs test/test_model_configs.cpp)
  target_compile_definitions(test_model_configs PRIVATE
    FAROL2_SOURCE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/..")
  ament_target_dependencies(test_model_configs rcl_yaml_param_parser)
  target_link_libraries(test_model_configs farol2_sim_dynamics)
  add_test(NAME simulation_launch_policy
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_simulation_launch.py)
  set_tests_properties(simulation_launch_policy PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../farol2_bringup:$ENV{PYTHONPATH}")
endif()

ament_package()
