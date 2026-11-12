cmake_minimum_required(VERSION 3.16)
project(farol2_utils)

option(FAROL2_UTILS_GEOMETRY_ONLY "Expose Eigen-only geometry helpers without ROS" OFF)
find_package(Eigen3 REQUIRED)
add_library(farol2_utils_geometry INTERFACE)
add_library(farol2_utils::farol2_utils_geometry ALIAS farol2_utils_geometry)
target_compile_features(farol2_utils_geometry INTERFACE cxx_std_17)
target_include_directories(farol2_utils_geometry INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(farol2_utils_geometry INTERFACE Eigen3::Eigen)
if(FAROL2_UTILS_GEOMETRY_ONLY)
  return()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}
  src/low_pass_filter.cpp
  src/notch_filter.cpp
  src/moving_average_filter.cpp
  src/tf_utils.cpp
  src/parameter_utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_eigen
  tf2_ros
)

# enforce at least c++ 17
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(
  TARGETS ${PROJECT_NAME} farol2_utils_geometry
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 rclcpp tf2 tf2_eigen tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry farol2_utils_geometry)

  ament_add_gtest(test_validation test/test_validation.cpp)
  target_link_libraries(test_validation farol2_utils_geometry)

  ament_add_gtest(test_parameter_utils test/test_parameter_utils.cpp)
  target_link_libraries(test_parameter_utils ${PROJECT_NAME})

  ament_add_gtest(test_notch_filter
    test/test_notch_filter.cpp
  )
  target_link_libraries(test_notch_filter ${PROJECT_NAME})

  ament_add_gtest(test_low_pass_filter
    test/test_low_pass_filter.cpp
  )
  target_link_libraries(test_low_pass_filter ${PROJECT_NAME})

  ament_add_gtest(test_moving_average_filter
    test/test_moving_average_filter.cpp
  )
  target_link_libraries(test_moving_average_filter ${PROJECT_NAME})
endif()

ament_package()
