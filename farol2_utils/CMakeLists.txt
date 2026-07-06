cmake_minimum_required(VERSION 3.8)
project(farol2_utils)

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
  TARGETS ${PROJECT_NAME}
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
