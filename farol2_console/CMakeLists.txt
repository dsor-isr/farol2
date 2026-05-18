cmake_minimum_required(VERSION 3.8)
project(farol2_console)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(farol2_interfaces REQUIRED)
find_package(farol2_path_following REQUIRED)
find_package(farol2_waypoint REQUIRED)
find_package(farol2_planning REQUIRED)
find_package(farol2_utils REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(console_parser_node
	src/console_parser.cpp
)

add_executable(nav2console_state_node
	src/nav2console_state.cpp
)

ament_target_dependencies(console_parser_node
	rclcpp
	std_msgs
	geometry_msgs
	farol2_interfaces
	farol2_path_following
	farol2_planning
	farol2_utils
	Eigen3
)

ament_target_dependencies(nav2console_state_node
	rclcpp
	farol2_interfaces
)

target_include_directories(console_parser_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(nav2console_state_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(console_parser_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(nav2console_state_node PUBLIC c_std_99 cxx_std_17)

target_link_libraries(console_parser_node Eigen3::Eigen)

install(TARGETS
	console_parser_node
	nav2console_state_node
	DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
	scripts/console_server.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME console_server
)

install(DIRECTORY include/
	DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	set(ament_cmake_copyright_FOUND TRUE)
	set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
