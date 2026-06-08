cmake_minimum_required(VERSION 3.1.0)
project(motion_planning_cpp)

find_package(catkin
 # ROS components
 REQUIRED COMPONENTS
 std_msgs 
 farol_msgs
 roscpp

 # farol_gimmicks_library
 farol_gimmicks_library
 std_srvs
 message_generation
 nav_msgs
 geometry_msgs
)


add_service_files(
  FILES
  SetState.srv
  SetBoundsandGains.srv
  SetBezierParams.srv
  SetObstacles.srv
  RunOptimization.srv
  GetPlannerConfig.srv
)

add_message_files(
  FILES
  StampedMatrix.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs message_runtime nav_msgs geometry_msgs
)

add_compile_options(-std=c++17 -Wall -g -O0) 

# -------------------------------
# Minimal change: enable AddressSanitizer
# -------------------------------
#if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
#    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
#    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
#endif()

include_directories(
 include/motion_planning_cpp_ros
 include/motion_planning_cpp_algorithms
 ${catkin_INCLUDE_DIRS}
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_path(CASADI_INCLUDE_DIR
  casadi/casadi.hpp
  HINTS $ENV{CASADI_PREFIX}/include
)

if(CASADI_INCLUDE_DIR)
  set(CASADI_INCLUDE_DIR ${CASADI_INCLUDE_DIR} ${CASADI_INCLUDE_DIR}/casadi)
  set(CASADI_FOUND_INCLUDE TRUE)
  message(STATUS "Found CasADi include dir: ${CASADI_INCLUDE_DIR}")
else()
  message(STATUS "Could not find CasADi include dir")
endif()

# Find CasADi library
find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib
)

# Check if CasADi library is found
if(CASADI_LIBRARY)
  set(CASADI_LIBRARIES ${CASADI_LIBRARY})
  message(STATUS "Found CasADi libs: ${CASADI_LIBRARIES}")
else()
  message(STATUS "Could not find CasADi libs")
endif()

# If CasADi found, set it up for linking
if(CASADI_FOUND_INCLUDE AND CASADI_LIBRARIES)
  set(CASADI_FOUND TRUE)
endif()

# add_executable(${PROJECT_NAME}_node src/motion_planning_cpp_ros/MotionPlanningCppNode.cpp src/motion_planning_cpp_algorithms/BezierOptimization.cpp)
add_executable(${PROJECT_NAME}_node
  src/motion_planning_cpp_algorithms/SingleVehicleMotionPlan.cpp
  src/motion_planning_cpp_algorithms/MultipleVehicleMotionPlan.cpp
  src/motion_planning_cpp_algorithms/BezierUtils.cpp
  src/motion_planning_cpp_ros/InteractiveMotionPlanningNode.cpp
  #src/motion_planning_cpp_ros/DistributedMotionPlanningNode.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
# HDF5
find_package(HDF5 REQUIRED COMPONENTS C CXX)
include_directories(${HDF5_INCLUDE_DIRS})
link_directories(${HDF5_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES} 
  ${CASADI_LIBRARIES}
  ${HDF5_LIBRARIES})
