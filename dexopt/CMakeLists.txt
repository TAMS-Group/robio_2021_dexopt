cmake_minimum_required(VERSION 3.1)

project(dexopt)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -mavx -U__AVX__")
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  tractor
  moveit_ros_planning
  tf
  tf_conversions
  interactive_markers
  moveit_ros_robot_interaction
  kdl_parser
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tractor
  moveit_ros_planning
  tf
  tf_conversions
  interactive_markers
  moveit_ros_robot_interaction
  kdl_parser
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dexopt src/ops.cpp src/dexopt.cpp)
add_dependencies(dexopt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dexopt ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})
set_target_properties(dexopt PROPERTIES OUTPUT_NAME dexopt PREFIX "")
