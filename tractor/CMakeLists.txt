cmake_minimum_required(VERSION 3.1)
project(tractor)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -mavx -U__AVX__")
set(CMAKE_CXX_STANDARD 17)

find_package(Bullet REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_core
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        moveit_core
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    include
)

add_library(${PROJECT_NAME}
    src/collision/link.cpp
    src/collision/query.cpp
    src/collision/robot.cpp
    src/collision/shape.cpp
    src/core/allocator.cpp
    src/core/array.cpp
    src/core/batch.cpp
    src/core/buffer.cpp
    src/core/constraints.cpp
    src/core/eigen.cpp
    src/core/engine.cpp
    src/core/gradients.cpp
    src/core/linesearch.cpp
    src/core/operator.cpp
    src/core/ops.cpp
    src/core/profiler.cpp
    src/core/program.cpp
    src/core/recorder.cpp
    src/core/solver.cpp
    src/core/type.cpp
    src/core/var.cpp
    src/engines/base.cpp
    src/engines/jit.cpp
    src/engines/loop.cpp
    src/engines/simple.cpp
    src/engines/test.cpp
    src/geometry/eigen.cpp
    src/geometry/fast.cpp
    src/geometry/ops.cpp
    src/geometry/plane.cpp
    src/geometry/pose.cpp
    src/geometry/quaternion.cpp
    src/geometry/twist.cpp
    src/geometry/types.cpp
    src/geometry/vector3.cpp
    src/robot/goals.cpp
    src/robot/jointstate.cpp
    src/robot/jointtypes.cpp
    src/robot/linkstate.cpp
    src/robot/robot.cpp
    src/robot/robotinfo.cpp
    src/robot/robotmodel.cpp
    src/robot/robotstate.cpp
    src/robot/trajectory.cpp
    src/solvers/base.cpp
    src/solvers/ec.cpp
    src/solvers/gd.cpp
    src/solvers/ip.cpp
    src/solvers/pgd.cpp
    src/solvers/ps.cpp
    src/solvers/sq.cpp
    src/test/test.cpp
    src/tractor.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BULLET_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
