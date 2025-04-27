cmake_minimum_required(VERSION 2.8.3)
project(frenet_ilqr_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++17 -w)
set(CMAKE_CXX_FLAGS " -Wall -Wextra ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "Release")

set(catkin_deps
        roscpp
        grid_map_ros
        tinyspline_ros
        ros_viz_tools
        )

find_package(catkin REQUIRED COMPONENTS
        ${catkin_deps}
        )

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)  # 添加 glog 的查找

# Set the path to OSQP library and include files
set(OSQP_ROOT "${CMAKE_SOURCE_DIR}/path_optimizer_ilqr/src/osqp_solver/osqp")
set(OSQP_INCLUDE_DIR "${OSQP_ROOT}/include")
set(OSQP_LIB_DIR "${OSQP_ROOT}/lib")
# Add the OSQP include directory
include_directories(${OSQP_INCLUDE_DIR})

message(STATUS "CMAKE_SOURCE_DIR path: ${CMAKE_SOURCE_DIR}")


catkin_package(
        INCLUDE_DIRS src
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${catkin_deps}
        DEPENDS OpenCV
)

include_directories(
        src
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
        src/path/data_structure.cpp
        src/path/reference_line.cpp
        src/path/spline.cpp
        src/path/free_space.cpp
        src/path/tool.cpp
        src/path/free_space.cpp
        src/path/gflags.cpp
        src/solver/solver.cpp
        src/solver/problem_manager.cpp
        src/path/path_problem_manager.cpp
        )
target_link_libraries(${PROJECT_NAME}
        glog::glog  # 使用 glog 的现代 CMake 目标
        gflags
        ${catkin_LIBRARIES}
        )
message(STATUS "OSQP_LIB_DIR path: ${OSQP_LIB_DIR}")
add_executable(${PROJECT_NAME}_demo
        src/test/demo.cpp
        src/test/Map.cpp
        src/test/reference_line_processor.cpp
        src/osqp_solver/path_planning_interface.cpp
        src/osqp_solver/piecewise_jerk_path_problem.cc
        src/osqp_solver/piecewise_jerk_problem.cc
        src/a_star/a_star.cpp
        src/common_func/common_func.cc)
target_link_libraries(${PROJECT_NAME}_demo
        ${PROJECT_NAME}
        glog::glog  # 确保 demo 也链接 glog
        ${OpenCV_LIBRARIES}
        "${OSQP_LIB_DIR}/libosqp.so"
        )