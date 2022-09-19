cmake_minimum_required(VERSION 3.5)
project(legged_wheel)

set(BUILD_TYPE_RELEASE TRUE)
set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99")
set(CMAKE_CXX_STANDARD 11)

set(MUJOCO_LIB_DIR ${CMAKE_SOURCE_DIR}/sim/lib/)
link_directories(${MUJOCO_LIB_DIR})

find_library(GLFW libglfw.so.3 HINTS ${MUJOCO_LIB_DIR})
# find_library(MUJOCO libmujoco200.so ${MUJOCO_LIB_DIR} REQUIRE

include_directories("./include/")
include_directories("../Utilities/include/")
include_directories("../controller/include/")
# include_directories("../lcm/")
link_directories(${CMAKE_BIN_DIR})

add_executable(legged_wheel_sim simulate.cpp)
target_include_directories(legged_wheel_sim PRIVATE ./include/)
target_link_libraries(legged_wheel_sim mujoco200 ${GLFW} GL glew uitools pthread rt)

