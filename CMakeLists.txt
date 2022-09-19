cmake_minimum_required(VERSION 3.5)
project(leggedWheel)

set(BUILD_TYPE_RELEASE TRUE)
set(CMAKE_CXX_FLAGS "-DEIGEN_STACK_ALLOCATION_LIMIT=0 -O3 -no-pie -ggdb -march=native -mavx -mfma -ffast-math -fPIC")

set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -fPIC")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(THIS_COM ${CMAKE_CURRENT_SOURCE_DIR}/)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
          ${CMAKE_BINARY_DIR}/Configuration.h)

message("THIS_COM: " ${THIS_COM})
message("Current cmake binary dir: " ${CMAKE_BINARY_DIR})

include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(controller)
# add_subdirectory(robot)
add_subdirectory(sim)
add_subdirectory(thirdparty)

