cmake_minimum_required(VERSION 3.1.3)
set(CMAKE_CXX_STANDARD 17)

project(robot_control)

list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")

find_package(PkgConfig)
pkg_search_module(PINO REQUIRED pinocchio)
if(PINO_FOUND)
  message("Pinocchio found")
  message("Pinocchio cflags: " ${PINO_CFLAGS})
  message("Pinocchio ldflags: " ${PINO_LDFLAGS})
  message("Pinocchio include dir at: " ${PINO_INCLUDE_DIRS})
  message("Pinocchio libraries dir at: " ${PINO_LIBRARY_DIRS})
  include_directories(SYSTEM ${PINO_INCLUDE_DIRS})
  link_directories(${PINO_LIBRARY_DIRS})
else()
  message(FATAL_ERROR "Cannot find pinocchio")
endif(PINO_FOUND)

# find_package(pinocchio REQUIRED)
# if(pinocchio_FOUND)
#   message("pinocchio found")
#   include_directories(SYSTEM ${pinocchio_INCLUDE_DIRS})
#   message("pinocchio include dir at " ${pinocchio_INCLUDE_DIRS})
#   message("pinocchio libraries dir at " ${pinocchio_LIBRARIES})
# else()
#   message(FATAL_ERROR "Cannot find pinocchio")
# endif()

include_directories("./include")

include_directories(SYSTEM "${THIS_COM}/thirdparty/qpOASES/include/")
include_directories(SYSTEM "${THIS_COM}/thirdparty/ParamHandler/")
include_directories(SYSTEM "${THIS_COM}/thirdparty/inih/")
include_directories(SYSTEM "${THIS_COM}/Utilities/include/")
#include_directories(SYSTEM "${THIS_COM}/lcm/")
include_directories(SYSTEM "${THIS_COM}/user/")
include_directories(${CMAKE_BINARY_DIR})

file(GLOB_RECURSE sources "src/*.cpp" "main.cpp" "${THIS_COM}/user/*.cpp")
# file(GLOB_RECURSE sources2 "armctrl.cpp" "src/CtrlData.cpp" "src/RobotArmModel.cpp" "src/RobotArmController.cpp" )

#add_library(mininezhamodel SHARED "src/miniNezhaModel.cpp" "src/miniNezhaWBModel.cpp")
#target_link_libraries(mininezhamodel ${PINO_LDFLAGS})

add_library(robotmodel SHARED "src/RobotModel.cpp")
target_link_libraries(robotmodel ${PINO_LDFLAGS})

add_executable(${PROJECT_NAME} ${sources})
target_link_libraries(${PROJECT_NAME} PUBLIC inih dynacore_param_handler qpOASES pthread rt matio lcm robotmodel)

