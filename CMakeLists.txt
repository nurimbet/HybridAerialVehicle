#
#
#

project(sdfReader)

cmake_minimum_required(VERSION 2.8)

find_package(Threads REQUIRED)
find_package(DART REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OMPL)
find_package(Bullet REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -mtune=native -g -Wall -O3")

#include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(${BULLET_INCLUDE_DIR})
#include_directories("/home/nurimbet/dart/")
#set(DART_INCLUDE_DIRS
#    "${CMAKE_INSTALL_PREFIX}/include"
#    "${EIGEN3_INCLUDE_DIRS}")
#if(HAVE_BULLET_COLLISION)
#  list(APPEND DART_INCLUDE_DIRS "${BULLET_INCLUDE_DIRS}")

aux_source_directory(. SRC_LIST)

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME}
    dart
    ${OMPL_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )
