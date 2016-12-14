project(Hybrid)

cmake_minimum_required(VERSION 2.8)

find_package(Threads REQUIRED)
find_package(DART REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OMPL REQUIRED)
find_package(Bullet REQUIRED)

include(FindBoost)
find_package(Boost REQUIRED system)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall ")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${BULLET_INCLUDE_DIR})


aux_source_directory(. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "./sdfReader.cpp")
list(REMOVE_ITEM SRC_LIST "./main.cpp")
list(REMOVE_ITEM SRC_LIST "./Quadrotor.cpp")
list(REMOVE_ITEM SRC_LIST "./FixedWingPlanningNonApp.cpp")
list(REMOVE_ITEM SRC_LIST "./QuadrotorPlanningNonApp.cpp")

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME}
    dart
    ${OMPL_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )
