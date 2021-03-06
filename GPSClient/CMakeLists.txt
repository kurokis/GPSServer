cmake_minimum_required(VERSION 2.8)
project(gpsclient)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

option(GPS_DEBUG_MODE "Debug mode" OFF)
if(GPS_DEBUG_MODE)
  add_definitions(-DGPS_DEBUG_MODE)
endif()
MESSAGE("GPS_DEBUG_MODE? " ${GPS_DEBUG_MODE})

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/gps ${CMAKE_CURRENT_BINARY_DIR}/gps_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/serial ${CMAKE_CURRENT_BINARY_DIR}/serial_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/tcp ${CMAKE_CURRENT_BINARY_DIR}/tcp_build)

add_executable(gpsclient src/gpsclient.cpp)
target_link_libraries(gpsclient TCPServer TCPClient GPS Serial)
