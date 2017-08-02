cmake_minimum_required(VERSION 2.8)
project(gps)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

option(GPS_DEBUG_MODE "Debug mode" ON)
if(GPS_DEBUG_MODE)
  add_definitions(-DGPS_DEBUG_MODE)
endif()
MESSAGE("GPS_DEBUG_MODE? " ${GPS_DEBUG_MODE})

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

add_subdirectory(src/mytcp)
add_subdirectory(src/mygps)
add_subdirectory(src/myserial)

add_executable(gpsserver src/gpsserver.cpp)
target_link_libraries(gpsserver  MyTCPServer MyTCPClient MyGPS MySerial)
