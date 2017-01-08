cmake_minimum_required(VERSION 3.0)

set(MODULE_NAME Osiris)
add_definitions(-DMODULE_NAME=\"Osiris\")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_definitions(-Wall)
add_definitions(-fPIC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z")
add_definitions(-std=c++1z)

project(Osiris)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Threads)


find_package(OpenCV 3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

#find_package(DanLib REQUIRED)
#
#include_directories(${DanLib_INCLUDE_DIR})

include_directories(src)

set(CMAKE_DEBUG_POSTFIX d)

add_executable(
  Osiris
  src/Osiris.cpp
  src/FilteredCircularBuffer.hpp
  )

target_link_libraries(
  Osiris
  ${OpenCV_LIBRARIES}
  )
