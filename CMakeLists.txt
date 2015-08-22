cmake_minimum_required(VERSION 3.0)
project(QuadTargetFSM)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CURRENT_SOURCE_DIR src)
include_directories(include)

file(GLOB INCLUDE_FILES include/*.h*)
file(GLOB SRC_FILES src/*.c*)

add_executable(QuadTarget ${INCLUDE_FILES} ${SRC_FILES} )
target_link_libraries( QuadTarget ${OpenCV_LIBS} )

