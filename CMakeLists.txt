cmake_minimum_required(VERSION 3.0)
project(QuadTargetFSM)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CURRENT_SOURCE_DIR src)

set(SOURCE_FILES src/Marker.cpp src/Target.cpp src/PersistentTarget.cpp src/StateMachine.cpp src/TargetFinder.h src/TargetFinder.cpp src/main.cpp)
add_executable(QuadTarget ${SOURCE_FILES})
target_link_libraries( QuadTarget ${OpenCV_LIBS} )