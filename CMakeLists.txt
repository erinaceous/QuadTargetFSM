cmake_minimum_required(VERSION 3.0)
project(QuadTargetFSM)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Doxygen)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CURRENT_SOURCE_DIR src)
include_directories(src/include)

file(GLOB INCLUDE_FILES src/include/*.h*)
file(GLOB SRC_FILES src/*.c*)

if(DOXYGEN_FOUND)
configure_file(src/Doxyfile.in
${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc)
add_custom_target(doc
${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)

add_executable(QuadTarget ${INCLUDE_FILES} ${SRC_FILES})
target_link_libraries(QuadTarget ${OpenCV_LIBS})

