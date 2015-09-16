cmake_minimum_required(VERSION 3.0)
project(QuadTargetFSM)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Doxygen)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CURRENT_SOURCE_DIR src)

include_directories(src/include)
include_directories(src/utils)
file(GLOB INCLUDE_FILES src/include/*.h* src/utils/*.h*)
file(GLOB SRC_FILES src/utils/*.c* src/*.c*)

if(DOXYGEN_FOUND)
configure_file(Doxyfile.in
${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc)
add_custom_target(doc
${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
COMMAND sed -i s/^\\s*refman\\.idx// ${CMAKE_CURRENT_BINARY_DIR}/doc/latex/Makefile
COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/doc/latex
COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/doc/latex/refman.pdf ${CMAKE_CURRENT_BINARY_DIR}/QuadTargetFSM.pdf
WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)

set(EXTRA_LIBS "")
set(EXTRA_FILES "")
option(PI_BUILD "PI_BUILD" OFF)
if(PI_BUILD)
    include_directories(picam)
    file(GLOB EXTRA_FILES picam/*.c* picam/*.h*)
    include_directories(/opt/vc/include)
    include_directories(/opt/vc/userland-master/interface/vcos)
    include_directories(/opt/vc/userland-master)
    include_directories(/opt/vc/userland-master/interface/vcos/pthreads)
    include_directories(/opt/vc/userland-master/interface/vmcs_host/linux)
    include_directories(/opt/vc/userland-master/host_applications/linux/libs/bcm_host/include)
    link_directories(/opt/vc/lib)
    set(EXTRA_LIBS "libmmal_core.so libmmal_util.so libmmal_vc_client.so libvcos.so libbcm_host.so GLESv2 EGL")
    add_definitions(-DPI_BUILD)
endif(PI_BUILD)

add_executable(QuadTarget ${EXTRA_FILES} ${INCLUDE_FILES} ${SRC_FILES})
target_link_libraries(QuadTarget ${EXTRA_LIBS} ${OpenCV_LIBS})

