# Box3D as a dependency, so its top level pthread flags and heavy validation stay off unless asked for
cmake_minimum_required(VERSION 3.22)
project(box3d_web C CXX)

set(BOX3D_SOURCE "$ENV{HOME}/Projects/box3d" CACHE PATH "Upstream Box3D checkout")
option(WEB_THREADS "Build with pthreads and shared memory" OFF)

if(WEB_THREADS)
	string(APPEND CMAKE_C_FLAGS " -pthread")
	string(APPEND CMAKE_CXX_FLAGS " -pthread")
endif()

set(BOX3D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX3D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${BOX3D_SOURCE} box3d EXCLUDE_FROM_ALL)

set(SCENES "${CMAKE_CURRENT_SOURCE_DIR}/../../../verify/scenes")
add_executable(box3d_web driver.c ${SCENES}/scenes.c)
target_include_directories(box3d_web PRIVATE ${SCENES})
target_compile_options(box3d_web PRIVATE -ffp-contract=off)
target_link_libraries(box3d_web PRIVATE box3d)
target_link_options(box3d_web PRIVATE
	-sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=Box3D
	-sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=64MB -sMAXIMUM_MEMORY=4GB
	-sSTACK_SIZE=4MB
	-sEXPORTED_RUNTIME_METHODS=HEAPU8
	-sENVIRONMENT=node,web,worker)
if(WEB_THREADS)
	target_link_options(box3d_web PRIVATE -pthread -sPTHREAD_POOL_SIZE=8)
endif()
