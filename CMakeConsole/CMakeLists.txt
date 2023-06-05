# CMakeList.txt : CMake project for CMakeConsole, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

#includes

file(GLOB FILES_SRC
    "*.cpp"
    "*.h"
)
set(SOLUTIONTITLE CMakeConsole CACHE TYPE STRING)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 /bigobj /EHa")
set(VVRFRAMEWORK_DIR "${CMAKE_SOURCE_DIR}/CMakeConsole/VVRFramework")

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${VVRFRAMEWORK_DIR}/include)
include_directories(${VVRFRAMEWORK_DIR}/include/VVRScene)
include_directories(${VVRFRAMEWORK_DIR}/include/GeoLib)
include_directories(${VVRFRAMEWORK_DIR}/include/MathGeoLib)

# Add source to this project's executable.
add_executable (CMakeConsole "CMakeConsole.cpp" "CMakeConsole.h")

# TODO: Add tests and install targets if needed.
install (TARGETS  CMakeConsole DESTINATION "cmakeconsole/bin")


target_link_libraries(
${SOLUTIONTITLE} 
${VVRFRAMEWORK_DIR}/lib/VVRScene_d.lib 
${VVRFRAMEWORK_DIR}/lib/GeoLib_d.lib 
${VVRFRAMEWORK_DIR}/lib/MathGeoLib_d.lib
)

add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_DIR}/lib" "${CMAKE_SOURCE_DIR}/out/install/x86-Debug/cmakeconsole/bin")
#add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_DIR}/GeoLib/$<CONFIG>" ${CMAKE_BINARY_DIR}/$<CONFIG>)
#add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_DIR}/MathGeoLib/$<CONFIG>" ${CMAKE_BINARY_DIR}/$<CONFIG>)

#include_directories(${CMAKE_SOURCE_DIR})
#include_directories(${VVRFRAMEWORK_DIR})
#include_directories(${VVRFRAMEWORK_DIR}/GeoLib)
#include_directories(${VVRFRAMEWORK_DIR}/MathGeoLib/src)
#add_executable(${SOLUTIONTITLE} ${FILES_SRC})
#target_link_libraries(
#${SOLUTIONTITLE} 
#${VVRFRAMEWORK_LIBRARY_DIR}/VVRScene/Debug/VVRScene_d.lib 
#${VVRFRAMEWORK_LIBRARY_DIR}/GeoLib/Debug/GeoLib_d.lib 
#${VVRFRAMEWORK_LIBRARY_DIR}/MathGeoLib/Debug/MathGeoLib_d.lib
#)
#add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_LIBRARY_DIR}/VVRScene/$<CONFIG>" ${CMAKE_BINARY_DIR}/$<CONFIG>)
#add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_LIBRARY_DIR}/GeoLib/$<CONFIG>" ${CMAKE_BINARY_DIR}/$<CONFIG>)
#add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_LIBRARY_DIR}/MathGeoLib/$<CONFIG>" ${CMAKE_BINARY_DIR}/$<CONFIG>)