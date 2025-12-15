cmake_minimum_required(VERSION 3.10)

project(3DViewer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(spdlog REQUIRED)
find_package(SDL2 REQUIRED)
find_package(SDL2_IMAGE REQUIRED)
find_package(Eigen3 REQUIRED)

#include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include ${SDL2_TTF_INCLUDE_DIRS})

add_library               (external.sdl2 INTERFACE)
target_include_directories(external.sdl2 INTERFACE ${SDL2_INCLUDE_DIRS} ${SDL2_IMAGE_INCLUDE_DIRS})
target_link_libraries     (external.sdl2 INTERFACE ${SDL2_LIBRARIES} ${SDL2_IMAGE_LIBRARIES})

add_subdirectory(${CMAKE_SOURCE_DIR}/src)