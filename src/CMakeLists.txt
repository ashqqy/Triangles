cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(triangles_lib)

include_directories(${CMAKE_SOURCE_DIR}/include)
add_library(${PROJECT_NAME} triangles.cpp)
