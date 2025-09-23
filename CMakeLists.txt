cmake_minimum_required (VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project (Triangles)
add_subdirectory(tests)

set(SOURCE_FILES
src/main.cpp
src/triangles.cpp)

include_directories(${CMAKE_SOURCE_DIR}/include)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
