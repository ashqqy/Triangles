cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tests)

set(SOURCE_FILES
tests.cpp
${CMAKE_SOURCE_DIR}/src/triangles.cpp)

include_directories(${CMAKE_SOURCE_DIR}/include)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries (${PROJECT_NAME} PUBLIC
    gtest
    gtest_main
)
