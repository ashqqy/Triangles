cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(generator)

set(SOURCE_FILES
generator.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
