cmake_minimum_required (VERSION 3.10)
project (Triangles)

add_subdirectory(src)
add_subdirectory(tests)
include_directories(${CMAKE_SOURCE_DIR}/include)
add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} triangles_lib)
