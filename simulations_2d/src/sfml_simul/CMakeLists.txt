cmake_minimum_required(VERSION 3.1)

project(SFML_Project)

set(CMAKE_CXX_STANDARD 11)

find_package(SFML 2.5 COMPONENTS graphics audio REQUIRED)

add_executable(checkerboard main.cpp)
add_executable(grassfire grassfire.cpp)

target_link_libraries(run PRIVATE sfml-graphics)
