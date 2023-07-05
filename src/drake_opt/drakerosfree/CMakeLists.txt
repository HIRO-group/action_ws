cmake_minimum_required(VERSION 3.10)
project(drake_test)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(drake CONFIG REQUIRED PATHS /opt/drake)

add_executable(drake_test src/test.cpp)
target_link_libraries(drake_test drake::drake)

