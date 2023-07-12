cmake_minimum_required(VERSION 3.10)
project(drake_test)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(drake CONFIG REQUIRED)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# find_package(GUROBI REQUIRED)
# include_directories(${GUROBI_INCLUDE_DIRS})

add_executable(franka_sim src/franka_sim.cpp)
target_link_libraries(franka_sim drake::drake)

add_executable(opt_test src/opt_test.cpp)
target_link_libraries(opt_test drake::drake)

# target_link_libraries(drake_test drake::drake ${GUROBI_LIBRARY})

