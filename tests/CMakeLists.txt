cmake_minimum_required(VERSION 3.12)
project(NS3SimulationTests)

set(CMAKE_CXX_STANDARD 20)

find_package(Catch2 3 REQUIRED)

set(
    TEST_SOURCES
    entity_test
)

include_directories(${CMAKE_SOURCE_DIR}/)

foreach(name ${TEST_SOURCES})
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} Catch2::Catch2WithMain NS3Simulation)
endforeach()
