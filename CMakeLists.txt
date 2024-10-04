cmake_minimum_required(VERSION 3.12)
project(NS3Simulation)

# Find NS-3
find_package(ns3 REQUIRED)

# Add your source files
set(SOURCES
    main.cpp
    # Add your other source files here
)

# Create the executable
add_executable(NS3Simulation ${SOURCES})

# Link NS-3 libraries
target_link_libraries(NS3Simulation PRIVATE ns3)

# Set NS-3 include directories
target_include_directories(NS3Simulation PRIVATE ${NS3_INCLUDE_DIRS})