cmake_minimum_required(VERSION 3.12)
project(NS3Simulation)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find NS-3
find_package(ns3 REQUIRED)

# Add your source files
set(SOURCES
    main.cpp
    # Add your other source files here
)

# Create the executable
add_executable(NS3Simulation ${SOURCES})

# Windows with MinGW Installations
if( ${CMAKE_SYSTEM_NAME} MATCHES "Windows" AND MINGW )
    # if working on Windows but not in the lab
    # update the include directory location
    include_directories("Z:/include")
    # update the lib directory location
    target_link_directories(${PROJECT_NAME} PUBLIC "Z:/lib")
# OS X Installations
elseif( APPLE AND ${CMAKE_SYSTEM_NAME} MATCHES "Darwin" )
		# Specify the directory where your libraries are located
		set(LIB_DIR "/usr/local/lib")
    # update the include directory location
    include_directories("/usr/local/include")

		file(GLOB MY_LIBS "${LIB_DIR}/libns3*dylib")
    # update the lib directory location
    target_link_directories(${PROJECT_NAME} PUBLIC "/usr/local/lib")
		target_link_libraries(${PROJECT_NAME} ${MY_LIBS})
# Blanket *nix Installations
elseif( UNIX AND ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
    # update the include directory location
    include_directories("/usr/local/include")
    # update the lib directory location
    target_link_directories(${PROJECT_NAME} PUBLIC "/usr/local/lib")
endif()
