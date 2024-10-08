cmake_minimum_required(VERSION 3.12)
project(NS3Simulation)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find NS-3
find_package(ns3 REQUIRED)

# Add your source files
set(SOURCES
    src/entity.cpp
    src/visualizations/sfml_map_node_visualizer.cpp

    src/utils/geo_utils.cpp
    # Add your other source files here
)

include_directories(${CMAKE_SOURCE_DIR}/src)

include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.x
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

add_library(${PROJECT_NAME} ${SOURCES})

# Create the executable
add_executable(NS3SimulationExe main.cpp ${SOURCES})

# Windows with MinGW Installations
if( ${CMAKE_SYSTEM_NAME} MATCHES "Windows" AND MINGW )
    # if working on Windows but not in the lab
    # update the include directory location
    include_directories("Z:/include")
    # update the lib directory location
    target_link_directories(${PROJECT_NAME} PUBLIC "Z:/lib")
    target_link_directories(NS3SimulationExe PUBLIC "Z:/lib")
# OS X Installations
elseif( APPLE AND ${CMAKE_SYSTEM_NAME} MATCHES "Darwin" )
		# Specify the directory where your libraries are located
		set(LIB_DIR "/usr/local/lib")
    # update the include directory location
    include_directories("/usr/local/include")

		file(GLOB MY_LIBS "${LIB_DIR}/libns3*dylib")
    # update the lib directory location
    target_link_directories(${PROJECT_NAME} PUBLIC "/usr/local/lib")
		target_link_libraries(${PROJECT_NAME} PUBLIC ${MY_LIBS} PRIVATE sfml-graphics)

    target_link_directories(NS3SimulationExe PUBLIC "/usr/local/lib")
    target_link_libraries(NS3SimulationExe PUBLIC ${MY_LIBS} PRIVATE sfml-graphics)

    target_link_options(NS3SimulationExe PRIVATE LINKER:-sectcreate,__TEXT,__info_plist,${CMAKE_CURRENT_SOURCE_DIR}/Info.plist)
# Blanket *nix Installations
elseif( UNIX AND ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
    # update the include directory location
    include_directories("/usr/local/include")
    # update the lib directory location
    target_link_directories(${PROJECT_NAME} PUBLIC "/usr/local/lib")
    target_link_directories(NS3SimulationExe PUBLIC "/usr/local/lib")
endif()

# Also build tests
add_subdirectory(tests)