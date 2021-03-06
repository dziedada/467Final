cmake_minimum_required (VERSION 3.5)
project(467-final-project)

# Add path to search for modules
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake")

# Set software source dir
set(SOFTWARE_SOURCE_DIR ${PROJECT_SOURCE_DIR})

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS ON)

# Add option to build modules
option(BUILD_VISION "Build the vision system" ON)
option(BUILD_TESTS "Build test cases" OFF)
option(BUILD_HOUGH_CIRCLE "Build test cases" OFF)
option(BUILD_PLANNER "Build planner system" ON)

# Set all compiled libraries to be placed in software/lib
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Set all compiled executables to be placed in software/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Find all globally required libraries
find_package(YAMLCPP REQUIRED)
find_package(ZCM REQUIRED)
find_package(libusb-1.0 REQUIRED)
find_package(LCM REQUIRED)

# Include generated zcm messages and include folder
include_directories(SYSTEM
    ${PROJECT_SOURCE_DIR}/generated
    ${PROJECT_SOURCE_DIR}/include
    ${LIBUSB_1_INCLUDE_DIRS}
)

# Set include directory
set(SW_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

# Set the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Find headers
include(CheckIncludeFiles)

# Set global C++ flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -Wall -Wextra -pedantic -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++17")

# Set global debug flags
# Helps with debugging memory and thread issues
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Add source directory
add_subdirectory(src)
add_subdirectory(drivers)

# Setup testing options
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

message(STATUS "")
message(STATUS "Building a ${CMAKE_BUILD_TYPE} build")
message(STATUS "")
