# Project
project(bba)

# Requirements and flags
cmake_minimum_required( VERSION 3.0 )
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Directory of header files 
include_directories(${PROJECT_SOURCE_DIR}/src)

# Add directory of executable
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
message(STATUS "added ${PROJECT_SOURCE_DIR}/src subdirectory")
