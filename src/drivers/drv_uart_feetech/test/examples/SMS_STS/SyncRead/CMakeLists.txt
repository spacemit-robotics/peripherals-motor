cmake_minimum_required(VERSION 2.8.3)
set(project "SyncRead")
project(${project})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Include directories
include_directories(../../../../include)

# Link directories (where libSCServo.a is located in build directory)
link_directories(../../../../src/build)

# Example source files
file(GLOB hdrs *.h)
file(GLOB srs *.cpp)

add_executable(${project} ${hdrs} ${srs})

# Link static library and pthread
target_link_libraries(${project} SCServo pthread)
