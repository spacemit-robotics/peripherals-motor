cmake_minimum_required(VERSION 3.10)
set(project "SCServo")
project(${project})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Include directories
include_directories(../include)

# Header files
file(GLOB hdrs ../include/*.h)
# Common protocol sources only; model adapters are built by the motor framework.
file(GLOB srs common/*.cpp)

add_library(${project} STATIC ${hdrs} ${srs})

# Link pthread
target_link_libraries(${project} pthread)

#add_executable(${project} main.cpp ${hdrs} ${srs})
