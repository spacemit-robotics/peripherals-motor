cmake_minimum_required(VERSION 3.10)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(drv_ethercat_jmc C)
endif()

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
    /opt/etherlab/include
)

# Source files
set(DRV_ETHERCAT_JMC_SOURCES
    src/ethercat_cia402.c
    src/jmc_ihss42.c
)

# Create static library
add_library(drv_ethercat_jmc STATIC ${DRV_ETHERCAT_JMC_SOURCES})

# Attempt to find the IgH EtherCAT library (libethercat)
find_library(ETHERCAT_LIB ethercat HINTS /opt/etherlab/lib)
if(ETHERCAT_LIB)
    target_link_libraries(drv_ethercat_jmc PRIVATE ${ETHERCAT_LIB} pthread)
    message(STATUS "Found EtherCAT library: ${ETHERCAT_LIB}")
else()
    target_link_libraries(drv_ethercat_jmc PRIVATE pthread ethercat)
    message(WARNING "EtherCAT library not found via find_library, will attempt to link -lethercat anyway")
endif()

# Set link directories to find the ethercat library
link_directories(/opt/etherlab/lib)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(drv_ethercat_jmc PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# Build test executable and output to build directory
add_executable(test_ethercat_jmc test/test_ethercat_jmc.c)
target_link_libraries(test_ethercat_jmc PRIVATE drv_ethercat_jmc pthread)
set_target_properties(test_ethercat_jmc PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
