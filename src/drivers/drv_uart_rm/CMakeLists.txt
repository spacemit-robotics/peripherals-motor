# drv_uart_rm/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(drv_uart_rm LANGUAGES C CXX)

# Options
option(BUILD_TESTS "Build test programs" ON)

# Integration with Cargo
set(RUST_LIB_NAME "reachy_mini_motor_controller")
set(RUST_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/target/release/lib${RUST_LIB_NAME}.a")

# Add a custom command to run cargo build
add_custom_command(
    OUTPUT "${RUST_LIB_PATH}"
    COMMAND cargo build --release
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Building Rust core for UART RM driver via Cargo"
    VERBATIM
)

# Add a custom target that depends on the library file
add_custom_target(uart_rm_cargo_build DEPENDS "${RUST_LIB_PATH}")

# Define source files for this driver
set(UART_RM_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drv_uart_xl330.c
)

# Create a static library for the driver
# When part of a parent project, this library will be linked into 'motor'
add_library(uart_rm STATIC ${UART_RM_SOURCES})
add_dependencies(uart_rm uart_rm_cargo_build)

target_include_directories(uart_rm PUBLIC 
    src 
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
)
target_link_libraries(uart_rm PRIVATE "${RUST_LIB_PATH}" dl m pthread)

# RISC-V and other platforms often need -fPIC for shared/position-independent usage
set_target_properties(uart_rm PROPERTIES POSITION_INDEPENDENT_CODE ON)

# If this is a standalone build, we need to include the motor core logic
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Standalone build: adding motor_core.c to uart_rm library.")
    target_sources(uart_rm PRIVATE ../../../src/motor_core.c)
else()
    # Pass the target up if this is a subdirectory
    set(UART_RM_SOURCES ${UART_RM_SOURCES} PARENT_SCOPE)
    set(RUST_LIB_PATH ${RUST_LIB_PATH} PARENT_SCOPE)
endif()

# Add UART RM motor tests
if(BUILD_TESTS)
    # Note: These tests depend on 'motor.h'. 
    # In standalone mode, we assume include paths are relative to the project root.
    
    # Check if 'motor' target exists (it will if add_subdirectory is used)
    # If not, we might need a dummy or skip parts that use motor_core
    set(MOTOR_LIB motor)
    if(NOT TARGET motor)
        # Standalone mode: we might just link against uart_rm and hope for the best
        # Or you might need to add motor core sources here if they are tiny
        message(STATUS "Standalone build: 'motor' target not found, linking against 'uart_rm' only.")
        set(MOTOR_LIB uart_rm)
    endif()

    # test_uart_xl330 (Driver level sine-wave test for XL330)
    add_executable(test_uart_xl330 test/test_uart_xl330.c)
    set_target_properties(test_uart_xl330 PROPERTIES LINKER_LANGUAGE CXX)
    target_include_directories(test_uart_xl330 PRIVATE 
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
    )
    target_link_libraries(test_uart_xl330
        -Wl,--whole-archive
        ${MOTOR_LIB}
        -Wl,--no-whole-archive
        pthread
        m
    )

    install(TARGETS test_uart_xl330 RUNTIME DESTINATION bin)
endif()