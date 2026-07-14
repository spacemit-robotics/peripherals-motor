# drv_uart_xl330/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(drv_uart_xl330 LANGUAGES C CXX)

# Options
option(BUILD_TESTS "Build test programs" ON)

# Integration with Cargo
set(RUST_LIB_NAME "reachy_mini_motor_controller")
# Place cargo build output in a persistent directory outside the cmake build tree.
# This ensures incremental Rust builds remain fast even after cmake clean/reconfigure.
# The parent CMakeLists.txt sets RUST_TARGET_DIR; fall back for standalone builds.
if(NOT DEFINED RUST_TARGET_DIR)
    set(RUST_TARGET_DIR "${CMAKE_INSTALL_PREFIX}/../build/cargo/xl330")
    get_filename_component(RUST_TARGET_DIR "${RUST_TARGET_DIR}" ABSOLUTE)
endif()
set(CARGO_EXECUTABLE "" CACHE FILEPATH "Cargo executable")
set(SROBOTIS_CARGO_HOME "" CACHE PATH "Cargo home used for SDK builds")
set(SROBOTIS_CARGO_RUSTC "" CACHE FILEPATH "Rust compiler used by Cargo")
set(SROBOTIS_CARGO_TARGET "" CACHE STRING "Cargo target triple")
set(SROBOTIS_CARGO_TARGET_LINKER "" CACHE FILEPATH "Cargo target linker")
set(SROBOTIS_CARGO_TARGET_AR "" CACHE FILEPATH "Cargo target archiver")
set(SROBOTIS_RUST_SYSROOT "" CACHE PATH "Rust sysroot containing target rustlib")
set(SROBOTIS_CARGO_LINK_SYSROOT "" CACHE PATH "Target sysroot passed to the Cargo linker")

if(NOT CARGO_EXECUTABLE OR NOT IS_ABSOLUTE "${CARGO_EXECUTABLE}")
    set(_srobotis_cargo_names)
    if(CARGO_EXECUTABLE)
        list(APPEND _srobotis_cargo_names "${CARGO_EXECUTABLE}")
    endif()
    list(APPEND _srobotis_cargo_names cargo cargo-1.91)
    unset(_SROBOTIS_FOUND_CARGO CACHE)
    find_program(_SROBOTIS_FOUND_CARGO NAMES ${_srobotis_cargo_names})
    if(_SROBOTIS_FOUND_CARGO)
        set(CARGO_EXECUTABLE "${_SROBOTIS_FOUND_CARGO}" CACHE FILEPATH "Cargo executable" FORCE)
    endif()
endif()
if(NOT CARGO_EXECUTABLE)
    message(FATAL_ERROR "Cargo executable not found. Install cargo/cargo-1.91 or pass -DCARGO_EXECUTABLE=/path/to/cargo.")
endif()

if(NOT SROBOTIS_CARGO_RUSTC OR NOT IS_ABSOLUTE "${SROBOTIS_CARGO_RUSTC}")
    set(_srobotis_rustc_names)
    if(SROBOTIS_CARGO_RUSTC)
        list(APPEND _srobotis_rustc_names "${SROBOTIS_CARGO_RUSTC}")
    endif()
    list(APPEND _srobotis_rustc_names rustc rustc-1.91)
    unset(_SROBOTIS_FOUND_RUSTC CACHE)
    find_program(_SROBOTIS_FOUND_RUSTC NAMES ${_srobotis_rustc_names})
    if(_SROBOTIS_FOUND_RUSTC)
        set(SROBOTIS_CARGO_RUSTC "${_SROBOTIS_FOUND_RUSTC}" CACHE FILEPATH "Rust compiler used by Cargo" FORCE)
    endif()
endif()

if(SROBOTIS_CARGO_TARGET)
    set(RUST_LIB_PATH "${RUST_TARGET_DIR}/${SROBOTIS_CARGO_TARGET}/release/lib${RUST_LIB_NAME}.a")
else()
    set(RUST_LIB_PATH "${RUST_TARGET_DIR}/release/lib${RUST_LIB_NAME}.a")
endif()

# Collect Rust source files so CMake re-triggers cargo when they change.
# Cargo itself handles incremental compilation — only your crate is rebuilt,
# not the 50+ dependency crates (those are cached in RUST_TARGET_DIR).
#
# NOTE: GLOB_RECURSE only runs at cmake configure time. If you add new .rs files,
# re-run cmake (or just run mm which does it automatically).
file(GLOB_RECURSE RUST_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.rs"
    "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml"
    "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.lock"
)

# Use a stamp file so that cargo is always invoked when sources change.
# Cargo handles its own fine-grained incrementality internally (~0.1s no-op).
set(RUST_STAMP_FILE "${RUST_TARGET_DIR}/.build_stamp")

set(_cargo_env CARGO_INCREMENTAL=1)
if(SROBOTIS_CARGO_HOME)
    list(APPEND _cargo_env "CARGO_HOME=${SROBOTIS_CARGO_HOME}")
endif()
if(SROBOTIS_CARGO_RUSTC)
    list(APPEND _cargo_env "RUSTC=${SROBOTIS_CARGO_RUSTC}")
endif()
if(SROBOTIS_CARGO_TARGET)
    string(TOUPPER "${SROBOTIS_CARGO_TARGET}" _cargo_target_env)
    string(REPLACE "-" "_" _cargo_target_env "${_cargo_target_env}")
    if(SROBOTIS_CARGO_TARGET_LINKER)
        list(APPEND _cargo_env "CARGO_TARGET_${_cargo_target_env}_LINKER=${SROBOTIS_CARGO_TARGET_LINKER}")
    endif()
    if(SROBOTIS_CARGO_TARGET_AR)
        list(APPEND _cargo_env "CARGO_TARGET_${_cargo_target_env}_AR=${SROBOTIS_CARGO_TARGET_AR}")
    endif()
endif()
set(_cargo_rustflags "")
if(DEFINED ENV{RUSTFLAGS} AND NOT "$ENV{RUSTFLAGS}" STREQUAL "")
    list(APPEND _cargo_rustflags "$ENV{RUSTFLAGS}")
endif()
if(SROBOTIS_RUST_SYSROOT)
    list(APPEND _cargo_rustflags "--sysroot=${SROBOTIS_RUST_SYSROOT}")
endif()
if(SROBOTIS_CARGO_LINK_SYSROOT)
    list(APPEND _cargo_rustflags "-C" "link-arg=--sysroot=${SROBOTIS_CARGO_LINK_SYSROOT}")
endif()
if(_cargo_rustflags)
    string(JOIN " " _cargo_rustflags_text ${_cargo_rustflags})
    list(APPEND _cargo_env "RUSTFLAGS=${_cargo_rustflags_text}")
endif()

set(_cargo_args build --release --target-dir "${RUST_TARGET_DIR}")
if(SROBOTIS_CARGO_TARGET)
    list(APPEND _cargo_args --target "${SROBOTIS_CARGO_TARGET}")
endif()

add_custom_command(
    OUTPUT "${RUST_STAMP_FILE}"
    # CARGO_INCREMENTAL=1: enable incremental compilation even in release mode.
    # This dramatically speeds up rebuilds when only a few functions change
    # (seconds instead of recompiling the entire crate from scratch).
    COMMAND ${CMAKE_COMMAND} -E env ${_cargo_env}
            "${CARGO_EXECUTABLE}" ${_cargo_args}
    COMMAND ${CMAKE_COMMAND} -E touch "${RUST_STAMP_FILE}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    DEPENDS ${RUST_SOURCES}
    COMMENT "Building Rust core for UART XL330 driver via Cargo (incremental, output: ${RUST_TARGET_DIR})"
    VERBATIM
)

# Add a custom target that depends on the stamp file.
# The actual .a library is a byproduct — cargo updates it only when content changes.
add_custom_target(uart_xl330_cargo_build DEPENDS "${RUST_STAMP_FILE}")

# Tell CMake the .a file is produced as a byproduct of the stamp command,
# so downstream targets can link against it without a circular dependency.
set_source_files_properties("${RUST_LIB_PATH}" PROPERTIES GENERATED TRUE)

# Define source files for this driver
set(UART_XL330_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drv_uart_xl330.c
)

# Create a static library for the driver
# When part of a parent project, this library will be linked into 'motor'
add_library(uart_xl330 STATIC ${UART_XL330_SOURCES})
add_dependencies(uart_xl330 uart_xl330_cargo_build)

target_include_directories(uart_xl330 PUBLIC 
    src 
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
)
target_link_libraries(uart_xl330 PRIVATE "${RUST_LIB_PATH}" dl m pthread)

# RISC-V and other platforms often need -fPIC for shared/position-independent usage
set_target_properties(uart_xl330 PROPERTIES POSITION_INDEPENDENT_CODE ON)

# If this is a standalone build, we need to include the motor core logic
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Standalone build: adding motor_core.c to uart_xl330 library.")
    target_sources(uart_xl330 PRIVATE ../../../src/motor_core.c)
else()
    # Pass the target up if this is a subdirectory
    set(UART_XL330_SOURCES ${UART_XL330_SOURCES} PARENT_SCOPE)
    set(RUST_LIB_PATH ${RUST_LIB_PATH} PARENT_SCOPE)
endif()

# Add UART XL330 motor tests
if(BUILD_TESTS)
    # Note: These tests depend on 'motor.h'. 
    # In standalone mode, we assume include paths are relative to the project root.
    
    # Check if 'motor' target exists (it will if add_subdirectory is used)
    # If not, we might need a dummy or skip parts that use motor_core
    set(MOTOR_LIB motor)
    if(NOT TARGET motor)
        # Standalone mode: we might just link against uart_xl330 and hope for the best
        # Or you might need to add motor core sources here if they are tiny
        message(STATUS "Standalone build: 'motor' target not found, linking against 'uart_xl330' only.")
        set(MOTOR_LIB uart_xl330)
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
