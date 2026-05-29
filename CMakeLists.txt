cmake_minimum_required(VERSION 3.10)
project(motor LANGUAGES C CXX)

# Semicolon-separated list of enabled drivers; when set, only requested drivers are built.
# Example: -DSROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS=drv_can_generic;drv_pwm_generic
set(SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS "" CACHE STRING "Enabled motor drivers (semicolon-separated). Empty means auto-detect.")

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set C++ standard (for Damiao driver)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# pwm motor driver paths
set(PWM_DRIVER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/")

# CAN DM driver path
set(CAN_DM_DRIVER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/drv_can_dm")

set(_pwm_rohs_requested OFF)
if("${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
    set(_pwm_rohs_requested ON)
elseif("drv_pwm_RoHS" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS OR "pwm_RoHS" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS)
    set(_pwm_rohs_requested ON)
endif()

set(_pwm_generic_requested OFF)
if("${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
    set(_pwm_generic_requested ON)
elseif("drv_pwm_generic" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS OR "pwm_generic" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS)
    set(_pwm_generic_requested ON)
endif()

# Decide whether CAN DM is requested:
set(_can_dm_requested OFF)
if("${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
    set(_can_dm_requested ON)
elseif("drv_can_dm" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS OR "can_dm" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS)
    set(_can_dm_requested ON)
endif()

# Feetech driver paths
set(FEETECH_DRIVER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/drv_uart_feetech")
set(PWM_DRIVER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers")

# Feetech timestamp debug option
option(FEETECH_TIMESTAMP_DEBUG "Enable timestamp debug prints in feetech driver" OFF)

# Decide whether Feetech is requested:
set(_feetech_requested OFF)
if("${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
    set(_feetech_requested ON)
elseif("drv_uart_feetech" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS OR "feetech" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS)
    set(_feetech_requested ON)
endif()

set(BUILD_FEETECH_DRIVER OFF)
if(_feetech_requested)
    if(EXISTS "${FEETECH_DRIVER_PATH}")
        set(BUILD_FEETECH_DRIVER ON)
        message(STATUS "Found Feetech driver at: ${FEETECH_DRIVER_PATH}")

        include_directories(${FEETECH_DRIVER_PATH}/include)
        set(FEETECH_SOURCES
            ${FEETECH_DRIVER_PATH}/src/feetech_motor_adapter.cpp
            ${FEETECH_DRIVER_PATH}/src/feetech_pack.cpp
            ${FEETECH_DRIVER_PATH}/src/SMS_STS.cpp
            ${FEETECH_DRIVER_PATH}/src/SCS.cpp
            ${FEETECH_DRIVER_PATH}/src/SCSCL.cpp
            ${FEETECH_DRIVER_PATH}/src/SCSerial.cpp
            ${FEETECH_DRIVER_PATH}/src/HLSCL.cpp
        )
    elseif(NOT "${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
        # Only warn when user explicitly requested Feetech via enabled drivers list.
        if(NOT EXISTS "${FEETECH_DRIVER_PATH}")
            message(WARNING "Feetech driver requested but not found at: ${FEETECH_DRIVER_PATH}")
        endif()

    endif()
endif()


# Decide whether PWM RoHS is requested (explicit only; empty list = not requested)
set(_pwm_rohs_requested OFF)
if("drv_pwm_RoHS" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS OR "pwm_RoHS" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS)
    set(_pwm_rohs_requested ON)
endif()

set(BUILD_PWM_ROHS_DRIVER OFF)
if(_pwm_rohs_requested)
    if(EXISTS "${PWM_DRIVER_PATH}/drv_pwm_RoHS.c")
        set(BUILD_PWM_ROHS_DRIVER ON)
        set(PWM_ROHS_SOURCES
            ${PWM_DRIVER_PATH}/drv_pwm_RoHS.c
        )
    elseif(NOT "${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
        message(WARNING "PWM motor driver requested but not found at: ${PWM_DRIVER_PATH}")
    endif()
endif()

set(BUILD_PWM_GENERIC_DRIVER OFF)
if(_pwm_generic_requested)
    if(EXISTS "${PWM_DRIVER_PATH}/drv_pwm_generic.c")
        set(BUILD_PWM_GENERIC_DRIVER ON)
        set(PWM_GENERIC_SOURCES
            ${PWM_DRIVER_PATH}/drv_pwm_generic.c
        )
    elseif(NOT "${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
        message(WARNING "PWM motor driver requested but not found at: ${PWM_DRIVER_PATH}")
    endif()
endif()

set(BUILD_CAN_DM_DRIVER OFF)
if(_can_dm_requested)
    if(EXISTS "${CAN_DM_DRIVER_PATH}")
        set(BUILD_CAN_DM_DRIVER ON)
        message(STATUS "Found CAN DM driver at: ${CAN_DM_DRIVER_PATH}")

        include_directories(${CAN_DM_DRIVER_PATH}/include)
        set(CAN_DM_SOURCES
            ${CAN_DM_DRIVER_PATH}/src/drv_can_dm.cpp
            ${CAN_DM_DRIVER_PATH}/src/damiao.cpp
            ${CAN_DM_DRIVER_PATH}/src/damiao_pack.cpp
            ${CAN_DM_DRIVER_PATH}/src/damiao_hw.cpp
            ${CAN_DM_DRIVER_PATH}/src/socketcan.cpp
        )
    elseif(NOT "${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
        # Only warn when user explicitly requested CAN DM via enabled drivers list.
        if(NOT EXISTS "${CAN_DM_DRIVER_PATH}")
            message(WARNING "CAN DM driver requested but not found at: ${CAN_DM_DRIVER_PATH}")
        endif()
    endif()
endif()

# ECAT JMC driver path
set(ECAT_JMC_DRIVER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/drv_ethercat_jmc")

# UART XL330 driver path
set(UART_XL330_DRIVER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/drv_uart_xl330")

# Find ethercat library (IgH EtherCAT Master).
# Rely on CMAKE_PREFIX_PATH (set by build system) for portable resolution.
find_library(ETHERCAT_LIBRARY
    NAMES ethercat libethercat.so.1
)
# Fallback: search for versioned shared library (e.g. libethercat.so.1) in case
# the unversioned symlink libethercat.so was not created by the install.
if(NOT ETHERCAT_LIBRARY)
    set(_saved_suffixes ${CMAKE_FIND_LIBRARY_SUFFIXES})
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".so.1" ".so.2")
    find_library(ETHERCAT_LIBRARY
        NAMES ethercat
        PATHS /usr/local/lib /usr/lib /usr/lib64
    )
    set(CMAKE_FIND_LIBRARY_SUFFIXES ${_saved_suffixes})
endif()
if(ETHERCAT_LIBRARY)
    message(STATUS "Found ethercat library: ${ETHERCAT_LIBRARY}")
else()
    message(STATUS "ethercat library not found")
endif()

# Decide whether ECAT JMC is requested:
set(_ecat_jmc_requested OFF)
if("${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
    set(_ecat_jmc_requested ON)
elseif("drv_ethercat_jmc" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS OR "ethercat_jmc" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS)
    set(_ecat_jmc_requested ON)
endif()

set(BUILD_ECAT_JMC_DRIVER OFF)
if(_ecat_jmc_requested)
    if(EXISTS "${ECAT_JMC_DRIVER_PATH}" AND ETHERCAT_LIBRARY)
        set(BUILD_ECAT_JMC_DRIVER ON)
        message(STATUS "Found ECAT JMC driver at: ${ECAT_JMC_DRIVER_PATH}")

        include_directories(
            ${ECAT_JMC_DRIVER_PATH}/include
        )
        set(ECAT_JMC_SOURCES
            ${ECAT_JMC_DRIVER_PATH}/src/ethercat_cia402.c
            ${ECAT_JMC_DRIVER_PATH}/src/jmc_ihss42.c
            ${ECAT_JMC_DRIVER_PATH}/src/drv_ethercat_jmc_adapter.c
        )
    elseif(EXISTS "${ECAT_JMC_DRIVER_PATH}" AND NOT ETHERCAT_LIBRARY)
        message(WARNING "ECAT JMC driver found but ethercat library is missing, skipping driver")
    elseif(NOT "${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
        if(NOT EXISTS "${ECAT_JMC_DRIVER_PATH}")
            message(WARNING "ECAT JMC driver requested but not found at: ${ECAT_JMC_DRIVER_PATH}")
        endif()
    endif()
endif()

# Decide whether UART XL330 is requested:
set(_uart_xl330_requested OFF)
if("${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
    set(_uart_xl330_requested ON)
elseif("drv_uart_xl330" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS OR "uart_xl330" IN_LIST SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS)
    set(_uart_xl330_requested ON)
endif()

set(BUILD_UART_XL330_DRIVER OFF)
if(_uart_xl330_requested)
    if(EXISTS "${UART_XL330_DRIVER_PATH}")
        set(BUILD_UART_XL330_DRIVER ON)
        message(STATUS "Found UART XL330 driver at: ${UART_XL330_DRIVER_PATH}")
        set(UART_XL330_SOURCES ${UART_XL330_DRIVER_PATH}/src/drv_uart_xl330.c)
        set(RUST_LIB_NAME "reachy_mini_motor_controller")
        # Persistent cargo output directory that survives cmake clean.
        # Default: sibling to cmake build dir. Override via -DRUST_TARGET_DIR=...
        set(RUST_TARGET_DIR "${CMAKE_BINARY_DIR}/../cargo/xl330" CACHE PATH
            "Cargo target directory for XL330 Rust driver (persistent across cmake clean)")
        get_filename_component(RUST_TARGET_DIR "${RUST_TARGET_DIR}" ABSOLUTE)
        set(RUST_LIB_PATH "${RUST_TARGET_DIR}/release/lib${RUST_LIB_NAME}.a")
    elseif(NOT "${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
        if(NOT EXISTS "${UART_XL330_DRIVER_PATH}")
            message(WARNING "UART XL330 driver requested but not found at: ${UART_XL330_DRIVER_PATH}")
        endif()
    endif()
endif()

# Core source files
set(MOTOR_CORE_SOURCES
    src/motor_core.c
)

# Driver source files
set(MOTOR_DRIVER_SOURCES)



# Add Feetech driver if available
if(BUILD_FEETECH_DRIVER)
    list(APPEND MOTOR_DRIVER_SOURCES
        ${FEETECH_SOURCES}
    )
endif()

# Add pwm ROHS motor driver if available
if(BUILD_PWM_ROHS_DRIVER)
    list(APPEND MOTOR_DRIVER_SOURCES
        ${PWM_ROHS_SOURCES}
    )
endif()

## Add pwm generic motor driver if available
if(BUILD_PWM_GENERIC_DRIVER)
   list(APPEND MOTOR_DRIVER_SOURCES
       ${PWM_GENERIC_SOURCES}
   )
endif()
#
# Add CAN DM motor driver if available
if(BUILD_CAN_DM_DRIVER)
    list(APPEND MOTOR_DRIVER_SOURCES
        ${CAN_DM_SOURCES}
    )
    # Suppress warnings for CAN DM driver
    set_source_files_properties(${CAN_DM_SOURCES} PROPERTIES
        COMPILE_FLAGS "-Wno-reorder -Wno-unused-value -Wno-missing-field-initializers -Wno-type-limits"
    )
endif()

# Add ECAT JMC motor driver if available
if(BUILD_ECAT_JMC_DRIVER)
    list(APPEND MOTOR_DRIVER_SOURCES
        ${ECAT_JMC_SOURCES}
    )
endif()

# Add UART XL330 motor driver if available
if(BUILD_UART_XL330_DRIVER)
    list(APPEND MOTOR_DRIVER_SOURCES
        ${UART_XL330_SOURCES}
    )
    # Ensure cargo build is run before motor is built
    # we'll use add_dependencies later on the motor target
endif()

# All source files
set(MOTOR_SOURCES
    ${MOTOR_CORE_SOURCES}
    ${MOTOR_DRIVER_SOURCES}
)

# Create static library
add_library(motor STATIC ${MOTOR_SOURCES})
set_target_properties(motor PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Find libgpiod
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD libgpiod)

if(NOT LIBGPIOD_FOUND)
    message(WARNING "libgpiod not found. Please install it: sudo apt-get install libgpiod-dev")
endif()

# Detect libgpiod API version via compile check.
# Some sysroots report version 2.x but ship v1 headers, so we probe the actual
# API availability rather than trusting the version string alone.
include(CheckCSourceCompiles)
if(LIBGPIOD_FOUND)
    set(CMAKE_REQUIRED_INCLUDES ${LIBGPIOD_INCLUDE_DIRS})
    set(CMAKE_REQUIRED_LIBRARIES ${LIBGPIOD_LIBRARIES})
    check_c_source_compiles("
        #include <gpiod.h>
        int main(void) {
            /* enum gpiod_line_value is v2-only; v1 uses int directly */
            enum gpiod_line_value val = GPIOD_LINE_VALUE_ACTIVE;
            (void)val;
            return 0;
        }
    " LIBGPIOD_HAS_V2_API)
    unset(CMAKE_REQUIRED_INCLUDES)
    unset(CMAKE_REQUIRED_LIBRARIES)

    if(LIBGPIOD_HAS_V2_API)
        add_definitions(-DLIBGPIOD_V2)
        message(STATUS "libgpiod: using v2 API (version ${LIBGPIOD_VERSION})")
    else()
        add_definitions(-DLIBGPIOD_V1)
        message(STATUS "libgpiod: using v1 API (version ${LIBGPIOD_VERSION})")
    endif()
endif()

# Link pthread for Feetech driver (if using threads)
if(BUILD_FEETECH_DRIVER)
    target_link_libraries(motor PUBLIC pthread)
    if(FEETECH_TIMESTAMP_DEBUG)
        target_compile_definitions(motor PRIVATE FEETECH_TIMESTAMP_DEBUG)
    endif()
endif()

# Link pthread for CAN DM driver (if using threads)
if(BUILD_CAN_DM_DRIVER)
    target_link_libraries(motor PUBLIC pthread)
endif()

if(LIBGPIOD_FOUND)
    include_directories(${LIBGPIOD_INCLUDE_DIRS})
endif()

# Link pthread for ECAT JMC driver (if using threads)
if(BUILD_ECAT_JMC_DRIVER)
    target_link_libraries(motor PUBLIC ${ETHERCAT_LIBRARY} pthread)
endif()

# Link libraries to motor (PUBLIC so consumers inherit these deps)
target_link_libraries(motor PUBLIC
    ${LIBGPIOD_LIBRARIES}
    pthread
    dl
    rt
    m
    stdc++
)

# Link Rust core for UART XL330 if built
if(BUILD_UART_XL330_DRIVER)
    add_dependencies(motor uart_xl330_cargo_build)
    # Mark the Rust .a as generated so CMake doesn't complain about missing file at configure time
    set_source_files_properties("${RUST_LIB_PATH}" PROPERTIES GENERATED TRUE)
    # Consumers will link the installed Rust lib via the MOTOR_RUST_LIB variable provided by motorConfig.cmake
    target_link_libraries(motor PUBLIC $<BUILD_INTERFACE:${RUST_LIB_PATH}>)
    # Rust staticlib might need dl and m
    target_link_libraries(motor PUBLIC dl m)
endif()

# Set compiler/linker flags for driver auto-registration
# The __attribute__((used)) in MOTOR_DRIVER_REGISTER ensures symbols are kept
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(motor PRIVATE
        -Wall
        -Wextra
        -Wno-unused-parameter
    )
endif()

# ============================================================================
# Install rules & CMake config export
# ============================================================================

# Set public include directory on the target (for export)
target_include_directories(motor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS motor EXPORT motorTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

if(BUILD_UART_XL330_DRIVER)
    install(FILES "${RUST_LIB_PATH}" DESTINATION lib)
endif()

install(FILES include/motor.h DESTINATION include)

# Export CMake targets
install(EXPORT motorTargets
    FILE motorTargets.cmake
    NAMESPACE components::
    DESTINATION lib/cmake/motor
)

# Generate motorConfig.cmake with usage requirements
include(CMakePackageConfigHelpers)

# Determine extra link flags for consumers (whole-archive + Rust lib)
set(_MOTOR_EXTRA_LINK_LIBS "")
if(BUILD_UART_XL330_DRIVER)
    set(_MOTOR_RUST_LIB "\${CMAKE_INSTALL_PREFIX}/lib/libreachy_mini_motor_controller.a")
    set(_MOTOR_EXTRA_LINK_LIBS "${_MOTOR_RUST_LIB}")
endif()

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/motorConfig.cmake.in" [=[
# motorConfig.cmake — auto-generated
#
# Provides:
#   components::motor           — imported static library
#   motor::motor                — alias for backward compatibility
#   MOTOR_WHOLE_ARCHIVE    — TRUE (consumers must use --whole-archive for driver registration)
#   MOTOR_RUST_LIB         — path to Rust companion lib (empty if not built)
#
# Usage:
#   find_package(motor REQUIRED)
#   target_link_libraries(my_app
#       -Wl,--whole-archive motor::motor -Wl,--no-whole-archive
#       ${MOTOR_RUST_LIB}   # if non-empty
#   )

include("${CMAKE_CURRENT_LIST_DIR}/motorTargets.cmake")

# Ensure the imported target automatically links the Rust companion library
if(NOT "@_MOTOR_EXTRA_LINK_LIBS@" STREQUAL "")
    set_property(TARGET components::motor APPEND PROPERTY INTERFACE_LINK_LIBRARIES "@_MOTOR_EXTRA_LINK_LIBS@")
endif()

# Backward compatibility alias
if(NOT TARGET motor::motor AND TARGET components::motor)
    if(CMAKE_VERSION VERSION_LESS 3.18)
        add_library(motor::motor INTERFACE IMPORTED)
        set_target_properties(motor::motor PROPERTIES INTERFACE_LINK_LIBRARIES components::motor)
    else()
        add_library(motor::motor ALIAS components::motor)
    endif()
endif()

set(MOTOR_WHOLE_ARCHIVE TRUE)
set(MOTOR_RUST_LIB "@_MOTOR_EXTRA_LINK_LIBS@")
]=])

# Replace @vars@ in the config file
configure_file(
    "${CMAKE_CURRENT_BINARY_DIR}/motorConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/motorConfig.cmake"
    @ONLY
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/motorConfig.cmake"
    DESTINATION lib/cmake/motor
)

# Optional: Create a test executable
option(BUILD_TESTS "Build test programs" ON)

if(BUILD_TESTS)
  
    # Add UART motor test (generic, supports feetech etc.) when Feetech driver is built
    if(BUILD_FEETECH_DRIVER)
        add_executable(test_motor_uart tests/test_motor_uart.c)
        set_target_properties(test_motor_uart PROPERTIES LINKER_LANGUAGE CXX BUILD_RPATH "$ORIGIN/../lib")
        target_include_directories(test_motor_uart PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/src
        )
        target_link_libraries(test_motor_uart
            -Wl,--whole-archive
            motor
            -Wl,--no-whole-archive
            pthread
        )
        install(TARGETS test_motor_uart RUNTIME DESTINATION bin)
    endif()

    # Add pwm motor test if driver is built
    if(BUILD_PWM_GENERIC_DRIVER)
        add_executable(test_motor_pwm tests/test_motor_pwm.c)
        set_target_properties(test_motor_pwm PROPERTIES LINKER_LANGUAGE C BUILD_RPATH "$ORIGIN/../lib")
        target_include_directories(test_motor_pwm PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
        target_link_libraries(test_motor_pwm
            -Wl,--whole-archive
            motor
            -Wl,--no-whole-archive
            ${LIBGPIOD_LIBRARIES}
            pthread
        )
        install(TARGETS test_motor_pwm RUNTIME DESTINATION bin)
    endif()
    #
    # Add CAN DM motor test if driver is built
    if(BUILD_CAN_DM_DRIVER)
        add_executable(test_motor_can tests/test_motor_can.c)
        set_target_properties(test_motor_can PROPERTIES LINKER_LANGUAGE CXX BUILD_RPATH "$ORIGIN/../lib")
        target_include_directories(test_motor_can PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${CAN_DM_DRIVER_PATH}/include
        )
        target_link_libraries(test_motor_can
            -Wl,--whole-archive
            motor
            -Wl,--no-whole-archive
            pthread
        )
        install(TARGETS test_motor_can RUNTIME DESTINATION bin)
    endif()
    
    # Add ECAT motor test if driver is built
    if(BUILD_ECAT_JMC_DRIVER)
        add_executable(test_motor_ecat tests/test_motor_ecat.c)
        set_target_properties(test_motor_ecat PROPERTIES LINKER_LANGUAGE CXX BUILD_RPATH "$ORIGIN/../lib")
        target_include_directories(test_motor_ecat PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        )

        target_link_libraries(test_motor_ecat
            -Wl,--whole-archive
            motor
            -Wl,--no-whole-archive
            ${ETHERCAT_LIBRARY}
            pthread
        )
        install(TARGETS test_motor_ecat RUNTIME DESTINATION bin)
    endif()

    # UART XL330 motor test (XL330/XC330)
    if(BUILD_UART_XL330_DRIVER)
        add_executable(test_motor_xl330 tests/test_uart_xl330.c)
        set_target_properties(test_motor_xl330 PROPERTIES LINKER_LANGUAGE CXX BUILD_RPATH "$ORIGIN/../lib")
        target_include_directories(test_motor_xl330 PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/src
        )
        target_link_libraries(test_motor_xl330
            -Wl,--whole-archive
            motor
            -Wl,--no-whole-archive
            pthread
        )
        install(TARGETS test_motor_xl330 RUNTIME DESTINATION bin)
    endif()


    # UART XL330 motor tests moved to src/drivers/drv_uart_xl330/CMakeLists.txt
endif()

if(BUILD_UART_XL330_DRIVER)
    add_subdirectory(src/drivers/drv_uart_xl330)
endif()
