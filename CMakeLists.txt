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
            ${CAN_DM_DRIVER_PATH}/src/pack_damiao.cpp
            ${CAN_DM_DRIVER_PATH}/src/control_loop.cpp
            ${CAN_DM_DRIVER_PATH}/src/DmHW.cpp
            ${CAN_DM_DRIVER_PATH}/src/socketcan.cpp
        )
    elseif(NOT "${SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS}" STREQUAL "")
        # Only warn when user explicitly requested CAN DM via enabled drivers list.
        if(NOT EXISTS "${CAN_DM_DRIVER_PATH}")
            message(WARNING "CAN DM driver requested but not found at: ${CAN_DM_DRIVER_PATH}")
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

# All source files
set(MOTOR_SOURCES
    ${MOTOR_CORE_SOURCES}
    ${MOTOR_DRIVER_SOURCES}
)

# Create static library
add_library(motor STATIC ${MOTOR_SOURCES})

# Find libgpiod
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD libgpiod)

if(NOT LIBGPIOD_FOUND)
    message(WARNING "libgpiod not found. Please install it: sudo apt-get install libgpiod-dev")
endif()

# 获取版本号并传递给编译器
if(LIBGPIOD_VERSION VERSION_LESS "2.0.0")
    add_definitions(-DLIBGPIOD_V1)
else()
    add_definitions(-DLIBGPIOD_V2)
endif()

# Link pthread for Feetech driver (if using threads)
if(BUILD_FEETECH_DRIVER)
    target_link_libraries(motor PRIVATE pthread)
endif()

# Link pthread for CAN DM driver (if using threads)
if(BUILD_CAN_DM_DRIVER)
    target_link_libraries(motor PRIVATE pthread)
endif()

if(LIBGPIOD_FOUND)
    include_directories(${LIBGPIOD_INCLUDE_DIRS})
endif()

# Link libraries to motor
target_link_libraries(motor PRIVATE
    ${LIBGPIOD_LIBRARIES}
    pthread  # 只需要链接一次
)

# Set compiler/linker flags for driver auto-registration
# The __attribute__((used)) in MOTOR_DRIVER_REGISTER ensures symbols are kept
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(motor PRIVATE
        -Wall
        -Wextra
        -Wno-unused-parameter
    )
endif()

# Install rules
install(TARGETS motor
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES include/motor.h
    DESTINATION include
)

# Optional: Create a test executable
option(BUILD_TESTS "Build test programs" ON)

if(BUILD_TESTS)
  
    # Add UART motor test (generic, supports feetech etc.) when Feetech driver is built
    if(BUILD_FEETECH_DRIVER)
        add_executable(test_motor_uart test/test_motor_uart.c)
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
        add_executable(test_motor_pwm test/test_motor_pwm.c)
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
        add_executable(test_motor_can test/test_motor_can.c)
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
endif()
