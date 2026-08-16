cmake_minimum_required(VERSION 3.16)
project(drv_can_agibot LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(agibot_driver STATIC
    src/agibot_protocol.c
    src/drv_can_agibot.c
)
target_include_directories(agibot_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
)
target_include_directories(agibot_driver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

option(BUILD_TESTS "Build AGIBOT protocol test" ON)
if(BUILD_TESTS)
    add_executable(test_agibot_protocol
        test/test_agibot_protocol.c
        ../../motor_core.c
    )
    target_include_directories(test_agibot_protocol PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
    )
    target_link_libraries(test_agibot_protocol PRIVATE
        -Wl,--whole-archive
        agibot_driver
        -Wl,--no-whole-archive
        m
    )
endif()
