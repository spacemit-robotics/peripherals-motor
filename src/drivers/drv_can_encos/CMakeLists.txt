cmake_minimum_required(VERSION 3.16)
project(drv_can_encos LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

add_library(encos_driver STATIC
    src/encos_protocol.c
    src/drv_can_encos.c
)
target_include_directories(encos_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
)
target_include_directories(encos_driver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)
target_link_libraries(encos_driver PRIVATE Threads::Threads m)

option(BUILD_TESTS "Build Encos protocol test" ON)
if(BUILD_TESTS)
    add_executable(test_encos_protocol
        test/test_encos_protocol.c
        ../../motor_core.c
    )
    target_include_directories(test_encos_protocol PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
    )
    target_link_libraries(test_encos_protocol PRIVATE
        -Wl,--whole-archive
        encos_driver
        -Wl,--no-whole-archive
        Threads::Threads
        m
    )
endif()
