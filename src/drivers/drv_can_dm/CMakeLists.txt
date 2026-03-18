cmake_minimum_required(VERSION 3.16)
project(dm_main)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

# ========== 驱动库（基于 DamiaoHW + damiao_pack） ==========
add_library(mylib STATIC
    src/socketcan.cpp
    src/damiao.cpp
    src/damiao_hw.cpp
    src/damiao_pack.cpp
    src/drv_can_dm.cpp
)

target_include_directories(mylib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mylib PRIVATE Threads::Threads)

# ========== 驱动 OBJECT 库 ==========
add_library(dm1_driver_obj OBJECT
    src/drv_can_dm.cpp
)
target_include_directories(dm1_driver_obj PRIVATE
    ../../../include
    ../..
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ========== 模式切换测试 ==========
add_executable(test_dm_mode_switch
    test/test_dm_mode_switch.cpp
    ../../motor_core.c
    $<TARGET_OBJECTS:dm1_driver_obj>
)

target_include_directories(test_dm_mode_switch PRIVATE
    ../../../include
    ../..
)

target_link_libraries(test_dm_mode_switch PRIVATE
    mylib
)
set_target_properties(test_dm_mode_switch PROPERTIES BUILD_RPATH "$ORIGIN/../lib")
install(TARGETS test_dm_mode_switch RUNTIME DESTINATION bin)
