cmake_minimum_required(VERSION 3.16)
project(dm_main)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)



# 添加库
add_library(mylib STATIC
    src/socketcan.cpp
    src/damiao.cpp
    src/DmHW.cpp  
    src/pack_damiao.cpp
    src/drv_can_dm.cpp
)

# 包含目录
target_include_directories(mylib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mylib PRIVATE Threads::Threads)

# # 添加可执行文件
# add_executable(${PROJECT_NAME}
#     src/control_loop.cpp
#     src/main.cpp
# )

# # 链接库到可执行文件
# target_link_libraries(${PROJECT_NAME} PRIVATE
#     mylib 
# )

# 添加测试可执行文件
add_executable(test_dm_motor
    test/test.cpp
)

target_link_libraries(test_dm_motor PRIVATE
    mylib
)
set_target_properties(test_dm_motor PROPERTIES BUILD_RPATH "$ORIGIN/../lib")
install(TARGETS test_dm_motor RUNTIME DESTINATION bin)

# Add driver as OBJECT library to ensure registration constructor is linked
add_library(dm_driver_obj OBJECT
    src/drv_can_dm.cpp
)
target_include_directories(dm_driver_obj PRIVATE
    ../../../include
    ../..
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 添加 motor_framework 集成测试
# 注意：这里假设 motor_framework 的核心包含文件和库在上一级或标准路径
# 为了演示，我们需要编译 drv_can_dm.cpp
add_executable(test_dm_framework
    test/test_dm_framework.cpp
    ../../motor_core.c
    # ../../motor_can.c
    $<TARGET_OBJECTS:dm_driver_obj>
)

# 包含 motor_framework 头文件路径
target_include_directories(test_dm_framework PRIVATE
    ../../../include
    ../.. # src/drivers/.. -> src/
)

target_link_libraries(test_dm_framework PRIVATE
    mylib
)
set_target_properties(test_dm_framework PROPERTIES BUILD_RPATH "$ORIGIN/../lib")
install(TARGETS test_dm_framework RUNTIME DESTINATION bin)


