cmake_minimum_required(VERSION 3.10)
project(feetech LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to enable debug tools
option(DEBUG "Enable debug tools compilation" OFF)
option(FEETECH_TIMESTAMP_DEBUG "Enable timestamp debug prints in feetech driver" OFF)

# Shared protocol sources; model adapters remain directly under src/.
file(GLOB FEETECH_HDRS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
file(GLOB FEETECH_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/common/*.cpp")


# Motor framework sources for integration tests
set(MOTOR_FRAMEWORK_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../motor_core.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/feetech_sts3215_adapter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/feetech_hd1910_adapter.cpp
)

# test_simple executable (motor framework integration test)
add_executable(test_motor_uart
  test/test_simple.c
  ${FEETECH_SRCS}
  ${MOTOR_FRAMEWORK_SRCS}
)

target_include_directories(test_motor_uart PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../include  # motor.h framework
  ${CMAKE_CURRENT_SOURCE_DIR}/../../            # motor_core.h
)
target_link_libraries(test_motor_uart pthread)

# Hardware smoke test for the separately registered HD1910 adapter.
add_executable(test_hd1910_smoke
  test/test_hd1910_smoke.c
  ${FEETECH_SRCS}
  ${MOTOR_FRAMEWORK_SRCS}
)
target_include_directories(test_hd1910_smoke PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../
)
set_target_properties(test_hd1910_smoke PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(test_hd1910_smoke pthread)

# Synchronous position and feedback test for HD1910 hardware.
add_executable(test_hd1910_feedback
  test/test_hd1910_feedback.c
  ${FEETECH_SRCS}
  ${MOTOR_FRAMEWORK_SRCS}
)
target_include_directories(test_hd1910_feedback PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../
)
set_target_properties(test_hd1910_feedback PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(test_hd1910_feedback pthread)

# Small physical movement test for HD1910 hardware.
add_executable(test_hd1910_small_move
  test/test_hd1910_small_move.c
  ${FEETECH_SRCS}
  ${MOTOR_FRAMEWORK_SRCS}
)
target_include_directories(test_hd1910_small_move PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../
)
set_target_properties(test_hd1910_small_move PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(test_hd1910_small_move pthread)

if(FEETECH_TIMESTAMP_DEBUG)
  target_compile_definitions(test_motor_uart PRIVATE FEETECH_TIMESTAMP_DEBUG)
endif()



# test_debug executable (communication debugging tool)
if(DEBUG)
  add_executable(test_debug
    test/test_debug.cpp
    ${FEETECH_SRCS}
  )

  target_include_directories(test_debug PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_debug pthread)
endif()


# Install (optional)
install(TARGETS test_motor_uart RUNTIME DESTINATION bin)
install(TARGETS test_hd1910_smoke RUNTIME DESTINATION bin)
install(TARGETS test_hd1910_feedback RUNTIME DESTINATION bin)
install(TARGETS test_hd1910_small_move RUNTIME DESTINATION bin)
if(DEBUG)
  install(TARGETS test_debug RUNTIME DESTINATION bin)
endif()
install(DIRECTORY include/ DESTINATION include/feetech)

# Debug: Print include paths
message(STATUS "Motor include path: ${CMAKE_CURRENT_SOURCE_DIR}/../../../include")
message(STATUS "Motor src path: ${CMAKE_CURRENT_SOURCE_DIR}/../../")
