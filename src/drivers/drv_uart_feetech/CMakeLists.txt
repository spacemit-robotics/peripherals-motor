cmake_minimum_required(VERSION 3.10)
project(feetech LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Get all source files
file(GLOB FEETECH_HDRS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
file(GLOB FEETECH_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# Remove motor adapter as it should only be compiled as part of motor library
list(REMOVE_ITEM FEETECH_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/feetech_motor_adapter.cpp")


# Motor framework sources for integration tests
set(MOTOR_FRAMEWORK_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../motor_core.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/feetech_motor_adapter.cpp
)

# test_simple executable (motor framework integration test)
add_executable(test_simple
  test/test_simple.c
  ${FEETECH_SRCS}
  ${MOTOR_FRAMEWORK_SRCS}
)

target_include_directories(test_simple PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../include  # motor.h framework
  ${CMAKE_CURRENT_SOURCE_DIR}/../../            # motor_core.h
)
target_link_libraries(test_simple pthread)



# test_debug executable (communication debugging tool)
add_executable(test_debug
  test/test_debug.cpp
  ${FEETECH_SRCS}
)

target_include_directories(test_debug PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(test_debug pthread)


# Install (optional)
install(TARGETS test_simple test_debug RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/feetech)

# Debug: Print include paths
message(STATUS "Motor include path: ${CMAKE_CURRENT_SOURCE_DIR}/../../../include")
message(STATUS "Motor src path: ${CMAKE_CURRENT_SOURCE_DIR}/../../")
