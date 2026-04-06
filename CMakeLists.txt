cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RC_02)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # User/bsp sources
    User/bsp/can.c
    User/bsp/mm.c
    User/bsp/time.c
    User/bsp/uart.c
    User/bsp/pwm.c

    # User/component sources
    User/component/crc16.c
    User/component/crc8.c
    User/component/filter.c
    User/component/limiter.c
    User/component/mixer.c
    User/component/pid.c
    User/component/user_math.c

    # User/device sources
    User/device/buzzer.c
    User/device/dr16.c
    User/device/motor.c
    User/device/motor_rm.c
    User/device/motor_dm.c
    User/device/motor_lz.c
    User/device/motors/motor.cpp
    User/device/motors/motor_packages/self_test/motor_self_test.cpp
    User/device/motors/motor_packages/soft_limit_learning/motor_soft_limit_learning.cpp
    # User/module sources
    User/module/chassis.c
    User/module/pole.c
    User/module/arm.cpp
    User/module/rod.c
    # User/module/cmd sources
    User/module/cmd/cmd.c
    User/module/cmd/cmd_adapter.c
    User/module/cmd/cmd_behavior.c
    User/module/cmd/cmd_example.c

    # User/module sources
    User/module/config.c

    # User/task sources
    User/task/blink.c
    User/task/chassis_main.c
    User/task/cmd_main.c
    User/task/init.c
    User/task/rc_main.c
    User/task/user_task.c
    User/task/sick.c
    User/task/arm_main.c
    User/task/rod_main.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
