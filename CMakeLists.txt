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

# Add compiler flags to fix CMSIS GCC header compatibility with C++
# This is needed for ARM CMSIS 5.2.0 which has local types in extern declarations
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -fpermissive
)

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
    User/bsp/fdcan.c
    User/bsp/mm.c
    User/bsp/time.c
    User/bsp/uart.c
    User/bsp/pwm.c
    User/bsp/gpio.c
    User/bsp/spi.c
    # User/component sources
    User/component/crc16.c
    User/component/crc8.c
    User/component/filter.c
    User/component/limiter.c
    User/component/mixer.c
    User/component/pid.c
    User/component/user_math.c
    User/component/ahrs.c
    # User/device sources
    User/device/bmi088.c
    User/device/buzzer.c
    User/device/dr16.c
    User/device/motor.c
    User/device/motor_rm.c
    User/device/motor_dm.c
    User/device/motor_lz.c
    User/device/motor/factory/motor_factory.cpp
    User/device/motor/protocol/rm_protocol.cpp
    User/device/motor/protocol/dm_protocol.cpp
    User/device/motor/protocol/lz_protocol.cpp
    User/device/motor/packages/controller/motor_controller.cpp
    User/device/motor/packages/soft_limit_learning/soft_limit_learning.cpp
    User/component/toolbox/matrix.cpp
    User/component/toolbox/robotics.cpp
    User/component/toolbox/utils.cpp
    User/component/arm_lib/adapter/toolbox_adapter.cpp
    User/component/arm_lib/model/joint.cpp
    User/component/arm_lib/model/link.cpp
    User/component/arm_lib/model/serial_chain.cpp
    User/component/arm_lib/model/tool_frame.cpp
    User/component/arm_lib/parser/chain_builder.cpp
    User/component/arm_lib/parser/urdf_loader.cpp
    User/component/arm_lib/kinematics/fk.cpp
    User/component/arm_lib/kinematics/jacobian.cpp
    User/component/arm_lib/kinematics/pose_error.cpp
    User/component/arm_lib/kinematics/ik.cpp
    User/component/arm_lib/kinematics/ik_analytic.cpp
    User/component/arm_lib/kinematics/ik_3r_planar.cpp
    User/component/arm_lib/kinematics/ik_numeric.cpp
    User/component/arm_lib/solver/lm_solver.cpp
    User/component/arm_lib/solver/nr_solver.cpp
    User/component/arm_lib/solver/seed_policy.cpp
    User/component/arm_lib/dynamics/gravity.cpp
    User/component/arm_lib/trajectory/interpolation.cpp
    User/component/arm_lib/trajectory/joint_traj.cpp
    User/component/arm_lib/trajectory/cartesian_traj.cpp
    # User/module sources
    User/module/chassis.cpp
    User/module/pole.c
    User/module/rod.cpp
    User/module/autoCtrlAPI/api/auto_ctrl_api.c
    User/module/autoCtrlAPI/core/auto_ctrl_math.c
    User/module/autoCtrlAPI/primitive/auto_ctrl_primitive.c
    User/module/autoCtrlAPI/step/auto_ctrl_step.c
    User/module/autoCtrlAPI/template/auto_ctrl_template_runner.c

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
    User/task/auto_ctrl_feed.c
    User/task/user_task.c
    User/task/sick.c
    User/task/arm_main.cpp
    User/task/rod_main.c
    User/task/atti_esti.c
    User/task/pc_uart_tx.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User
    Drivers/CMSIS/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM7
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM7lfdp_math.a

    # Add user defined libraries
)
