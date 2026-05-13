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
    User/component/pid.c
    User/component/timer.cpp
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
    User/device/wheel/wheel.cpp
    User/component/toolbox/matrix.cpp
    User/component/toolbox/robotics.cpp
    User/component/toolbox/utils.cpp
    User/robotics/arm/adapter/toolbox_adapter.cpp
    User/robotics/arm/model/joint.cpp
    User/robotics/arm/model/link.cpp
    User/robotics/arm/model/serial_chain.cpp
    User/robotics/arm/model/tool_frame.cpp
    User/robotics/arm/parser/chain_builder.cpp
    User/robotics/arm/parser/dh_loader.cpp
    User/robotics/arm/parser/urdf_loader.cpp
    User/robotics/arm/kinematics/fk.cpp
    User/robotics/arm/kinematics/jacobian.cpp
    User/robotics/arm/kinematics/pose_error.cpp
    User/robotics/arm/kinematics/ik.cpp
    User/robotics/arm/kinematics/ik_analytic.cpp
    User/robotics/arm/kinematics/ik_numeric.cpp
    User/robotics/arm/kinematics/analytic_solvers/ik_3r_planar.cpp
    User/robotics/arm/kinematics/analytic_solvers/ik_6r_pieper.cpp
    User/robotics/arm/solver/lm_solver.cpp
    User/robotics/arm/solver/nr_solver.cpp
    User/robotics/arm/solver/seed_policy.cpp
    User/robotics/arm/dynamics/gravity.cpp
    User/robotics/arm/trajectory/interpolation.cpp
    User/robotics/arm/trajectory/joint_traj.cpp
    User/robotics/arm/trajectory/cartesian_traj.cpp
    User/robotics/arm/control/joint_servo.cpp
    User/robotics/arm/control/cartesian_servo.cpp
    User/robotics/arm/application/three_pit_cartesian_app.cpp
    # User/module sources
    User/module/chassis/front_omni_rear_mecanum.cpp
    User/module/chassis/mecanum.cpp
    User/module/pole.cpp
    User/module/rod.cpp
    User/module/autoCtrlAPI/api/auto_ctrl_api.c
    User/module/autoCtrlAPI/core/auto_ctrl_math.cpp
    User/module/autoCtrlAPI/primitive/auto_ctrl_primitive.cpp
    User/module/autoCtrlAPI/step/auto_ctrl_step.c
    User/module/autoCtrlAPI/template/auto_ctrl_template_runner.c

    # User/module sources
    User/module/arm/arm.cpp
    User/module/config.c
    User/module/rod_new.cpp
    User/module/pc_protocol/pc_protocol.cpp

    # User/task sources
    User/task/blink.c
    User/task/chassis_main.cpp
    User/task/cmd_main.c
    User/task/init.c
    User/task/rc_main.c
    User/task/auto_ctrl_feed.c
    User/task/user_task.c
    User/task/sick.c
    User/task/arm_main.cpp
    User/task/rod_main.c
    User/task/atti_esti.c
    User/task/pc_comm_task.c
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
