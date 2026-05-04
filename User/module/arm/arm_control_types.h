/**
 ******************************************************************************
 * @file    arm_control_types.h
 * @brief   Arm control command types
 ******************************************************************************
 */

#ifndef ARM_CONTROL_TYPES_H
#define ARM_CONTROL_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "device/motor_dm.h"
#include "device/motor_lz.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ARM_CTRL_REMOTE_CARTESIAN = 0,
    ARM_CTRL_POSE_ABSOLUTE,
    ARM_CTRL_POSE_DELTA,
    ARM_CTRL_CUSTOM_JOINT,
} Arm_CtrlType_t;

#define ARM_JOINT_COUNT 3

typedef struct {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
} ArmPose_t;

typedef struct {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
} ArmPoseDelta_t;

typedef struct {
    float q[ARM_JOINT_COUNT];
} ArmJointAngles_t;

typedef enum {
    ARM_CTRL_FRAME_WORLD = 0,
    ARM_CTRL_FRAME_TOOL = 1,
    ARM_CTRL_FRAME_HEADING = 2,
} ArmControlFrame_t;

typedef struct {
    bool enable;
    bool set_target_as_current;
    bool gripper_toggle;
    bool gripper_close;
    Arm_CtrlType_t ctrl_type;
    ArmControlFrame_t frame;
    ArmPose_t target_pose;
    ArmPoseDelta_t target_delta;
    ArmPoseDelta_t joy_vel;
    ArmJointAngles_t target_joints;
} Arm_CMD_t;

typedef struct {
    float y_min;
    float y_max;
    float z_min;
    float z_max;
    float pitch_min;
    float pitch_max;
} ArmWorkspaceLimit_t;

typedef struct {
    float input_deadzone;
    float max_y_velocity;
    float max_z_velocity;
    float max_pitch_velocity;
    float max_linear_velocity;
    float max_angular_velocity;
    float max_linear_acceleration;
    float max_angular_acceleration;
    float max_joint_step;
    float joint_max_velocity[ARM_JOINT_COUNT];
    float joint_max_acceleration[ARM_JOINT_COUNT];
    ArmWorkspaceLimit_t workspace;
} ArmCartesianRemoteParam_t;

typedef struct {
    MOTOR_LZ_Param_t joint1_motor_param;
    MOTOR_DM_Param_t joint2_motor_param;
    MOTOR_DM_Param_t joint3_motor_param;
    float joint_kp[ARM_JOINT_COUNT];
    float joint_kd[ARM_JOINT_COUNT];
    float gravity_comp_scale[ARM_JOINT_COUNT];
    ArmCartesianRemoteParam_t remote_cartesian;
} Arm_Params_t;

#ifdef __cplusplus
}
#endif

#endif  // ARM_CONTROL_TYPES_H
