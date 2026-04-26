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

#ifdef __cplusplus
}
#endif

#endif  // ARM_CONTROL_TYPES_H
