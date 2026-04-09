/*
 * Arm module: 4x3508 support motors + 2x2006 drive-wheel motors.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/pid.h"
#include "device/motor_rm.h"
#include "device/motor_lz.h"
#include "device/motor_dm.h"

#ifdef __cplusplus
#include "device/motors/motor.hpp"
#include "device/motors/motor_packages/self_test/motor_self_test.hpp"
#include "device/motors/motor_packages/limit_selfLearning/dual_limit/motor_dual_limit_calibration.hpp"
#include "device/motors/motor_packages/limit_selfLearning/single_limit/motor_single_limit_calibration.hpp"
#endif


#define ARM_OK (0)
#define ARM_ERR (-1)
#define ARM_ERR_NULL (-2)

#define ARM_STUDY_POINT_NUM (5)//记录的轨迹点数

typedef enum {
  ARM_MODE_RELAX = 0,
  ARM_MODE_CILIBRATE, //校准模式，上电执行电机校准
  ARM_MODE_ACTIVE,
  ARM_MODE_TEACH, //示教模式，记录轨迹
  ARM_MODE_STUDY,  //学习模式，模仿示教模式的轨迹
  ARM_MODE_POINT2POINT //点对点控制模式，直接运动到预设的目标点
} Arm_Mode_t;

typedef enum {
  ARM_CALI_MODE_SOFT_LIMIT_LEARN = 0,
  ARM_CALI_MODE_ZERO_LIMIT_TRAVEL,
} Arm_Joint3CaliMode_t;



typedef struct {
  struct {
    Arm_Joint3CaliMode_t mode;
    float seek_velocity_rad_per_sec;
    bool seek_positive_direction;
    float user_travel_rad;
  } joint3_cali;

  MOTOR_LZ_Param_t lzmotor_param;
  MOTOR_DM_Param_t dmmotor_param;
  MOTOR_RM_Param_t rmmotor_param;

  KPID_Params_t pid_rmmotor_pos;
  KPID_Params_t pid_rmmotor_vel;
} Arm_Params_t;

typedef struct {
  MOTOR_LZ_Feedback_t lzmotor_feedback;
  MOTOR_Feedback_t dmmotor_feedback;
  MOTOR_RM_Feedback_t rmmotor_feedback;
} Arm_Feedback_t;

typedef struct {
  float joint1;  // 关节1绝对角度 = LZ电机角度
  float joint2;  // 关节2绝对角度 = DM电机角度 + 关节1角度
  float joint3;  // 关节3绝对角度 = RM电机角度 + 关节2角度
} Arm_JointAngle_t;

typedef struct {
  MOTOR_LZ_MotionParam_t lzmotor;
  MOTOR_MIT_Output_t dmmotor;
  float rmmotor;
} Arm_Output_t;

typedef enum {
  ARM_STUDY_SLEEP=0,  //默认上电姿态
  ARM_STUDY_MINUS_20CM,//-20取矿轨迹
  ARM_STUDY_PLUS_20CM,//+20放矿轨迹
  ARM_STUDY_PLUS_40CM,//+40放矿轨迹
  ARM_STUDY_SAVE_LOW, //存矿轨迹
  ARM_STUDY_SAVE_HIGH, //存矿轨迹
  ARM_STUDY_PLACE, //放矿轨迹
  ARM_STUDY_NONE
} Arm_StudyMode_t;//示教轨迹控制，记录整个运动过程

typedef enum {
  ARM_POINT_SLEEP=0,  //默认上电姿态
  ARM_POINT_MINUS_20CM,//-20取矿姿态
  ARM_POINT_PLUS_20CM,//+20放矿姿态
  ARM_POINT_PLUS_40CM,//+40放矿姿态
  ARM_POINT_SAVE_LOW, //存矿姿态
  ARM_POINT_SAVE_HIGH, //存矿姿态
  ARM_POINT_PLACE, //放矿姿态
  ARM_POINT_NONE
} Arm_Point2PointMode_t;//点对点控制，只记录终值点，插值轨迹控制运动

typedef struct {
  Arm_Mode_t mode;
  Arm_Point2PointMode_t point2point_mode; //点对点控制模式，决定执行哪个预设姿态
  Arm_StudyMode_t study_mode; //示教模式，记录哪个轨迹
  
} Arm_CMD_t;

typedef struct {
  float lzmotor_pos;    // 关节1的绝对目标角度（不含耦合补偿）
  float dmmotor_pos;    // 关节2的绝对目标角度（控制时自动补偿关节1的当前角度）
  float rmmotor_pos;    // 关节3的绝对目标角度（控制时自动补偿关节2的当前角度）
} Arm_Point2Point_t;


typedef struct {
    float time[ARM_STUDY_POINT_NUM];
    float lzmotor_pos[ARM_STUDY_POINT_NUM];
    float dmmotor_pos[ARM_STUDY_POINT_NUM];
    float rmmotor_pos[ARM_STUDY_POINT_NUM];
} Arm_Study_t;

typedef struct {

  Arm_Params_t *param;
  MOTOR_LZ_Param_t *lzmotor_param;
  MOTOR_DM_Param_t *dmmotor_param;
  MOTOR_RM_Param_t *rmmotor_param;
  Arm_Mode_t mode;

  struct {
    float now;            /* 当前时间，单位秒 */
    uint64_t lask_wakeup; /* 上次唤醒时间，单位微秒 */
    float dt;             /* 两次唤醒间隔时间，单位秒 */
  } timer; //灵足电机特有状态

  Arm_Study_t study[ARM_STUDY_NONE]; //示教模式记录的轨迹点
  Arm_StudyMode_t study_mode; //当前示教模式，决定执行哪个轨迹点

  Arm_Point2Point_t point2point[ARM_POINT_NONE]; //当前点对点控制的目标姿态
  Arm_Point2PointMode_t point2point_mode; //点对点控制模式，决定执行哪个预设姿态

  struct {
    KPID_t rmmotor_pos;
    KPID_t rmmotor_vel;
  } pid;

  Arm_CMD_t cmd;

  struct {
    bool cilibrate;
    bool self_test_started;
    bool soft_limit_started;
    Arm_Joint3CaliMode_t cali_mode;
    bool zero_limit_travel_started;
    float user_travel_rad;
    float rmmotor_min;
    float rmmotor_max;
#ifdef __cplusplus
    mrobot::Motor *motor;
    void *self_test_storage;
    void *dual_limit_storage;
    void *single_limit_storage;
    mrobot::MotorSelfTest *self_test;
    mrobot::MotorDualLimitCalibration *dual_limit;
    mrobot::MotorSingleLimitCalibration *single_limit;
#endif
  } joint3cil;


  struct {
    MOTOR_LZ_MotionParam_t lzmotor;
    MOTOR_MIT_Output_t dmmotor;
    float rmmotor;
  } setpoint;

  // 插值轨迹相关
  struct {
    float lzmotor_target;
    float dmmotor_target;
    float rmmotor_target;
    bool lzmotor_active;
    bool dmmotor_active;
    bool rmmotor_active;
  } interp;

  Arm_Feedback_t feedback;
  Arm_JointAngle_t joint_angle;  // 去耦合后的各关节绝对角度
  struct {
    MOTOR_RM_t *rmmotor;
    MOTOR_DM_t *dmmotor;
    MOTOR_LZ_t *lzmotor;
  } motor;
  
  Arm_Output_t out;

} Arm_t;

int8_t Arm_Init(Arm_t *a, Arm_Params_t *param, float target_freq);
int8_t Arm_UpdateFeedback(Arm_t *a);
int8_t Arm_Control(Arm_t *a, const Arm_CMD_t *a_cmd);
int8_t Arm_Output(Arm_t *a);
void Arm_ResetOutput(Arm_t *a);
void Arm_Relax(Arm_t *a);
void Arm_Teach(Arm_t *a, Arm_StudyMode_t study_mode);
#ifdef __cplusplus
}
#endif




