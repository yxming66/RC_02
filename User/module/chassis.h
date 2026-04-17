/*
 * 底盘模块
 */
#pragma once


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <cmsis_os2.h>
#include "bsp/can.h"
#include "component/filter.h"
#include "component/mixer.h"
#include "component/pid.h"
#include <stdbool.h>
#include "component/ahrs.h"
#include "device/motor_rm.h"

#ifdef __cplusplus
#include "device/motor.h"
#include "device/motor/packages/controller/motor_controller.hpp"
#include "device/motor/protocol/rm_protocol.hpp"
#endif
/* Exported constants ------------------------------------------------------- */
#define CHASSIS_OK (0)        /* 运行正常 */
#define CHASSIS_ERR (-1)      /* 运行时出现一般错误 */
#define CHASSIS_ERR_NULL (-2) /* 运行时检测到空指针 */
#define CHASSIS_ERR_MODE (-3) /* 运行时检测到底盘模式配置错误 */
#define CHASSIS_ERR_TYPE (-4) /* 运行时检测到底盘类型配置错误 */
	
#define MAX_MOTOR_CURRENT 20.0f 
/* 底盘控制模式 */
typedef enum { 
  CHASSIS_MODE_RELAX,             /* 放松模式，电机不输出 */
  CHASSIS_MODE_BREAK,             /* 刹车模式，电机闭环保持静止 */
  CHASSIS_MODE_FOLLOW_GIMBAL,     /* 跟随云台朝向 */
  CHASSIS_MODE_FOLLOW_GIMBAL_35,  /* 相对云台固定偏置 35° 跟随 */
  CHASSIS_MODE_ROTOR,             /* 小陀螺模式，底盘持续自旋 */
  CHASSIS_MODE_INDEPENDENT,       /* 独立模式，不受云台朝向影响 */
  CHASSIS_MODE_OPEN,              /* 开环模式，直接输出到电机 */
} Chassis_Mode_t;
	
/* 小陀螺旋转方向模式 */
typedef enum {
  ROTOR_MODE_CW,   /* 顺时针旋转 */
  ROTOR_MODE_CCW,  /* 逆时针旋转 */
  ROTOR_MODE_RAND, /* 随机方向旋转 */
} Chassis_RotorMode_t;

/* UI 导出结构（供 referee 系统绘制） */
typedef struct {
  Chassis_Mode_t mode;
  float angle;
} Chassis_RefereeUI_t;

/**
 * @brief 底盘统一方向约定
 *
 * 1. 车体坐标系：右手系
 *    - +vx：车头方向（前进）
 *    - +vy：车体左侧方向（左移）
 *    - +wz：从车体上方俯视为逆时针旋转
 *
 * 2. 云台/视觉对接建议：
 *    - 视觉给到底盘的前向速度使用 +vx
 *    - 视觉给到底盘的左向速度使用 +vy
 *    - 视觉给到底盘的偏航角速度使用 +wz
 *
 * 3. 电机方向约定：
 *    - 各电机反馈正方向必须先在驱动层统一
 *    - 若某电机实际安装方向相反，仅修改 motor_param.reverse
 *    - 上层模块禁止再次做符号翻转
 *
 * 4. 麦轮轮序约定：
 *    前方
 *    [1] [0]
 *    [2] [3]
 */

/* 底盘控制命令 */
typedef struct {
  Chassis_Mode_t mode;     /* 底盘运行模式 */
  Chassis_RotorMode_t mode_rotor; /* 小陀螺旋转模式 */
  MoveVector_t ctrl_vec;      /* 底盘控制向量：vx/vy 单位 m/s，wz 单位 rad/s */
} Chassis_CMD_t;

/* 通用限幅结构 */
typedef struct {
  float max;
  float min;
} Chassis_Limit_t;

  /* 底盘类型（底盘机械构型） */
typedef enum {
  CHASSIS_TYPE_MECANUM,    /* 四麦克纳姆轮 */
  CHASSIS_TYPE_PARLFIX4,   /* 四平行定向轮 */
  CHASSIS_TYPE_PARLFIX2,   /* 双平行定向轮 */
  CHASSIS_TYPE_OMNI_CROSS, /* 交叉布局全向轮 */
  CHASSIS_TYPE_OMNI_PLUS,  /* 十字布局全向轮 */
  CHASSIS_TYPE_DRONE,      /* 飞行器底盘 */
  CHASSIS_TYPE_SINGLE,     /* 单驱动轮 */
} Chassis_Type_t;


/* 底盘参数结构体 */
typedef struct {
	MOTOR_RM_Param_t motor_param[4];
  struct {
    float external_ratio;
    bool reverse_output;
  } motor_install[4];
	  struct {
    KPID_Params_t motor_pid_param;  /* 轮速闭环 PID 参数 */
    KPID_Params_t motor_pos_pid_param; /* 预留位置环参数 */
    KPID_Params_t follow_pid_param; /* 跟随云台 PID 参数 */
  } pid;
  Chassis_Type_t type; /* 底盘类型 */

  /* 低通滤波器截止频率 */
  struct {
    float in;  /* 反馈输入滤波 */
    float out; /* 控制输出滤波 */
  } low_pass_cutoff_freq;

  /* 方向修正开关，符号统一应尽量在驱动层完成 */
  struct {
    bool yaw;
  } reverse;
  struct {
    float wheel_radius_m;       /* 车轮半径，单位 m */
    float wheelbase_m;          /* 前后轮中心距的一半所用等效长度参数，单位 m */
    float trackwidth_m;         /* 左右轮中心距的一半所用等效长度参数，单位 m */
    float wheel_output_max_speed; /* 轮线速度上限，单位 m/s */
  } physical;
	struct {
        float max_vx, max_vy, max_wz; /* 车体速度上限：m/s, m/s, rad/s */
        float max_torque_cmd;         /* 电机目标力矩限幅，单位 N·m */
    } limit; 
  struct {
    float sample_freq;
    float position_to_velocity_limit;
    float velocity_to_torque_limit;
  } controller;
} Chassis_Params_t;

typedef struct {
  AHRS_Gyro_t gyro;
  AHRS_Eulr_t eulr;
} Chassis_IMU_t;

typedef struct {
  struct {
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
    bool online;
  } motor[4];
	float encoder_gimbalYawMotor; /* 云台偏航反馈角 */
  MoveVector_t chassis_vel; /* 车体速度反馈：vx/vy 单位 m/s，wz 单位 rad/s */
} Chassis_Feedback_t;

typedef struct {
  float dt_s;
  float gimbal_beta_rad;
  MoveVector_t cmd_vec_raw;
  MoveVector_t cmd_vec_body;
  MoveVector_t cmd_vec_limited;
  float rotor_wz_cmd;
  float wheel_speed_ref_mps[4];
  float wheel_speed_fdb_mps[4];
  float wheel_speed_fdb_filtered_mps[4];
  float wheel_torque_pid_out[4];
  float wheel_torque_cmd_nm[4];
  float wheel_motor_velocity_rad_s[4];
  float wheel_motor_torque_nm[4];
  bool wheel_pending_valid[4];
  float wheel_pending_torque_current[4];
  float wheel_last_set_torque_nm[4];
  int8_t wheel_last_set_torque_ret[4];
  int8_t wheel_last_commit_ret[4];
  bool wheel_last_commit_skipped[4];
  float body_vel_raw_vx;
  float body_vel_raw_vy;
  float body_vel_raw_wz;
} Chassis_Debug_t;

/* 底盘输出结构体 */
typedef struct {
  float motor[4];  /* 当前发给底层电机链路的控制量 */
  int8_t set_torque_ret[4];
  int8_t controller_update_ret[4];
  int8_t commit_ret[4];
  bool command_pending[4];
  bool last_commit_ok[4];
} Chassis_Output_t;

/*
 * 底盘运行主结构体
 * 包含参数、状态、目标值、反馈值和输出量
 */
typedef struct {
  uint64_t last_wakeup;
  float dt;

  Chassis_Params_t *param; /* 底盘参数 */

  /* 模块状态 */
  Chassis_Mode_t mode; /* 当前底盘模式 */


  /* 底盘构型 */
  int8_t num_wheel; /* 轮子数量 */
  Mixer_t mixer;    /* 混控器：车体速度 -> 各轮目标速度 */
  MoveVector_t move_vec; /* 车体运动向量：vx/vy 单位 m/s，wz 单位 rad/s */
  void *motors[4];
  void *controllers[4];
	float mech_zero; /* 云台跟随参考零位 */
  float wz_multi; /* 小陀螺方向系数 */

  /* PID 计算目标值 */
  struct {
    float wheel_speed[4]; /* 轮输出轴线速度目标，单位 m/s */
  } setpoint;

  /* 反馈控制用 PID */
  struct {
    KPID_t motor[4]; /* 各轮速度环 PID */
    KPID_t follow; /* 云台跟随 PID */
  } pid;
 
	struct {
        Chassis_Limit_t vx, vy, wz; 
    } limit; 
	
  /* 滤波器 */
  struct {
    LowPassFilter2p_t in[4];  /* 速度反馈滤波 */
    LowPassFilter2p_t out[4]; /* 控制输出滤波 */
     LowPassFilter2p_t torque[4]; /* 力矩观测滤波，仅用于调试显示 */
    LowPassFilter2p_t chassis_vel[3]; /* 车体速度反馈滤波: vx/vy/wz */
  } filter;

  Chassis_Output_t out; /* 电机控制输出 */
	Chassis_Feedback_t feedback;
  Chassis_Debug_t debug;
} Chassis_t;

/* Exported functions prototypes -------------------------------------------- */

/**
 * \brief 初始化底盘模块
 *
 * \param c 底盘对象
 * \param param 底盘参数指针
 * \param target_freq 控制任务频率
 *
 * \return 运行结果
 */
int8_t Chassis_Init(Chassis_t *c, const Chassis_Params_t *param,
                    float target_freq);

/**
 * \brief 更新底盘反馈信息
 *
 * \param c 底盘对象
 *
 * \return 运行结果
 */
int8_t Chassis_UpdateFeedback(Chassis_t *c);

/**
 * \brief 执行底盘控制逻辑
 *
 * \param c 底盘对象
 * \param c_cmd 底盘控制命令
 * \param now 当前时刻，单位 ms
 *
 * \return 运行结果
 */
int8_t Chassis_Control(Chassis_t *c, const Chassis_CMD_t *c_cmd,
                       uint32_t now);


/**
 * \brief 下发底盘输出
 *
 * \param c 底盘对象
 */
void Chassis_Output(Chassis_t *c);

/**
 * \brief 清空底盘输出
 *
 * \param c 底盘对象
 */
void Chassis_ResetOutput(Chassis_t *c);

void Chassis_DumpUI(const Chassis_t *c, Chassis_RefereeUI_t *ui);


void Chassis_Power_Control(Chassis_t *c,float max_power);
/**
 * @brief 底盘功率限制
 *
 * @param c 底盘数据
 * @param cap 电容数据
 * @param ref 裁判系统数据
 * @return 函数运行结果
 */
// 尚未接入，后续实现时再启用
// int8_t Chassis_PowerLimit(Chassis_t *c, const CAN_Capacitor_t *cap,
//                          const Referee_ForChassis_t *ref);


/**
 * @brief 导出底盘数据
 *
 * @param c 底盘数据结构体
 * @param ui UI 数据结构体
 */
void Chassis_DumpUI(const Chassis_t *c, Chassis_RefereeUI_t *ui);

#ifdef __cplusplus
}
#endif




