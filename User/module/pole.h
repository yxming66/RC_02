/*
 * Pole module: 4x3508 support motors.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/pid.h"
#include "component/filter.h"
#include "device/motor.h"
#include "device/motor_rm.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "device/motor/packages/controller/motor_controller.hpp"
#endif

#define POLE_OK (0)
#define POLE_ERR (-1)
#define POLE_ERR_NULL (-2)

#define POLE_SUPPORT_MOTOR_NUM (4)
#define POLE_MOTOR_NUM (POLE_SUPPORT_MOTOR_NUM)

typedef enum {
  POLE_MODE_RELAX = 0,
  POLE_MODE_ACTIVE,
} Pole_Mode_t;

typedef struct {
  Pole_Mode_t mode;
  float lift[2];   /* [-1, 1], >0 means up */
  bool auto_target_enable[2];
  float auto_target_lift[2];   /* rad, relative to calibrated lower limit */
  float auto_lift_speed[2];    /* rad/s, <=0 means use default support_lift_speed */
  bool disable_lift_accel;     /* true: 该 cmd 不对 lift 加速度限幅（仅限速） */
} Pole_CMD_t;

typedef struct {
  MOTOR_RM_Param_t motor_param[POLE_MOTOR_NUM];
  struct {
    float external_ratio;
    bool reverse_output;
  } motor_install[POLE_MOTOR_NUM];
  struct {
    KPID_Params_t support_pos_pid;
    KPID_Params_t support_vel_pid;
  } pid;
  struct {
    float support_vel_feedback_cutoff_hz;   /* Hz, <=0 means fallback */
    float support_output_cutoff_hz;         /* Hz, <=0 means fallback */
    float support_vel_feedback_spike_rad_s; /* rad/s, per-cycle spike clamp, <=0 disable */
  } filter;
  struct {
    bool use_legacy_normalized_output; /* true: map output to [-1,1] and use MOTOR_RM_SetOutput */
  } output;
  struct {
    float load_enter_nm;      /* Nm, 侧平均载荷高于该阈值判定“接地”（离地->接地切换） */
    float load_exit_nm;       /* Nm, 侧平均载荷低于该阈值判定“离地”（接地->离地切换，需小于进入阈值） */
    float speed_gate_rad_s;   /* rad/s, 速度门限；速度过高时冻结载荷采样，减少动态工况误判 */
    uint8_t debounce_cycles;  /* control cycles, 状态切换去抖计数 */
    float load_lpf_hz;        /* Hz, 载荷估计低通频率，越小越稳但响应更慢 */
  } off_ground;
  struct {
    bool enable;                                   /* 总开关：true 启用离地重力补偿 */
    float torque_ff_nm[POLE_SUPPORT_MOTOR_NUM];   /* Nm, 每根撑杆的基础重力前馈（带符号） */
    float descending_scale;                        /* 0~1, 下压方向补偿缩放，防止下行发硬 */
    float err_deadband_rad;                        /* rad, 目标-反馈误差死区，用于判断上抬/下压方向 */
    float rise_rate;                               /* 1/s, 检测到离地时补偿混合系数上升斜率 */
    float fall_rate;                               /* 1/s, 回到接地时补偿混合系数下降斜率 */
  } gravity_comp;
  struct {
    float step_200_all_extend[2];      /* 200mm台阶-四撑杆全伸: [0]前两杆, [1]后两杆 */
    float step_200_front_retract[2];   /* 200mm台阶-前两撑杆收: [0]前两杆, [1]后两杆 */
    float step_200_all_retract[2];     /* 200mm台阶-四撑杆全收: [0]前两杆, [1]后两杆 */
    float step_200_small[2];           /* 200mm台阶-小目标位: [0]前两杆, [1]后两杆 */
    float step_400_all_extend[2];      /* 400mm台阶-四撑杆全伸: [0]前两杆, [1]后两杆 */
    float step_400_front_retract[2];   /* 400mm台阶-前两撑杆收: [0]前两杆, [1]后两杆 */
    float step_400_all_retract[2];     /* 400mm台阶-四撑杆全收: [0]前两杆, [1]后两杆 */
    float ore_release_target[2];       /* 一键放矿专用撑杆目标: [0]前两杆, [1]后两杆 */
    float ore_release_speed;           /* 一键放矿专用撑杆目标跟随速度，rad/s */
  } preset;
  struct {
    float max_torque_nm; /* Nm, <=0 means fallback to max_current */
    float max_current;   /* legacy fallback */
    float support_total_travel;   /* rad */
    float support_lift_speed;     /* rad/s */
    float support_lift_accel;     /* rad/s^2, target lift acceleration limit; <=0 disables soft start */
    float support_limit_soft_zone; /* rad, slow down near lower/upper limit */
    float support_hold_zone;      /* rad, near lower limit hold target */
    float support_hold_speed_threshold; /* rad/s, low-speed condition for hold */
    float support_hold_release_zone; /* rad, release hold only after moving away from lower limit */
    float support_output_slew_rate; /* output unit/s, <=0 disable */
  } limit;
  MOTOR_TemperatureProtectionConfig_t motor_temperature_protection;
} Pole_Params_t;

typedef struct {
  MOTOR_Feedback_t motor[POLE_MOTOR_NUM];
  bool temperature_warning[POLE_MOTOR_NUM];
  bool temperature_over_limit[POLE_MOTOR_NUM];
  float support_vel_filtered[POLE_SUPPORT_MOTOR_NUM];
  float support_angle_avg;
} Pole_Feedback_t;

typedef struct {
  float motor_load_nm[POLE_SUPPORT_MOTOR_NUM];   /* 每根撑杆载荷估计（低通后绝对值） */
  float side_load_nm[2];                         /* 每侧载荷估计（2根撑杆平均值） */
  uint8_t side_off_ground[2];                    /* 每侧离地状态：0接地，1离地 */
  float gravity_scale[2];                        /* 每侧重力补偿混合系数 [0,1] */
  float gravity_ff_nm[POLE_SUPPORT_MOTOR_NUM];   /* 每根撑杆当前实际重力前馈扭矩 */
  float torque_pid_nm[POLE_SUPPORT_MOTOR_NUM];   /* 每根撑杆 PID 扭矩（未加重力前馈） */
  float torque_cmd_nm[POLE_SUPPORT_MOTOR_NUM];   /* 每根撑杆最终输出扭矩（已限幅） */
  float target_angle_rad[POLE_SUPPORT_MOTOR_NUM];    /* 每根撑杆目标角度 */
  float feedback_angle_rad[POLE_SUPPORT_MOTOR_NUM];  /* 每根撑杆反馈角度 */
  float feedback_speed_rad_s[POLE_SUPPORT_MOTOR_NUM];/* 每根撑杆反馈角速度 */
  float final_target_lift[2];                    /* 每侧最终目标 lift, rad */
  float tracked_target_lift[2];                  /* 每侧规划后目标 lift, rad */
  float tracked_target_velocity[2];              /* 每侧规划后目标速度, rad/s */
} Pole_Debug_t;

typedef struct {
  float motor[POLE_MOTOR_NUM];
} Pole_Output_t;

typedef struct {
  uint32_t last_wakeup;
  float dt;

  const Pole_Params_t *param;
  Pole_Mode_t mode;
  void *motors[POLE_MOTOR_NUM];
  void *controllers[POLE_MOTOR_NUM];

  struct {
    bool calibrated;
    float lower[POLE_SUPPORT_MOTOR_NUM];
    float upper[POLE_SUPPORT_MOTOR_NUM];
    float target_offset[POLE_SUPPORT_MOTOR_NUM];
    float final_target_lift[2];
    float tracked_target_lift[2];
    float tracked_target_velocity[2];
    bool auto_target_was_enabled[2];
    bool lower_hold_latched[POLE_SUPPORT_MOTOR_NUM];
  } support_angle;

  struct {
    float motor_load_nm[POLE_SUPPORT_MOTOR_NUM];
    float side_load_nm[2];
    float gravity_scale[2];
    bool side_off_ground[2];
    uint8_t side_enter_count[2];
    uint8_t side_exit_count[2];
  } off_ground;

  struct {
    float support_target_angle[POLE_SUPPORT_MOTOR_NUM];
    bool disable_lift_accel;   /* 保存最近一次 cmd 的标志 */
  } setpoint;

  struct {
    KPID_t support_pos[POLE_SUPPORT_MOTOR_NUM];
    KPID_t support_vel[POLE_SUPPORT_MOTOR_NUM];
  } pid;

  struct {
    LowPassFilter2p_t support_vel_in[POLE_SUPPORT_MOTOR_NUM];
    LowPassFilter2p_t support_out[POLE_SUPPORT_MOTOR_NUM];
  } filter;

  Pole_Output_t out;
  Pole_Feedback_t feedback;
  Pole_Debug_t debug;

  /* PC上位机接口 */
  struct {
    bool pc_control_enable;
    float pc_lift[2];         /* PC下发升降命令 [-1, 1] */
    uint8_t pc_mode;          /* PC下发模式 */
  } pc_if;
} Pole_t;

#ifdef __cplusplus
extern "C" {
#endif

int8_t Pole_Init(Pole_t *c, const Pole_Params_t *param, float target_freq);
int8_t Pole_UpdateFeedback(Pole_t *c);
int8_t Pole_Control(Pole_t *c, const Pole_CMD_t *c_cmd, uint32_t now);
bool Pole_IsGroupAtTarget(const Pole_t *c, uint8_t group, float threshold_rad);
bool Pole_IsAllAtTarget(const Pole_t *c, float threshold_rad);
bool Pole_IsGroupAtFinalTarget(const Pole_t *c, uint8_t group,
                               float threshold_rad);
bool Pole_IsAllAtFinalTarget(const Pole_t *c, float threshold_rad);
bool Pole_IsSideOffGround(const Pole_t *c, uint8_t side);
bool Pole_IsAnyOffGround(const Pole_t *c);
bool Pole_HasTemperatureWarning(const Pole_t *c);
bool Pole_HasTemperatureOverLimit(const Pole_t *c);
float Pole_GetMaxTemperature(const Pole_t *c);
const Pole_Debug_t *Pole_GetDebug(const Pole_t *c);
void Pole_Output(Pole_t *c);
void Pole_ResetOutput(Pole_t *c);
void Pole_Power_Control(Pole_t *c, float max_power);

/* PC上位机接口函数 */
void Pole_InitPCInterface(Pole_t *c);
void Pole_SetPCCommand(Pole_t *c, uint8_t mode, float lift0, float lift1);
bool Pole_IsPCControlEnabled(const Pole_t *c);
const float* Pole_GetPCLiftCommand(const Pole_t *c);
void Pole_FillPCFeedback(const Pole_t *c, void *fb);

#ifdef __cplusplus
}
#endif




