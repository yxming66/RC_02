#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "component/ahrs.h"
#include "component/filter.h"
#include "component/pid.h"
#include "component/user_math.h"
#include "device/motor_rm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_OK (0)
#define CHASSIS_ERR (-1)
#define CHASSIS_ERR_NULL (-2)
#define CHASSIS_ERR_MODE (-3)
#define CHASSIS_ERR_TYPE (-4)

typedef enum {
  CHASSIS_MODE_RELAX,
  CHASSIS_MODE_BREAK,
  CHASSIS_MODE_FOLLOW_GIMBAL,
  CHASSIS_MODE_FOLLOW_GIMBAL_35,
  CHASSIS_MODE_ROTOR,
  CHASSIS_MODE_INDEPENDENT,
  CHASSIS_MODE_OPEN,
} Chassis_Mode_t;

typedef enum {
  ROTOR_MODE_RAND = 0,
  ROTOR_MODE_CW = 1,
  ROTOR_MODE_CCW = 2,
} Chassis_RotorMode_t;

typedef struct {
  Chassis_Mode_t mode;
  float angle;
} Chassis_RefereeUI_t;

typedef struct {
  Chassis_Mode_t mode;
  Chassis_RotorMode_t mode_rotor;
  MoveVector_t ctrl_vec;
} Chassis_CMD_t;

typedef struct {
  float max;
  float min;
} Chassis_Limit_t;

typedef enum {
  CHASSIS_TYPE_MECANUM,
  CHASSIS_TYPE_FRONT_OMNI_REAR_MECANUM,
  CHASSIS_TYPE_PARLFIX4,
  CHASSIS_TYPE_PARLFIX2,
  CHASSIS_TYPE_OMNI_CROSS,
  CHASSIS_TYPE_OMNI_PLUS,
  CHASSIS_TYPE_DRONE,
  CHASSIS_TYPE_SINGLE,
} Chassis_Type_t;

typedef struct {
  MOTOR_RM_Param_t motor_param[4];
  struct {
    float external_ratio;
    bool reverse_output;
  } motor_install[4];
  struct {
    KPID_Params_t motor_pid_param;
    KPID_Params_t motor_high_pole_pid_param;
    KPID_Params_t motor_pos_pid_param;
    KPID_Params_t follow_pid_param;
  } pid;
  Chassis_Type_t type;
  struct {
    float in;
    float out;
  } low_pass_cutoff_freq;
  struct {
    bool yaw;
  } reverse;
  struct {
    float wheel_radius_m;
    float wheelbase_m;  /* L：车体中心到前/后轮接地点距离，单位 m。 */
    float trackwidth_m; /* W：车体中心到左/右轮接地点距离，单位 m。 */
  } physical;
  struct {
    float max_vx;
    float max_vy;
    float max_wz;
    float max_torque_cmd;
  } limit;
  struct {
    float sample_freq;
    float wheel_start_accel_mps2; /* 轮速参考软启动加速度限幅；<=0 禁用。 */
    float wheel_static_friction_nm; /* 单轮静摩擦力矩前馈；<=0 禁用。 */
    float wheel_static_friction_deadband_mps; /* 静摩擦前馈使用的参考轮速死区。 */
    float wheel_high_pole_pid_switch_lift; /* 任一撑杆高度超过该值时切换高撑杆轮速 PID，单位 rad；<=0 禁用。 */
  } controller;
  struct {
    float lateral_vy_to_wz_feedforward; /* wz_ff = gain * vy，单位 rad/m。 */
    bool lateral_heading_hold_enable;   /* 横移时启用陀螺仪 yaw 保持。 */
    float lateral_heading_hold_kp;      /* yaw 误差到 wz 修正量的增益。 */
    float lateral_heading_hold_kd;      /* 陀螺仪 z 轴阻尼增益。 */
    float lateral_heading_hold_max_wz;  /* 陀螺仪最大修正角速度，单位 rad/s。 */
    float lateral_heading_hold_wz_deadband;    /* 原始 wz 视为 0 的死区。 */
    float lateral_heading_hold_error_deadband;  /* yaw 误差死区，单位 rad。 */
  } front_omni_rear_mecanum;
  MOTOR_TemperatureProtectionConfig_t motor_temperature_protection;
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
    bool temperature_warning;
    bool temperature_over_limit;
    bool temperature_limit_latched;
    bool online;
  } motor[4];
  float encoder_gimbalYawMotor;
  MoveVector_t chassis_vel;
} Chassis_Feedback_t;

typedef struct {
  Chassis_Mode_t mode;
  float dt_s;
  MoveVector_t target_vec;
  MoveVector_t feedback_vec;
  MoveVector_t output_vec;
  float wheel_speed_ref_mps[4];
  float wheel_speed_fdb_mps[4];
  bool wheel_high_pole_pid_active;
  float wheel_pid_error[4];
  float wheel_pid_integral[4];
  float wheel_torque_pid_out[4];
  float wheel_torque_cmd_nm[4];
} Chassis_Debug_t;

typedef struct {
  float motor[4];
  int8_t set_torque_ret[4];
  int8_t controller_update_ret[4];
  int8_t commit_ret[4];
  bool command_pending[4];
  bool last_commit_ok[4];
} Chassis_Output_t;

/* PC上位机接口 */
typedef struct {
  bool pc_control_enable;     /* PC控制使能 */
  float pc_vx;               /* 前向为正的速度命令，单位 m/s。 */
  float pc_vy;               /* 左向为正的速度命令，单位 m/s。 */
  float pc_wz;               /* 逆时针为正的 yaw 角速度命令，单位 rad/s。 */
} Chassis_PCInterface_t;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/* PC接口函数声明 */
void Chassis_InitPCInterface(Chassis_PCInterface_t *pc_if);
void Chassis_SetPCCommand(float vx, float vy, float wz);
const Chassis_PCInterface_t* Chassis_GetPCInterface(void);
void Chassis_FillPCFeedback(void *fb);
#endif
