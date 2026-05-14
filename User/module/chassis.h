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
    float wheelbase_m;
    float trackwidth_m;
    float wheel_output_max_speed;
  } physical;
  struct {
    float max_vx;
    float max_vy;
    float max_wz;
    float max_torque_cmd;
  } limit;
  struct {
    float sample_freq;
    float position_to_velocity_limit;
    float velocity_to_torque_limit;
    float lateral_vy_to_wz_feedforward; /* wz_ff = gain * vy, rad/m. */
    bool lateral_heading_hold_enable;   /* Enable gyro yaw hold during lateral move. */
    float lateral_heading_hold_kp;       /* yaw error to wz correction gain. */
    float lateral_heading_hold_kd;       /* gyro z damping gain. */
    float lateral_heading_hold_max_wz;   /* max gyro correction, rad/s. */
    float lateral_heading_hold_wz_deadband;    /* raw wz considered zero. */
    float lateral_heading_hold_error_deadband; /* yaw error deadband, rad. */
  } controller;
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
  float dt_s;
  float gimbal_beta_rad;
  MoveVector_t cmd_vec_raw;
  MoveVector_t cmd_vec_body;
  MoveVector_t cmd_vec_limited;
  float lateral_wz_feedforward;
  bool lateral_heading_hold_enabled;
  bool lateral_heading_hold_active;
  float lateral_heading_error_rad;
  float lateral_yaw_rate_rad_s;
  float lateral_heading_wz_correction;
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
  float pc_vx;               /* +forward velocity command, m/s */
  float pc_vy;               /* +left velocity command, m/s */
  float pc_wz;               /* +counter-clockwise yaw rate, rad/s */
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
