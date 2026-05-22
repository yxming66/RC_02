/*
 * Arm Simple: DM4340关节1 + JS6660舵机关节2 + 电磁阀吸盘
 */

#include "module/arm_simple.h"
#include "bsp/pwm.h"
#include "bsp/gpio.h"
#include <string.h>
#include <math.h>

static float ClampAngle(float angle, float min_angle, float max_angle)
{
    if (angle < min_angle) return min_angle;
    if (angle > max_angle) return max_angle;
    return angle;
}

static uint32_t ClampPulseUs(uint32_t pulse_us)
{
    if (pulse_us < SERVO_JS6660_PULSE_MIN_US) return SERVO_JS6660_PULSE_MIN_US;
    if (pulse_us > SERVO_JS6660_PULSE_MAX_US) return SERVO_JS6660_PULSE_MAX_US;
    return pulse_us;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float ArmSimple_Joint1GravityTorque(const ArmSimple_t *a)
{
    if (a == NULL || a->param == NULL) {
        return 0.0f;
    }

    if (a->mode == ARM_SIMPLE_MODE_RELAX) {
        return 0.0f;
    }

    const float mass_kg = a->param->mit.joint1_gravity_mass_kg;
    const float com_m = a->param->mit.joint1_gravity_com_m;
    if (fabsf(mass_kg) <= 1e-6f || com_m <= 0.0f) {
        return 0.0f;
    }

    const float angle_rad = a->target.joint1_target -
                            a->param->mit.joint1_gravity_zero_rad;
    float torque_nm = mass_kg * 9.80665f * com_m * cosf(angle_rad);
    const float limit_nm = a->param->mit.joint1_gravity_ff_limit_nm;
    if (limit_nm > 0.0f) {
        torque_nm = ClampFloat(torque_nm, -limit_nm, limit_nm);
    }
    return torque_nm;
}

int8_t ArmSimple_Init(ArmSimple_t *a, ArmSimple_Params_t *param, float target_freq)
{
    if (a == NULL || param == NULL || target_freq <= 0.0f) {
        return ARM_SIMPLE_ERR;
    }

    memset(a, 0, sizeof(*a));
    a->param = param;
    a->mode = ARM_SIMPLE_MODE_RELAX;
    a->suction = SUCTION_OFF;

    /* 初始化定时器 */
    a->timer.now = 0.0f;
    a->timer.last_wakeup_us = 0;
    a->timer.dt = 1.0f / target_freq;

    /* 初始化反馈 */
    a->feedback.joint1_angle = 0.0f;
    a->feedback.joint1_vel = 0.0f;
    a->feedback.joint2_angle = 0.0f;

    /* 初始化目标 */
    a->target.joint1_target = 0.0f;
    a->target.joint2_target = 0.0f;
    a->target.joint1_vel_target = 0.0f;

    MOTOR_DM_Register(&param->dm4340_param);

    /* 初始化舵机PWM */
    BSP_PWM_SetFreq(param->servo_param.pwm_channel,
                    (param->servo_param.freq_hz > 0.0f) ? param->servo_param.freq_hz : SERVO_JS6660_DEFAULT_FREQ_HZ);
    BSP_PWM_Start(param->servo_param.pwm_channel);

    /* 初始化吸盘GPIO（默认关闭） */
    BSP_GPIO_WritePin(param->suction_param.gpio, false);

    /* 默认预设姿态（可后续修改） */
    a->point2point[ARM_SIMPLE_POINT_SLEEP].joint1_pos = 0.0f;
    a->point2point[ARM_SIMPLE_POINT_SLEEP].joint2_pos = 0.0f;

    a->point2point[ARM_SIMPLE_POINT_GRAB].joint1_pos = 1.0f;
    a->point2point[ARM_SIMPLE_POINT_GRAB].joint2_pos = -1.57f;  /* -90° */

    a->point2point[ARM_SIMPLE_POINT_LIFT].joint1_pos = 1.5f;
    a->point2point[ARM_SIMPLE_POINT_LIFT].joint2_pos = 0.0f;

    a->point2point[ARM_SIMPLE_POINT_RELEASE].joint1_pos = 1.0f;
    a->point2point[ARM_SIMPLE_POINT_RELEASE].joint2_pos = 1.57f;  /* +90° */

    /* 初始化命令 */
    a->cmd.mode = ARM_SIMPLE_MODE_RELAX;
    a->cmd.point_mode = ARM_SIMPLE_POINT_NONE;
    a->cmd.suction = SUCTION_OFF;

    return ARM_SIMPLE_OK;
}

int8_t ArmSimple_UpdateFeedback(ArmSimple_t *a)
{
    if (a == NULL || a->param == NULL) {
        return ARM_SIMPLE_ERR;
    }

    MOTOR_DM_Update(&a->param->dm4340_param);

    MOTOR_DM_t *motor = MOTOR_DM_GetMotor(&a->param->dm4340_param);
    if (motor != NULL) {
        a->feedback.joint1_angle = motor->motor.feedback.rotor_abs_angle;
        a->feedback.joint1_vel = motor->motor.feedback.rotor_speed;
    }
 
    return ARM_SIMPLE_OK;
}

int8_t ArmSimple_Control(ArmSimple_t *a, const ArmSimple_CMD_t *cmd)
{
    if (a == NULL || a->param == NULL || cmd == NULL) {
        return ARM_SIMPLE_ERR;
    }

    a->cmd = *cmd;
    a->mode = cmd->mode;

    if (cmd->mode == ARM_SIMPLE_MODE_RELAX) {
        a->target.joint1_vel_target = 0.0f;
        ArmSimple_SetSuction(a, cmd->suction);
        return ARM_SIMPLE_OK;
    }

    if (cmd->point_mode < ARM_SIMPLE_POINT_NONE) {
        a->target.joint1_target = a->point2point[cmd->point_mode].joint1_pos;
        a->target.joint2_target = a->point2point[cmd->point_mode].joint2_pos;
    } else {
        a->target.joint1_target = cmd->target_joint.joint1;
        a->target.joint2_target = cmd->target_joint.joint2;
    }
    a->target.joint1_target = ClampAngle(a->target.joint1_target,
                                          a->param->soft_limit.joint1_min,
                                          a->param->soft_limit.joint1_max);
    a->target.joint2_target = ClampAngle(a->target.joint2_target,
                                          a->param->soft_limit.joint2_min,
                                          a->param->soft_limit.joint2_max);
    a->target.joint1_vel_target = 0.0f;
    a->feedback.joint2_angle = a->target.joint2_target;

    /* 吸盘控制 */
    ArmSimple_SetSuction(a, cmd->suction);

    return ARM_SIMPLE_OK;
}

int8_t ArmSimple_Output(ArmSimple_t *a)
{
    if (a == NULL || a->param == NULL) {
        return ARM_SIMPLE_ERR;
    }

    if (a->mode == ARM_SIMPLE_MODE_RELAX) {
        MOTOR_DM_Relax(&a->param->dm4340_param);
    } else {
        MOTOR_MIT_Output_t joint1_output = {
            .torque = a->param->mit.joint1_torque_ff +
                      ArmSimple_Joint1GravityTorque(a),
            .velocity = a->target.joint1_vel_target,
            .angle = a->target.joint1_target,
            .kp = a->param->mit.joint1_kp,
            .kd = a->param->mit.joint1_kd,
        };
        MOTOR_DM_MITCtrl(&a->param->dm4340_param, &joint1_output);
    }
    BSP_PWM_SetPulseUs(a->param->servo_param.pwm_channel,
                       ArmSimple_AngleToPulseUs(a->target.joint2_target, a->param));

    return ARM_SIMPLE_OK;
}

void ArmSimple_Relax(ArmSimple_t *a)
{
    if (a == NULL || a->param == NULL) return;

    MOTOR_DM_Relax(&a->param->dm4340_param);

    a->target.joint2_target = a->feedback.joint2_angle;
}

void ArmSimple_SetSuction(ArmSimple_t *a, Suction_State_t state)
{
    if (a == NULL || a->param == NULL) return;

    a->suction = state;
    BSP_GPIO_WritePin(a->param->suction_param.gpio, (state == SUCTION_ON) ? true : false);
}

bool ArmSimple_Joint1AtTarget(ArmSimple_t *a, float threshold_rad)
{
    if (a == NULL) return false;
    float err = fabsf(a->target.joint1_target - a->feedback.joint1_angle);
    return (err < threshold_rad);
}

bool ArmSimple_Joint2AtTarget(ArmSimple_t *a, float threshold_rad)
{
    if (a == NULL) return false;
    float err = fabsf(a->target.joint2_target - a->feedback.joint2_angle);
    return (err < threshold_rad);
}

uint32_t ArmSimple_AngleToPulseUs(float angle_rad, const ArmSimple_Params_t *param)
{
    if (param != NULL && param->servo_param.reverse) {
        angle_rad = -angle_rad;
    }

    float min_angle = SERVO_JS6660_MIN_ANGLE_RAD;
    float max_angle = SERVO_JS6660_MAX_ANGLE_RAD;
    if (param != NULL) {
        min_angle = param->soft_limit.joint2_min;
        max_angle = param->soft_limit.joint2_max;
    }

    if (min_angle >= 0.0f || max_angle <= 0.0f || min_angle >= max_angle) {
        min_angle = SERVO_JS6660_MIN_ANGLE_RAD;
        max_angle = SERVO_JS6660_MAX_ANGLE_RAD;
    }

    const float clamped = ClampAngle(angle_rad, min_angle, max_angle);
    float pulse_us = (float)SERVO_JS6660_PULSE_NEUTRAL_US;

    if (clamped >= 0.0f) {
        if (fabsf(max_angle) > 1e-6f) {
            pulse_us += (clamped / max_angle) *
                        ((float)SERVO_JS6660_PULSE_MAX_US - (float)SERVO_JS6660_PULSE_NEUTRAL_US);
        }
    } else {
        if (fabsf(min_angle) > 1e-6f) {
            pulse_us -= ((-clamped) / (-min_angle)) *
                        ((float)SERVO_JS6660_PULSE_NEUTRAL_US - (float)SERVO_JS6660_PULSE_MIN_US);
        }
    }

    return ClampPulseUs((uint32_t)(pulse_us + 0.5f));
}
