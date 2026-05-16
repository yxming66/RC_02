/*
 * Arm Simple: DM4310关节1 + JS6660舵机关节2 + 电磁阀吸盘
 */

#include "module/arm_simple.h"
#include "bsp/pwm.h"
#include "bsp/gpio.h"
#include <math.h>

static float ServoAngleToPulse(float angle_rad)
{
    /* 将角度映射到PWM脉宽: -270° ~ +270° 对应 0.5ms ~ 2.5ms */
    float normalized = (angle_rad - SERVO_JS6660_MIN_ANGLE_RAD) /
                       (SERVO_JS6660_MAX_ANGLE_RAD - SERVO_JS6660_MIN_ANGLE_RAD);
    normalized = (normalized < 0.0f) ? 0.0f : (normalized > 1.0f) ? 1.0f : normalized;
    float pulse_us = SERVO_JS6660_PULSE_MIN_US +
                     normalized * (SERVO_JS6660_PULSE_MAX_US - SERVO_JS6660_PULSE_MIN_US);
    return pulse_us;
}

static float ClampAngle(float angle, float min_angle, float max_angle)
{
    if (angle < min_angle) return min_angle;
    if (angle > max_angle) return max_angle;
    return angle;
}

int8_t ArmSimple_Init(ArmSimple_t *a, ArmSimple_Params_t *param, float target_freq)
{
    if (a == NULL || param == NULL) {
        return ARM_SIMPLE_ERR;
    }

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

    /* 初始化PID */
    KPID_Init(&a->pid.joint1_pos, &param->pid.joint1_pos, target_freq);
    KPID_Init(&a->pid.joint1_vel, &param->pid.joint1_vel, target_freq);

    /* 初始化舵机PWM */
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
    a->cmd.point_mode = ARM_SIMPLE_POINT_SLEEP;
    a->cmd.suction = SUCTION_OFF;

    return ARM_SIMPLE_OK;
}

int8_t ArmSimple_UpdateFeedback(ArmSimple_t *a)
{
    if (a == NULL) {
        return ARM_SIMPLE_ERR;
    }

    MOTOR_DM_t *motor = MOTOR_DM_GetMotor(&a->param->dm4310_param);
    if (motor != NULL) {
        a->feedback.joint1_angle = motor->motor.feedback.rotor_abs_angle;
        if (a->param->dm4310_param.reverse) {
            a->feedback.joint1_angle = -a->feedback.joint1_angle;
        }
        a->feedback.joint1_vel = motor->motor.feedback.rotor_speed;
        if (a->param->dm4310_param.reverse) {
            a->feedback.joint1_vel = -a->feedback.joint1_vel;
        }
    }

    return ARM_SIMPLE_OK;
}

int8_t ArmSimple_Control(ArmSimple_t *a, const ArmSimple_CMD_t *cmd)
{
    if (a == NULL || cmd == NULL) {
        return ARM_SIMPLE_ERR;
    }

    a->cmd = *cmd;
    a->mode = cmd->mode;

    switch (cmd->mode) {
        case ARM_SIMPLE_MODE_RELAX:
            ArmSimple_Relax(a);
            break;

        case ARM_SIMPLE_MODE_JOINT:
            /* 关节角度控制 */
            a->target.joint1_target = ClampAngle(cmd->target_joint.joint1,
                                                  a->param->soft_limit.joint1_min,
                                                  a->param->soft_limit.joint1_max);
            a->target.joint2_target = ClampAngle(cmd->target_joint.joint2,
                                                  a->param->soft_limit.joint2_min,
                                                  a->param->soft_limit.joint2_max);
            {
                float joint1_err = a->target.joint1_target - a->feedback.joint1_angle;
                float vel_output = KPID_Calculate(&a->pid.joint1_pos, joint1_err);
                vel_output = (vel_output > a->param->vel_limit.joint1_max_vel) ?
                              a->param->vel_limit.joint1_max_vel : vel_output;
                vel_output = (vel_output < -a->param->vel_limit.joint1_max_vel) ?
                              -a->param->vel_limit.joint1_max_vel : vel_output;
                a->target.joint1_vel_target = vel_output;
            }
            break;

        case ARM_SIMPLE_MODE_POS_VEL:
            /* 位置+速度控制 */
            a->target.joint1_target = ClampAngle(cmd->target_joint.joint1,
                                                  a->param->soft_limit.joint1_min,
                                                  a->param->soft_limit.joint1_max);
            a->target.joint2_target = ClampAngle(cmd->target_joint.joint2,
                                                  a->param->soft_limit.joint2_min,
                                                  a->param->soft_limit.joint2_max);
            a->target.joint1_vel_target = cmd->joint1_vel;
            break;

        default:
            break;
    }

    /* 点对点姿态 */
    if (cmd->point_mode < ARM_SIMPLE_POINT_NONE) {
        a->target.joint1_target = a->point2point[cmd->point_mode].joint1_pos;
        a->target.joint2_target = a->point2point[cmd->point_mode].joint2_pos;
    }

    /* 吸盘控制 */
    ArmSimple_SetSuction(a, cmd->suction);

    return ARM_SIMPLE_OK;
}

int8_t ArmSimple_Output(ArmSimple_t *a)
{
    if (a == NULL) {
        return ARM_SIMPLE_ERR;
    }

    MOTOR_MIT_Output_t mit_output = {0};

    switch (a->mode) {
        case ARM_SIMPLE_MODE_RELAX:
            mit_output.torque = 0.0f;
            mit_output.velocity = 0.0f;
            mit_output.angle = a->feedback.joint1_angle;
            mit_output.kp = 0.0f;
            mit_output.kd = 0.0f;
            break;

        case ARM_SIMPLE_MODE_JOINT:
        case ARM_SIMPLE_MODE_POS_VEL:
            mit_output.torque = 0.0f;
            mit_output.velocity = a->target.joint1_vel_target;
            mit_output.angle = a->target.joint1_target;
            mit_output.kp = 5.0f;
            mit_output.kd = 2.0f;
            break;

        default:
            break;
    }

    MOTOR_DM_MITCtrl(&a->param->dm4310_param, &mit_output);

    /* 舵机输出 */
    float servo_pulse = ServoAngleToPulse(a->target.joint2_target);
    BSP_PWM_SetPulseUs(a->param->servo_param.pwm_channel, (uint32_t)servo_pulse);

    return ARM_SIMPLE_OK;
}

void ArmSimple_Relax(ArmSimple_t *a)
{
    if (a == NULL) return;

    MOTOR_DM_Relax(&a->param->dm4310_param);

    /* 舵机保持当前位置 */
    float servo_pulse = ServoAngleToPulse(a->feedback.joint2_angle);
    BSP_PWM_SetPulseUs(a->param->servo_param.pwm_channel, (uint32_t)servo_pulse);
}

void ArmSimple_SetSuction(ArmSimple_t *a, Suction_State_t state)
{
    if (a == NULL) return;

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
