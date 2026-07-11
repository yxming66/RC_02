/*
 * Arm Simple: DM4340关节1 + JS6660舵机关节2 + 电磁阀吸盘
 */

#include "module/arm_simple.h"
#include "bsp/pwm.h"
#include "bsp/gpio.h"
#include "device/device.h"
#include "device/motor/core/motor_aliases.hpp"
#include "device/motor/core/motor_instance_config.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/factory/motor_factory.hpp"
#include <string.h>
#include <math.h>

using ArmSimpleDmMotor = mr::motor::DmJ4340Motor;

static ArmSimpleDmMotor *ArmSimple_GetDmMotor(ArmSimple_t *a)
{
    return (a == NULL) ? NULL : reinterpret_cast<ArmSimpleDmMotor *>(a->dm_motor);
}

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

static float ArmSimple_ResolveMaxVel(float command_max_vel,
                                     float default_max_vel)
{
    if (command_max_vel > 0.0f && isfinite(command_max_vel)) {
        return command_max_vel;
    }
    return (default_max_vel > 0.0f && isfinite(default_max_vel))
               ? default_max_vel
               : -1.0f;
}

static float ArmSimple_ResolveMaxAccel(float command_max_accel,
                                       float default_max_accel)
{
    if (command_max_accel > 0.0f && isfinite(command_max_accel)) {
        return command_max_accel;
    }
    return (default_max_accel > 0.0f && isfinite(default_max_accel))
               ? default_max_accel
               : -1.0f;
}

static float ArmSimple_MoveToward(float current, float target,
                                  float max_delta)
{
    if (max_delta < 0.0f || !isfinite(max_delta)) {
        return target;
    }

    if (max_delta == 0.0f) {
        return current;
    }

    const float delta = target - current;
    if (fabsf(delta) <= max_delta) {
        return target;
    }
    return current + ((delta > 0.0f) ? max_delta : -max_delta);
}

static float ArmSimple_Sign(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

static float ArmSimple_TrapezoidStep(float current_position,
                                     float target_position,
                                     float *current_velocity,
                                     float max_velocity,
                                     float max_acceleration,
                                     float dt)
{
    if (current_velocity == NULL || dt <= 0.0f || !isfinite(dt)) {
        return target_position;
    }

    if (max_velocity <= 0.0f || max_acceleration <= 0.0f ||
        !isfinite(max_velocity) || !isfinite(max_acceleration)) {
        const float next = ArmSimple_MoveToward(
            current_position, target_position,
            (max_velocity > 0.0f && isfinite(max_velocity))
                ? max_velocity * dt
                : -1.0f);
        *current_velocity = (next - current_position) / dt;
        return next;
    }

    const float error = target_position - current_position;
    const float distance = fabsf(error);
    const float velocity_epsilon = max_acceleration * dt;
    if (distance <= 1.0e-5f && fabsf(*current_velocity) <= velocity_epsilon) {
        *current_velocity = 0.0f;
        return target_position;
    }

    /* Rebuild the minimum-time trapezoid from the current planned state on
     * every cycle.  Unlike following sqrt(2*a*distance) with another
     * acceleration limiter, this analytical sample reaches the target with a
     * continuous velocity even when the behavior target changes mid-motion. */
    const float direction = (error >= 0.0f) ? 1.0f : -1.0f;
    const float directed_current_position = direction * current_position;
    const float directed_target_position = direction * target_position;
    float directed_current_velocity = direction * (*current_velocity);
    directed_current_velocity = ClampFloat(directed_current_velocity,
                                            -max_velocity, max_velocity);

    const float cutoff_begin = directed_current_velocity / max_acceleration;
    const float cutoff_begin_distance =
        0.5f * max_acceleration * cutoff_begin * cutoff_begin;
    float full_distance = cutoff_begin_distance +
                          (directed_target_position - directed_current_position);
    if (full_distance <= 0.0f || !isfinite(full_distance)) {
        const float next_velocity = ArmSimple_MoveToward(
            *current_velocity, 0.0f, max_acceleration * dt);
        const float next_position = current_position +
                                    0.5f * (*current_velocity + next_velocity) * dt;
        *current_velocity = next_velocity;
        return next_position;
    }

    float acceleration_time = max_velocity / max_acceleration;
    float full_speed_distance =
        full_distance - max_acceleration * acceleration_time * acceleration_time;
    if (full_speed_distance < 0.0f) {
        acceleration_time = sqrtf(full_distance / max_acceleration);
        full_speed_distance = 0.0f;
    }

    float end_acceleration = acceleration_time - cutoff_begin;
    if (end_acceleration < 0.0f) {
        end_acceleration = 0.0f;
    }
    const float peak_velocity = max_acceleration * acceleration_time;
    const float end_full_speed = end_acceleration +
        ((peak_velocity > 1.0e-6f) ? full_speed_distance / peak_velocity : 0.0f);
    const float end_deceleration = end_full_speed + acceleration_time;

    float directed_next_position = directed_target_position;
    float directed_next_velocity = 0.0f;
    if (dt < end_acceleration) {
        directed_next_velocity = directed_current_velocity + max_acceleration * dt;
        directed_next_position = directed_current_position +
            (directed_current_velocity + 0.5f * max_acceleration * dt) * dt;
    } else if (dt < end_full_speed) {
        const float acceleration_distance =
            (directed_current_velocity +
             0.5f * max_acceleration * end_acceleration) * end_acceleration;
        directed_next_velocity = peak_velocity;
        directed_next_position = directed_current_position + acceleration_distance +
                                 peak_velocity * (dt - end_acceleration);
    } else if (dt < end_deceleration) {
        const float time_left = end_deceleration - dt;
        directed_next_velocity = max_acceleration * time_left;
        directed_next_position = directed_target_position -
                                 0.5f * max_acceleration * time_left * time_left;
    }

    const float next_position = direction * directed_next_position;
    const float next_velocity = direction * directed_next_velocity;
    const float next_error = target_position - next_position;
    if (ArmSimple_Sign(error) != 0.0f &&
        ArmSimple_Sign(next_error) != ArmSimple_Sign(error)) {
        *current_velocity = 0.0f;
        return target_position;
    }
    *current_velocity = next_velocity;
    return next_position;
}

static void ArmSimple_UpdateOutputTargets(ArmSimple_t *a)
{
    if (a == NULL || a->param == NULL) {
        return;
    }

    if (!a->target.output_target_initialized) {
        a->target.joint1_output_target = ClampAngle(
            a->feedback.joint1_angle,
            a->param->soft_limit.joint1_min,
            a->param->soft_limit.joint1_max);
        a->target.joint2_output_target = ClampAngle(
            a->feedback.joint2_angle,
            a->param->soft_limit.joint2_min,
            a->param->soft_limit.joint2_max);
        a->target.joint1_output_vel = 0.0f;
        a->target.joint2_output_vel = 0.0f;
        a->target.output_target_initialized = true;
    }

    const float dt = (a->timer.dt > 0.0f && isfinite(a->timer.dt))
                         ? a->timer.dt
                         : 0.001f;
    a->target.joint1_output_target = ArmSimple_TrapezoidStep(
        a->target.joint1_output_target,
        a->target.joint1_target,
        &a->target.joint1_output_vel,
        a->target.joint1_max_vel_rad_s,
        a->target.joint1_max_accel_rad_s2,
        dt);
    a->target.joint2_output_target = ArmSimple_TrapezoidStep(
        a->target.joint2_output_target,
        a->target.joint2_target,
        &a->target.joint2_output_vel,
        a->target.joint2_max_vel_rad_s,
        a->target.joint2_max_accel_rad_s2,
        dt);

    a->target.joint1_output_target = ClampAngle(
        a->target.joint1_output_target,
        a->param->soft_limit.joint1_min,
        a->param->soft_limit.joint1_max);
    a->target.joint2_output_target = ClampAngle(
        a->target.joint2_output_target,
        a->param->soft_limit.joint2_min,
        a->param->soft_limit.joint2_max);
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

static mr::motor::MotorTemperatureProtectionConfig ArmSimple_BuildTemperatureProtection(
    const ArmSimple_Params_t *param)
{
    mr::motor::MotorTemperatureProtectionConfig config{};
    if (param == NULL) {
        return config;
    }

    const MOTOR_TemperatureProtectionConfig_t temperature_protection =
        MOTOR_NormalizeTemperatureProtection(param->joint1_temperature_protection);
    config.warning_c = temperature_protection.warning_c;
    config.limit_c = temperature_protection.limit_c;
    config.auto_relax_on_limit =
        temperature_protection.auto_relax_on_limit;
    return config;
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
    a->feedback.joint1_temp = 0.0f;
    a->feedback.joint1_temperature_warning = false;
    a->feedback.joint1_temperature_over_limit = false;
    a->feedback.joint1_temperature_limit_latched = false;
    a->feedback.joint2_angle = 0.0f;

    /* 初始化目标 */
    a->target.joint1_target = 0.0f;
    a->target.joint2_target = 0.0f;
    a->target.joint1_output_target = 0.0f;
    a->target.joint2_output_target = 0.0f;
    a->target.joint1_output_vel = 0.0f;
    a->target.joint2_output_vel = 0.0f;
    a->target.joint1_vel_target = 0.0f;
    a->target.joint1_max_vel_rad_s = ArmSimple_ResolveMaxVel(
        0.0f, param->vel_limit.joint1_max_vel);
    a->target.joint2_max_vel_rad_s = ArmSimple_ResolveMaxVel(
        0.0f, param->vel_limit.joint2_max_vel);
    a->target.joint1_max_accel_rad_s2 = ArmSimple_ResolveMaxAccel(
        0.0f, param->vel_limit.joint1_max_accel);
    a->target.joint2_max_accel_rad_s2 = ArmSimple_ResolveMaxAccel(
        0.0f, param->vel_limit.joint2_max_accel);
    a->target.output_target_initialized = false;

    const auto dm_config =
        mr::motor::MotorInstanceConfig<mr::motor::MotorKind::DM>::FromVendorParam(
            param->dm4340_param);
    ArmSimpleDmMotor *dm_motor =
        mr::motor::MotorFactory::Create<mr::motor::MotorKind::DM,
                                        mr::motor::MotorModel::J4340>(
            dm_config,
            mr::motor::kDirectDriveInstall,
            ArmSimple_BuildTemperatureProtection(param));
    if (dm_motor == NULL || dm_motor->Register() != DEVICE_OK) {
        return ARM_SIMPLE_ERR;
    }
    a->dm_motor = dm_motor;

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

    ArmSimpleDmMotor *motor = ArmSimple_GetDmMotor(a);
    if (motor == NULL) {
        return ARM_SIMPLE_ERR;
    }

    const int8_t ret = motor->Update();
    if (a->mode != ARM_SIMPLE_MODE_RELAX) {
        (void)motor->Enable();
    }
    const mr::motor::MotorState state = motor->GetState();
    a->feedback.joint1_angle = state.position_rad;
    a->feedback.joint1_vel = state.velocity_rad_s;
    a->feedback.joint1_temp = state.temperature_c;
    a->feedback.joint1_temperature_warning = state.temperature_warning;
    a->feedback.joint1_temperature_over_limit = state.temperature_over_limit;
    a->feedback.joint1_temperature_limit_latched =
        state.temperature_limit_latched;
    a->dm_enabled = state.online &&
                    state.protocol_state == mr::motor::MotorProtocolState::Enabled;
 
    return (ret == DEVICE_OK) ? ARM_SIMPLE_OK : ARM_SIMPLE_ERR;
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
        a->target.joint1_output_vel = 0.0f;
        a->target.joint2_output_vel = 0.0f;
        a->target.output_target_initialized = false;
        ArmSimple_SetSuction(a, cmd->suction);
        return ARM_SIMPLE_OK;
    }

    if (!a->target.output_target_initialized) {
        a->target.joint1_output_target = ClampAngle(
            a->feedback.joint1_angle,
            a->param->soft_limit.joint1_min,
            a->param->soft_limit.joint1_max);
        a->target.joint2_output_target = ClampAngle(
            a->feedback.joint2_angle,
            a->param->soft_limit.joint2_min,
            a->param->soft_limit.joint2_max);
        a->target.joint1_output_vel = 0.0f;
        a->target.joint2_output_vel = 0.0f;
        a->target.output_target_initialized = true;
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
    a->target.joint1_max_vel_rad_s = ArmSimple_ResolveMaxVel(
        cmd->joint1_max_vel_rad_s, a->param->vel_limit.joint1_max_vel);
    a->target.joint2_max_vel_rad_s = ArmSimple_ResolveMaxVel(
        cmd->joint2_max_vel_rad_s, a->param->vel_limit.joint2_max_vel);
    a->target.joint1_max_accel_rad_s2 = ArmSimple_ResolveMaxAccel(
        cmd->joint1_max_accel_rad_s2,
        a->param->vel_limit.joint1_max_accel);
    a->target.joint2_max_accel_rad_s2 = ArmSimple_ResolveMaxAccel(
        cmd->joint2_max_accel_rad_s2,
        a->param->vel_limit.joint2_max_accel);

    /* 吸盘控制 */
    ArmSimple_SetSuction(a, cmd->suction);

    return ARM_SIMPLE_OK;
}

int8_t ArmSimple_Output(ArmSimple_t *a)
{
    if (a == NULL || a->param == NULL) {
        return ARM_SIMPLE_ERR;
    }

    ArmSimpleDmMotor *motor = ArmSimple_GetDmMotor(a);
    if (motor == NULL) {
        return ARM_SIMPLE_ERR;
    }

    if (a->mode == ARM_SIMPLE_MODE_RELAX) {
        (void)motor->Relax();
        a->dm_enabled = false;
        a->target.joint1_output_vel = 0.0f;
        a->target.joint2_output_vel = 0.0f;
        a->target.output_target_initialized = false;
    } else {
        ArmSimple_UpdateOutputTargets(a);
        a->feedback.joint2_angle = a->target.joint2_output_target;
        (void)motor->SetMIT(a->target.joint1_output_target,
                            a->target.joint1_output_vel,
                            a->param->mit.joint1_kp,
                            a->param->mit.joint1_kd,
                            a->param->mit.joint1_torque_ff +
                                ArmSimple_Joint1GravityTorque(a));
        (void)motor->CommitCommand();
    }
    BSP_PWM_SetPulseUs(a->param->servo_param.pwm_channel,
                       ArmSimple_AngleToPulseUs(a->target.joint2_output_target, a->param));

    return ARM_SIMPLE_OK;
}

int8_t ArmSimple_SetJoint1Zero(ArmSimple_t *a)
{
    ArmSimpleDmMotor *motor = ArmSimple_GetDmMotor(a);
    if (motor == NULL) {
        return ARM_SIMPLE_ERR;
    }

    return (motor->SetZero() == DEVICE_OK) ? ARM_SIMPLE_OK : ARM_SIMPLE_ERR;
}

void ArmSimple_Relax(ArmSimple_t *a)
{
    if (a == NULL || a->param == NULL) return;

    ArmSimpleDmMotor *motor = ArmSimple_GetDmMotor(a);
    if (motor != NULL) {
        (void)motor->Relax();
    }
    a->mode = ARM_SIMPLE_MODE_RELAX;
    a->dm_enabled = false;
}

void ArmSimple_SetSuction(ArmSimple_t *a, Suction_State_t state)
{
    if (a == NULL || a->param == NULL) return;

    a->suction = state;
    BSP_GPIO_WritePin(a->param->suction_param.gpio, state == SUCTION_ON);
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
    if (param != NULL && isfinite(param->servo_param.center_angle_rad)) {
        angle_rad += param->servo_param.center_angle_rad;
    }

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

bool ArmSimple_MakeBehaviorCommand(const ArmSimple_Params_t *param,
                                   ArmSimple_BehaviorPoint_t point,
                                   Suction_State_t suction,
                                   ArmSimple_CMD_t *cmd)
{
    return ArmSimple_MakeBehaviorCommandWithSpeed(param, point, suction,
                                                  0.0f, 0.0f,
                                                  0.0f, 0.0f, cmd);
}

bool ArmSimple_MakeBehaviorCommandWithSpeed(const ArmSimple_Params_t *param,
                                            ArmSimple_BehaviorPoint_t point,
                                            Suction_State_t suction,
                                            float joint1_max_vel_rad_s,
                                            float joint2_max_vel_rad_s,
                                            float joint1_max_accel_rad_s2,
                                            float joint2_max_accel_rad_s2,
                                            ArmSimple_CMD_t *cmd)
{
    if (param == NULL || cmd == NULL || point >= ARM_SIMPLE_BEHAVIOR_NUM) {
        return false;
    }

    memset(cmd, 0, sizeof(*cmd));
    cmd->mode = ARM_SIMPLE_MODE_JOINT;
    cmd->point_mode = ARM_SIMPLE_POINT_NONE;
    cmd->suction = suction;
    cmd->target_joint.joint1 = param->preset.behavior_point[point].joint1_pos;
    cmd->target_joint.joint2 = param->preset.behavior_point[point].joint2_pos;
    cmd->joint1_vel = 0.0f;
    cmd->joint1_max_vel_rad_s = joint1_max_vel_rad_s;
    cmd->joint2_max_vel_rad_s = joint2_max_vel_rad_s;
    cmd->joint1_max_accel_rad_s2 = joint1_max_accel_rad_s2;
    cmd->joint2_max_accel_rad_s2 = joint2_max_accel_rad_s2;
    return true;
}
