/*
 * 底盘模块
 */

#include "cmsis_os2.h"
#include <new>
#include <stdlib.h>
#include "bsp/mm.h"
#include "bsp/can.h"
#include "component/ahrs.h"
#include "device/motor.h"
#include "device/motor/factory/motor_factory.hpp"
#include "device/motor/packages/controller/motor_controller.hpp"
#include "device/motor_rm.h"
#include "module/chassis.h"
#include "config.h"
#include "math.h"

using mrobot::motor::MotorController;
using mrobot::motor::MotorControllerConfig;
using mrobot::motor::MotorFactory;
using mrobot::motor::MotorHandle;
using mrobot::motor::MotorInstallSpec;
using mrobot::motor::MotorState;
using mrobot::motor::specs::kRm3508;

namespace {

alignas(MotorController) static unsigned char g_chassis_controller_storage[4][sizeof(MotorController)];

static MotorHandle& ChassisMotorHandle(Chassis_t *c, uint8_t idx) {
  return *reinterpret_cast<MotorHandle *>(&c->motors[idx]);
}

static MotorController *& ChassisController(Chassis_t *c, uint8_t idx) {
  return *reinterpret_cast<MotorController **>(&c->controllers[idx]);
}

static MotorInstallSpec ChassisInstallSpec(const Chassis_t *c, uint8_t idx) {
  MotorInstallSpec spec {};
  spec.external_ratio = c->param->motor_install[idx].external_ratio;
  spec.reverse_output = c->param->motor_install[idx].reverse_output;
  return spec;
}

static MOTOR_RM_Param_t *ChassisRmParam(Chassis_t *c, uint8_t idx) {
  if (!c || !c->param || idx >= c->num_wheel) return nullptr;
  return &c->param->motor_param[idx];
}

static void ChassisStoreMotorState(Chassis_t *c, uint8_t idx, const MotorState& state) {
  c->feedback.motor[idx].position_rad = state.position_rad;
  c->feedback.motor[idx].velocity_rad_s = state.velocity_rad_s;
  c->feedback.motor[idx].torque_nm = state.torque_nm;
  c->feedback.motor[idx].temperature_c = state.temperature_c;
  c->feedback.motor[idx].online = state.online;
}

}
/**
 * @brief 底盘小陀螺模式相关参数
 */
#define CHASSIS_ROTOR_WZ_MIN 0.6f         // 小陀螺最小角速度
#define CHASSIS_ROTOR_WZ_MAX 0.8f         // 小陀螺最大角速度
#define M_7OVER72PI (M_2PI * 7.0f / 72.0f)  // 35° 偏置角，对应 7/72 圈
#define CHASSIS_ROTOR_OMEGA 0.001f        // 小陀螺角速度变化频率

static void Chassis_LimitMoveVector(Chassis_t *c) {
  if (!c) return;

  /* 车体速度限幅：vx/vy 单位 m/s，wz 单位 rad/s */
  Clip(&c->move_vec.vx, -c->param->limit.max_vx, c->param->limit.max_vx);
  Clip(&c->move_vec.vy, -c->param->limit.max_vy, c->param->limit.max_vy);
  Clip(&c->move_vec.wz, -c->param->limit.max_wz, c->param->limit.max_wz);
}

static float Chassis_GetWheelSpeedFeedback(const Chassis_t *c, uint8_t idx) {
  if (!c || idx >= c->num_wheel) return 0.0f;
  const float wheel_radius = fmaxf(c->param->physical.wheel_radius_m, 1e-4f);
  return c->feedback.motor[idx].velocity_rad_s * wheel_radius;
}

static void Chassis_UpdateBodyVelocityFeedback(Chassis_t *c) {
  if (!c) return;
  ResetMoveVector(&c->feedback.chassis_vel);
  if (c->num_wheel != 4 || c->param->type != CHASSIS_TYPE_MECANUM) return;

  /* 麦轮正运动学：由四轮线速度反算车体速度 */
  const float k = c->param->physical.wheelbase_m + c->param->physical.trackwidth_m;
  const float v0 = Chassis_GetWheelSpeedFeedback(c, 0);
  const float v1 = Chassis_GetWheelSpeedFeedback(c, 1);
  const float v2 = Chassis_GetWheelSpeedFeedback(c, 2);
  const float v3 = Chassis_GetWheelSpeedFeedback(c, 3);

  const float raw_vx = 0.25f * (v0 + v1 - v2 - v3);
  const float raw_vy = 0.25f * (-v0 + v1 + v2 - v3);
  float raw_wz = 0.0f;
  if (fabsf(k) > 1e-4f) {
    raw_wz = -0.25f * (v0 + v1 + v2 + v3) / k;
  }

  /*
   * 当前 MIXER_MECANUM 采用的逆运动学是：
   *   v0 =  vx - vy + k*wz
   *   v1 =  vx + vy + k*wz
   *   v2 = -vx + vy + k*wz
   *   v3 = -vx - vy + k*wz
   *
   * 因此前进/平移反馈都应按 1/4 反解；
   * 旋转项若按右手系约定“逆时针为正”，则需对四轮同号和取反。
   */

  c->feedback.chassis_vel.vx = LowPassFilter2p_Apply(&c->filter.chassis_vel[0], raw_vx);
  c->feedback.chassis_vel.vy = LowPassFilter2p_Apply(&c->filter.chassis_vel[1], raw_vy);
  c->feedback.chassis_vel.wz = LowPassFilter2p_Apply(&c->filter.chassis_vel[2], raw_wz);
}



/**
 * @brief 设置底盘模式
 * @param c 底盘对象
 * @param mode 目标模式
 * @param now 当前时刻(ms)
 * @return CHASSIS_OK:成功   CHASSIS_ERR_NULL:空指针
 */
static int8_t Chassis_SetMode(Chassis_t *c, Chassis_Mode_t mode, uint32_t now) {
    if (!c) 
			return CHASSIS_ERR_NULL;
    if (mode == c->mode)
			return CHASSIS_OK;
// 小陀螺模式切入时随机选择旋转方向
    if (mode == CHASSIS_MODE_ROTOR && c->mode != CHASSIS_MODE_ROTOR) {
        srand(now);
        c->wz_multi = (rand() % 2) ? -1 : 1;
    }
// 重置 PID 与滤波器状态
    for (uint8_t i = 0; i < c->num_wheel; i++) {
        PID_Reset(&c->pid.motor[i]);
        LowPassFilter2p_Reset(&c->filter.in[i], 0.0f);
        LowPassFilter2p_Reset(&c->filter.out[i], 0.0f);
    }
    LowPassFilter2p_Reset(&c->filter.chassis_vel[0], 0.0f);
    LowPassFilter2p_Reset(&c->filter.chassis_vel[1], 0.0f);
    LowPassFilter2p_Reset(&c->filter.chassis_vel[2], 0.0f);

    c->mode = mode;
    return CHASSIS_OK;
}

/**
 * @brief 计算小陀螺模式角速度
 * @param min 最小角速度
 * @param max 最大角速度
 * @param now 当前时刻(ms)
 * @return 角速度(rad/s)
 */
static float Chassis_CalcWz(const float min, const float max, uint32_t now) {
    float wz_vary = fabsf(0.2f * sinf(CHASSIS_ROTOR_OMEGA * (float)now)) + min;
    return (wz_vary > max) ? max : wz_vary;
}

/**
 * @brief 初始化底盘模块
 * @param c 底盘对象
 * @param param 底盘参数
 * @param target_freq 控制频率(Hz)
 * @return CHASSIS_OK:成功 CHASSIS_ERR_NULL:空指针 CHASSIS_ERR_TYPE:类型错误
 */
int8_t Chassis_Init(Chassis_t *c, const Chassis_Params_t *param,
                    float target_freq) {
    if (!c) return CHASSIS_ERR_NULL;
										
      // 初始化 CAN 通信
		BSP_CAN_Init();
    c->param = const_cast<Chassis_Params_t *>(param);
    c->mode = CHASSIS_MODE_RELAX;
  // 根据底盘类型配置轮数与混控模式
    Mixer_Mode_t mixer_mode;
    switch (param->type) {
  		case CHASSIS_TYPE_MECANUM: // 麦克纳姆轮
					c->num_wheel = 4; 
				mixer_mode = MIXER_MECANUM;
				break;
        case CHASSIS_TYPE_PARLFIX4:
					c->num_wheel = 4; 
				mixer_mode = MIXER_PARLFIX4;
				break;
        case CHASSIS_TYPE_PARLFIX2:
					c->num_wheel = 2; 
				mixer_mode = MIXER_PARLFIX2; 
				break;
        case CHASSIS_TYPE_OMNI_CROSS:
					c->num_wheel = 4;
				mixer_mode = MIXER_OMNICROSS;
				break;
    case CHASSIS_TYPE_OMNI_PLUS: // 全向轮（十字布局）
					c->num_wheel = 4;
				mixer_mode = MIXER_OMNIPLUS;
				break;
        case CHASSIS_TYPE_SINGLE:
					c->num_wheel = 1; 
				mixer_mode = MIXER_SINGLE;
				break;
        default: 
					return CHASSIS_ERR_TYPE;
    }
  	// 初始化时基
				c->last_wakeup = 0;
				c->dt = 0.0f;
    // 初始化 PID、滤波器和反馈量
    for (uint8_t i = 0; i < c->num_wheel; i++) {
        PID_Init(&c->pid.motor[i], KPID_MODE_NO_D, target_freq, &param->pid.motor_pid_param);
        LowPassFilter2p_Init(&c->filter.in[i], target_freq, param->low_pass_cutoff_freq.in);
        LowPassFilter2p_Init(&c->filter.out[i], target_freq, param->low_pass_cutoff_freq.out);
    // 清零电机反馈缓存
				c->feedback.motor[i] = {};
        c->setpoint.wheel_speed[i] = 0.0f;
        c->out.motor[i] = 0.0f;
        c->out.set_torque_ret[i] = DEVICE_ERR;
        c->out.controller_update_ret[i] = DEVICE_ERR;
        c->out.commit_ret[i] = DEVICE_ERR;
        c->out.command_pending[i] = false;
        c->out.last_commit_ok[i] = false;
      ChassisController(c, i) = nullptr;
        ChassisMotorHandle(c, i) = MotorHandle{};
    }
    LowPassFilter2p_Init(&c->filter.chassis_vel[0], target_freq, param->low_pass_cutoff_freq.in);
    LowPassFilter2p_Init(&c->filter.chassis_vel[1], target_freq, param->low_pass_cutoff_freq.in);
    LowPassFilter2p_Init(&c->filter.chassis_vel[2], target_freq, param->low_pass_cutoff_freq.in);
    // 初始化跟随 PID 与混控器
    Mixer_Config_t mixer_config = {0};
    mixer_config.mecanum.wheelbase_m = c->param->physical.wheelbase_m;
    mixer_config.mecanum.trackwidth_m = c->param->physical.trackwidth_m;
    mixer_config.output_scale = 1.0f;
    mixer_config.normalize_output = (mixer_mode == MIXER_MECANUM) ? false : true;
    PID_Init(&c->pid.follow, KPID_MODE_CALC_D, target_freq, &param->pid.follow_pid_param);
    Mixer_Init(&c->mixer, mixer_mode, &mixer_config);
    // 清零车体速度命令与输出
    c->move_vec.vx = c->move_vec.vy = c->move_vec.wz = 0.0f;
    ResetMoveVector(&c->feedback.chassis_vel);
		for (uint8_t i = 0; i < c->num_wheel; i++) { 
				c->out.motor[i] = 0.0f;
		}
  // 注册底盘 RM 电机
    for (int i = 0; i < c->num_wheel; i++) {
      ChassisMotorHandle(c, i) = MotorFactory::CreateRm(c->param->motor_param[i], kRm3508, ChassisInstallSpec(c, i));
      if (!ChassisMotorHandle(c, i)) {
        return CHASSIS_ERR_NULL;
      }

      const MotorControllerConfig controller_config = {
        .velocity_pid = &param->pid.motor_pid_param,
        .position_pid = &param->pid.motor_pos_pid_param,
        .sample_freq = (param->controller.sample_freq > 0.0f) ? param->controller.sample_freq : target_freq,
        .position_to_velocity_limit = param->controller.position_to_velocity_limit,
        .velocity_to_torque_limit = param->controller.velocity_to_torque_limit,
      };

      ChassisController(c, i) = new (g_chassis_controller_storage[i]) MotorController(*ChassisMotorHandle(c, i), controller_config);
      if (ChassisController(c, i)->Register() != DEVICE_OK) {
        return CHASSIS_ERR_NULL;
      }
      if (ChassisController(c, i)->Enable() != DEVICE_OK) {
        return CHASSIS_ERR_NULL;
      }
    }
    Chassis_UpdateBodyVelocityFeedback(c);
    return CHASSIS_OK;
}

/**
 * @brief 更新底盘反馈
 * @param c 底盘对象
 * @return CHASSIS_OK:成功 CHASSIS_ERR_NULL:空指针
 */
int8_t Chassis_UpdateFeedback(Chassis_t *c) {
  // 更新所有 RM 电机反馈，并同步车体速度反馈
    for (uint8_t i = 0; i < c->num_wheel; i++) {
      if (ChassisController(c, i) == nullptr) {
        return CHASSIS_ERR_NULL;
      }

      if (ChassisController(c, i)->Update() != DEVICE_OK) {
				return CHASSIS_ERR_NULL;
			}
      ChassisStoreMotorState(c, i, ChassisController(c, i)->GetState());
		}
    Chassis_UpdateBodyVelocityFeedback(c);
    return CHASSIS_OK;
}

/**
 * @brief 底盘控制主流程
 * @param c 底盘对象
 * @param c_cmd 控制命令，vx/vy 单位 m/s，wz 单位 rad/s
 * @param now 当前时刻(ms)
 * @return CHASSIS_OK:成功 CHASSIS_ERR_NULL:空指针
 */
int8_t Chassis_Control(Chassis_t *c, const Chassis_CMD_t *c_cmd, uint32_t now) {
    if (!c || !c_cmd) return CHASSIS_ERR_NULL;
    // 计算控制周期
    c->dt = (float)(now - c->last_wakeup) / 1000.0f; 
    c->last_wakeup = now;
		if (!isfinite(c->dt) || c->dt <= 0.0f) {
				c->dt = 0.001f;            
		}
		if (c->dt < 0.0005f) c->dt = 0.0005f;   
		if (c->dt > 0.050f)  c->dt = 0.050f;    
    // 设置工作模式
    Chassis_SetMode(c, c_cmd->mode, now);
    // 计算车体平移速度命令
    switch (c->mode) {
        case CHASSIS_MODE_BREAK:
            c->move_vec.vx = c->move_vec.vy = 0.0f;
            break;
        case CHASSIS_MODE_INDEPENDENT:
            c->move_vec.vx = c_cmd->ctrl_vec.vx;
            c->move_vec.vy = c_cmd->ctrl_vec.vy;
            break;
        default: {      // 云台/世界系命令转换到底盘车体系
            float beta = c->feedback.encoder_gimbalYawMotor - c->mech_zero;
            float cosb = cosf(beta);
            float sinb = sinf(beta);
            c->move_vec.vx = cosb * c_cmd->ctrl_vec.vx - sinb * c_cmd->ctrl_vec.vy;
            c->move_vec.vy = sinb * c_cmd->ctrl_vec.vx + cosb * c_cmd->ctrl_vec.vy;
            break;
        }
    }
    // 计算车体角速度命令
    switch (c->mode) {
        case CHASSIS_MODE_RELAX:
        case CHASSIS_MODE_BREAK:
        case CHASSIS_MODE_INDEPENDENT:
						c->move_vec.wz = c_cmd->ctrl_vec.wz;
            break;
        case CHASSIS_MODE_OPEN:
						c->move_vec.wz = c_cmd->ctrl_vec.wz;
						break;
        // 云台跟随
        case CHASSIS_MODE_FOLLOW_GIMBAL:
            c->move_vec.wz = PID_Calc(&c->pid.follow, c->mech_zero, c->feedback.encoder_gimbalYawMotor, 0.0f, c->dt);
            break;
        // 云台跟随（带固定偏置）
        case CHASSIS_MODE_FOLLOW_GIMBAL_35:
            c->move_vec.wz = PID_Calc(&c->pid.follow,c->mech_zero +M_7OVER72PI, c->feedback.encoder_gimbalYawMotor, 0.0f, c->dt);
            break;
        // 小陀螺模式
        case CHASSIS_MODE_ROTOR:
            c->move_vec.wz = c->wz_multi * Chassis_CalcWz(CHASSIS_ROTOR_WZ_MIN,CHASSIS_ROTOR_WZ_MAX, now);
            break;
    }
      Chassis_LimitMoveVector(c);

      // 统一通过 Mixer_Apply 输出；麦轮真实单位逆解已收敛到 mixer 内部
      if (Mixer_Apply(&c->mixer, &c->move_vec, c->setpoint.wheel_speed,
                      c->num_wheel) != 0) {
        return CHASSIS_ERR_TYPE;
      }


    for (uint8_t i = 0; i < c->num_wheel; i++) {
        float rf = c->setpoint.wheel_speed[i]; /* 目标轮线速度，单位 m/s */
        float fb = LowPassFilter2p_Apply(&c->filter.in[i], Chassis_GetWheelSpeedFeedback(c, i));
				float out_torque;
        switch (c->mode) {
            case CHASSIS_MODE_BREAK:
            case CHASSIS_MODE_FOLLOW_GIMBAL:
            case CHASSIS_MODE_FOLLOW_GIMBAL_35:
            case CHASSIS_MODE_ROTOR:
            case CHASSIS_MODE_INDEPENDENT:
                out_torque = PID_Calc(&c->pid.motor[i], rf, fb, 0.0f, c->dt);
                break;
            case CHASSIS_MODE_OPEN:
              out_torque = c->setpoint.wheel_speed[i] /
                      fmaxf(c->param->physical.wheel_output_max_speed, 1e-4f);
                break;
            case CHASSIS_MODE_RELAX:
                out_torque = 0.0f;
                break;
        }


       // 当前底盘发送链路走的是 MOTOR_RM_SetOutput()，因此这里的 out.motor
       // 实际语义应视为归一化输出指令，而不是物理力矩 N·m。
         c->out.motor[i] = LowPassFilter2p_Apply(&c->filter.out[i], out_torque);
				 Clip(&c->out.motor[i], -c->param->limit.max_current, c->param->limit.max_current);
    }

    /*
     * 第二阶段：当前 chassis 已经自行完成速度环，直接把结果写到底层 RM 输出缓存。
     * 不再在这里调用 MotorController::Update()，否则会再次尝试取新反馈，
     * 在无新报文的周期上返回 DEVICE_ERR / DEVICE_ERR_NO_DEV，导致控制链抖动。
     */
    for (uint8_t i = 0; i < c->num_wheel; i++) {
      if (ChassisController(c, i) == nullptr || ChassisRmParam(c, i) == nullptr) {
        continue;
      }

      if (c->mode == CHASSIS_MODE_RELAX) {
        ChassisController(c, i)->Relax();
        c->out.motor[i] = 0.0f;
        c->out.set_torque_ret[i] = DEVICE_OK;
        c->out.controller_update_ret[i] = DEVICE_OK;
        c->out.command_pending[i] = false;
        continue;
      }

      c->out.set_torque_ret[i] = DEVICE_OK;
      c->out.controller_update_ret[i] = MOTOR_RM_SetOutput(ChassisRmParam(c, i), c->out.motor[i]);
      if (c->out.controller_update_ret[i] != DEVICE_OK) {
        return CHASSIS_ERR;
      }
      c->out.command_pending[i] = true;
    }

		
    return CHASSIS_OK;
}

/**
 * @brief 下发底盘电机输出
 * @param c 底盘对象
 */
void Chassis_Output(Chassis_t *c) {
    if (!c) 
			return;

    int8_t commit_ret = DEVICE_ERR_NULL;
    if (c->num_wheel > 0 && ChassisRmParam(c, 0) != nullptr) {
      commit_ret = MOTOR_RM_Ctrl(ChassisRmParam(c, 0));
    }

    for (uint8_t i = 0; i < c->num_wheel; i++) {
      c->out.commit_ret[i] = commit_ret;
      c->out.command_pending[i] = false;
      c->out.last_commit_ok[i] = (commit_ret == DEVICE_OK);
    }
}

/**
 * @brief 清空底盘输出
 * @param c 底盘对象
 */
void Chassis_ResetOutput(Chassis_t *c) {
    if (!c) return;
    for (uint8_t i = 0; i < c->num_wheel; i++) {
      if (ChassisController(c, i) != nullptr) {
        ChassisController(c, i)->Relax();
        ChassisController(c, i)->CommitCommand();
      }
    }
}

/**
 * @brief 导出底盘数据
 *
 * @param chassis 底盘数据结构体
 * @param ui UI数据结构体
 */
void Chassis_DumpUI(const Chassis_t *c, Chassis_RefereeUI_t *ui) {
  ui->mode = c->mode;
  ui->angle = c->feedback.encoder_gimbalYawMotor - c->mech_zero;
}
