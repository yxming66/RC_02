#pragma once

// 电机控制器封装层。
//
// 构建方式:
//   1. 默认参数: MotorControllerT<RmM2006Motor> controller(motor);
//      默认参数来自 motor_controller_defaults.hpp 中的 MotorControllerDefaults<MotorType> 特化。
//   2. 自定义参数: MotorControllerT<RmM2006Motor> controller(motor, config);
//      config 中的 PID 指针必须指向生命周期长于 controller 的静态/全局/外部对象。
//
// 推荐使用流程:
//   controller.Initialize();                    // Register + Enable
//   controller.Step([](auto& ctrl) {            // 周期调用: Update -> 设置命令 -> Commit
//       ctrl.SetPosition(target_rad);
//   });
//
// 精细控制流程仍然保留:
//   controller.Update();                        // 先刷新反馈和控制律
//   ...                                         // 可插入限位学习、状态观测、调试逻辑
//   controller.SetVelocity(target_rad_s);        // 选择本周期目标
//   controller.CommitCommand();                  // 周期末提交到底层电机协议
//
// 控制策略: 底层电机支持原生速度/位置/MIT 时优先透传；不支持时回退到软件 PID。

#include <stdint.h>

#include "component/controller/pid.hpp"
#include "component/pid.h"
#include "component/timer.hpp"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/motor.hpp"

namespace mr::motor {

struct MotorControllerConfig {
    // 速度环 PID。用于模拟速度、模拟位置的内环；nullptr 表示不可用。
    const KPID_Params_t* velocity_pid;
    // 位置环 PID。用于模拟位置或位置到力矩；nullptr 表示不可用。
    const KPID_Params_t* position_pid;
    // 控制器采样频率 Hz。<=0 时回退到 PID 库默认频率。
    float sample_freq;

    // 位置环输出速度目标的限幅 rad/s。<=0 时按电机 Traits 推荐/额定/最大速度回退。
    float position_to_velocity_limit;
    // 速度环输出力矩目标的限幅 N*m。<=0 时按电机 Traits 电流/力矩参数回退。
    float velocity_to_torque_limit;
    // 反馈位置和速度的一阶低通截止频率 Hz。<=0 时禁用反馈滤波。
    float feedback_lowpass_cutoff_hz;
    // 输出力矩的一阶低通截止频率 Hz。<=0 时禁用输出滤波。
    float output_lowpass_cutoff_hz;
};

template <typename MotorType>
struct MotorControllerDefaults {
    static constexpr MotorControllerConfig Config() {
        return {nullptr, nullptr, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }
};

} // namespace mr::motor

#include "device/motor/packages/controller/motor_controller_defaults.hpp"

namespace mr::motor {

template <typename MotorType>
class MotorControllerT final {
public:
    explicit MotorControllerT(MotorType& motor);
    MotorControllerT(MotorType& motor, const MotorControllerConfig& config);

    int8_t Initialize();
    int8_t Tick();

    template <typename CommandFn>
    int8_t Step(CommandFn command_fn) {
        const int8_t update_ret = Update();
        if (update_ret != DEVICE_OK) {
            return update_ret;
        }

        const int8_t command_ret = command_fn(*this);
        if (command_ret != DEVICE_OK) {
            return command_ret;
        }

        return CommitCommand();
    }

    int8_t Register();
    int8_t Enable();
    int8_t Disable();
    int8_t Relax();
    int8_t Update();
    int8_t CommitCommand();
    bool HasPendingCommand() const;
    void ClearPendingCommand();

    int8_t SetTorque(float torque_nm);
    int8_t SetVelocity(float velocity);
    int8_t SetPosition(float position, float max_velocity = 0.0f);
    int8_t SetPositionTorque(float position, float torque_limit = 0.0f);
    int8_t SetMIT(float position, float velocity, float kp, float kd, float torque_ff);

    MotorState GetState() const;
    const MotorInstallSpec& GetInstallConfig() const;
    const MotorType& GetMotor() const;
    float GetLastFeedbackPosition() const;
    float GetLastFeedbackVelocity() const;
    float GetLastOutputTorque() const;
    float CalculatePositionOutput(float target, float feedback, float dt_s);
    float CalculateVelocityOutput(float target, float feedback, float dt_s);
    int8_t ResetControllers();

private:
    enum class ControlMode : uint8_t {
        Torque = 0,
        NativeVelocity,
        EmulatedVelocity,
        NativePosition,
        EmulatedPosition,
        EmulatedPositionTorque,
        NativeMit,
        EmulatedMit,
    };

    float UpdateControlDt();
    float ResolveVelocityLimit(float request_limit) const;
    float ResolveTorqueLimit(float request_limit) const;
    float LowpassAlpha(float cutoff_hz, float dt_s) const;
    void ResetFiltersToState();
    MotorState FilterFeedback(const MotorState& state, float dt_s);
    float FilterOutputTorque(float torque_nm, float dt_s);

    MotorType& motor_;
    MotorControllerConfig config_;

    mr::comp::cntlr::pid velocity_pid_;
    mr::comp::cntlr::pid position_pid_;
    mr::comp::timer control_timer_;
    float control_dt_fallback_s_;
    bool velocity_pid_ready_;
    bool position_pid_ready_;
    float feedback_lowpass_cutoff_hz_;
    float output_lowpass_cutoff_hz_;
    bool feedback_filter_initialized_;
    bool output_filter_initialized_;
    float filtered_position_;
    float filtered_velocity_;
    float filtered_output_torque_;

    ControlMode mode_;
    float target_torque_;
    float target_velocity_;
    float target_position_;
    float position_velocity_limit_;
    float velocity_torque_limit_;
    float target_mit_kp_;
    float target_mit_kd_;
    float target_mit_torque_ff_;
    float last_feedback_position_;
    float last_feedback_velocity_;
    float last_output_torque_;
};

using MotorController = MotorControllerT<mr::motor::RmM3508Motor>;

} // namespace mr::motor
