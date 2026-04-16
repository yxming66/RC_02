#pragma once

// 定义统一电机状态缓存结构。
// 上层控制统一从这里读取输出端位置、速度、力矩、在线与故障状态。

#include <stdint.h>

namespace mrobot::motor {

enum class MotorProtocolState : uint8_t {
    Unregistered = 0,
    Registered,
    Enabled,
    Disabled,
    Relaxed,
    Fault,
};

struct MotorState {
    // 统一为输出端反馈；无减速器时等同于电机本体反馈。
    float position_rad = 0.0f;
    // 输出端单圈角，范围 [-pi, pi)。
    float position_single_turn_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    // 统一为输出端力矩，单位 N*m。
    float torque_nm = 0.0f;
    float temperature_c = 0.0f;
    bool online = false;
    bool command_pending = false;
    bool last_commit_ok = false;
    MotorProtocolState protocol_state = MotorProtocolState::Unregistered;
    uint32_t protocol_status_code = 0;
    uint32_t protocol_fault = 0;
    float device_temperature_c = 0.0f;
};

} // namespace mrobot::motor
