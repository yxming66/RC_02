#pragma once

namespace mrobot::motor {

struct MotorState {
    // 统一为输出端反馈；无减速器时等同于电机本体反馈。
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    // 统一为输出端力矩，单位 N*m。
    float torque_nm = 0.0f;
    float temperature_c = 0.0f;
    bool online = false;
    bool command_pending = false;
    bool last_commit_ok = false;
};

} // namespace mrobot::motor
