#pragma once

#include <stdbool.h>

#include "device/motor/core/motor_capability.hpp"
#include "device/motor/core/motor_types.hpp"

namespace mrobot::motor {

struct MotorSpec {
    Vendor vendor;
    Model model;
    const char* name;

    // 电机本体减速比，定义为: 电机转子角 / 电机自身输出轴角。无减速器时填 1.0f。
    float reducer_ratio;
    // 兼容字段：当前控制器默认限幅仍优先读取这两个值，建议保持为“日常安全工作上限”。
    float max_current;   // 电机转子侧电流/等效电流上限（A 或协议等效电流单位）
    float max_velocity;  // 输出轴最大角速度（rad/s）
    float max_position;  // 输出轴最大角度（rad）

    // 建议/额定/峰值参数。
    float recommended_current;   // 建议长期使用的转子侧电流上限（A 或协议等效电流单位）
    float rated_current;         // 额定连续转子侧电流（A 或协议等效电流单位）
    float peak_current;          // 峰值/极限转子侧电流（A 或协议等效电流单位）

    float recommended_velocity;  // 建议长期使用的输出轴速度上限（rad/s）
    float rated_velocity;        // 额定输出轴速度（rad/s）
    float no_load_velocity;      // 空载输出轴速度（rad/s）

    float rated_torque;          // 额定连续输出力矩（N*m）
    float peak_torque;           // 峰值/堵转输出力矩（N*m）

    // 转矩常数单位 N*m/A，定义在电机转子侧；输出侧力矩 = 电流 * 转矩常数 * reducer_ratio * external_ratio。
    float torque_constant;
    uint16_t encoder_cpr;

    MotorCapability capabilities;
};

struct MotorInstallSpec {
    // 机构额外传动比，定义为: 电机输出轴角 / 机构最终输出角。无额外传动时填 1.0f。
    float external_ratio;
    bool reverse_output;
};

inline constexpr MotorInstallSpec kDirectDriveInstall {
    1.0f,
    false,
};

namespace specs {

// RM 系列这里统一记录为输出轴语义：
// - reducer_ratio: 电机转子角 / 电机自身输出轴角
// - MotorInstallSpec.external_ratio: 电机输出轴角 / 机构最终输出角
// - max_velocity/max_current: 为兼容旧代码，暂保持“默认控制限幅”语义
// - recommended_*: 推荐长期工作值
// - rated_*: 额定连续值
// - peak_*: 峰值/极限值
// - torque_constant: 电机转子侧转矩常数（N*m/A）

inline constexpr MotorSpec kRm2006 {
    Vendor::RM, Model::RM_2006, "RM2006",
    // C610+M2006: 减速比 36:1，额定电流 3A，额定转速 416rpm≈43.56rad/s，转矩常数 0.18N*m/A
    36.0f,
    3.0f, 43.56f, 0.0f,
    3.0f, 3.0f, 10.0f,
    43.56f, 43.56f, 52.36f,
    1.0f, 1.8f,
    0.18f, 8192,
    MotorCapability::Current
};

inline constexpr MotorSpec kRm3508 {
    Vendor::RM, Model::RM_3508, "RM3508",
    // C620+M3508: 减速比 3591/187≈19.20，额定电流 10A，额定转速 469rpm≈49.11rad/s，转矩常数 0.30N*m/A
    19.2032f,
    10.0f, 49.11f, 0.0f,
    10.0f, 10.0f, 20.0f,
    49.11f, 49.11f, 50.47f,
    3.0f, 4.5f,
    0.30f, 8192,
    MotorCapability::Current
};

inline constexpr MotorSpec kRm6020 {
    Vendor::RM, Model::RM_6020, "RM6020",
    // GM6020: 额定电流 1.62A，最大空载转速 320rpm≈33.51rad/s，转矩常数 741mN*m/A
    1.0f,
    1.62f, 33.51f, 0.0f,
    1.62f, 1.62f, 3.0f,
    13.82f, 13.82f, 33.51f,
    1.2f, 1.2f,
    0.741f, 8192,
    MotorCapability::Current
};

inline constexpr MotorSpec kDmJ4310 {
    Vendor::DM, Model::DM_J4310, "DM-J4310",
    1.0f,
    10.0f, 30.0f, 12.5f,
    10.0f, 10.0f, 10.0f,
    30.0f, 30.0f, 30.0f,
    0.0f, 12.0f,
    1.2f, 0,
    MotorCapability::Current | MotorCapability::Velocity | MotorCapability::Position | MotorCapability::MIT
};

inline constexpr MotorSpec kLzRso0 {
    Vendor::LZ, Model::LZ_RSO0, "LZ-RSO0",
    1.0f,
    60.0f, 20.0f, 12.57f,
    60.0f, 60.0f, 60.0f,
    20.0f, 20.0f, 20.0f,
    0.0f, 60.0f,
    1.0f, 0,
    MotorCapability::Current | MotorCapability::Velocity | MotorCapability::Position | MotorCapability::MIT
};

} // namespace specs

} // namespace mrobot::motor
