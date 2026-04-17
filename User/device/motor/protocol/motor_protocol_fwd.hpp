#pragma once

// MotorProtocol 前向声明。
// 用于解开 core 与 protocol 之间的头文件依赖。

#include "device/motor/core/motor_kind.hpp"
#include "device/motor/core/motor_model.hpp"

namespace mrobot::motor {

template <MotorKind Kind, MotorModel Model>
class MotorProtocol;

} // namespace mrobot::motor