#pragma once

// 提供常用具体电机类型别名。
// 便于上层直接书写 RmM3508Motor / DmJ4310Motor 等类型名。

#include "device/motor/core/motor_t.hpp"

namespace mrobot::motor {

using RmM2006Motor = MotorT<MotorKind::RM, MotorModel::M2006>;
using RmM3508Motor = MotorT<MotorKind::RM, MotorModel::M3508>;
using RmM6020Motor = MotorT<MotorKind::RM, MotorModel::M6020>;

using DmJ4310Motor = MotorT<MotorKind::DM, MotorModel::J4310>;

using LzRso0Motor = MotorT<MotorKind::LZ, MotorModel::RSO0>;
using LzRso1Motor = MotorT<MotorKind::LZ, MotorModel::RSO1>;
using LzRso2Motor = MotorT<MotorKind::LZ, MotorModel::RSO2>;
using LzRso3Motor = MotorT<MotorKind::LZ, MotorModel::RSO3>;
using LzRso4Motor = MotorT<MotorKind::LZ, MotorModel::RSO4>;
using LzRso5Motor = MotorT<MotorKind::LZ, MotorModel::RSO5>;
using LzRso6Motor = MotorT<MotorKind::LZ, MotorModel::RSO6>;

} // namespace mrobot::motor