#pragma once

namespace mrobot {

inline const Motor::MotorOps Motor::kLZOps = {
    &Motor::LZRegister,
    &Motor::LZEnable,
    &Motor::LZUpdate,
    &Motor::LZRelax,
    &Motor::LZSetZero,
    &Motor::LZCurrent,
    &Motor::LZVelocity,
    &Motor::LZPosition,
    &Motor::LZMIT,
    &Motor::LZRawMotor,
};

inline const Motor::MotorOps Motor::kDMOps = {
    &Motor::DMRegister,
    &Motor::DMEnable,
    &Motor::DMUpdate,
    &Motor::DMRelax,
    &Motor::DMSetZero,
    &Motor::DMCurrent,
    &Motor::DMVelocity,
    &Motor::DMPosition,
    &Motor::DMMIT,
    &Motor::DMRawMotor,
};

inline const Motor::MotorOps Motor::kRMOps = {
    &Motor::RMRegister,
    &Motor::RMEnable,
    &Motor::RMUpdate,
    &Motor::RMRelax,
    &Motor::RMSetZero,
    &Motor::RMCurrent,
    &Motor::RMVelocity,
    &Motor::RMPosition,
    &Motor::RMMIT,
    &Motor::RMRawMotor,
};

inline const Motor::MotorOps* Motor::ResolveOps(MotorType type) {
    switch (type) {
        case MotorType::LZ:
            return &kLZOps;
        case MotorType::DM:
            return &kDMOps;
        case MotorType::RM:
            return &kRMOps;
        default:
            return nullptr;
    }
}

} // namespace mrobot
