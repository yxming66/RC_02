#pragma once

#include <new>
#include <stdint.h>

#include "device/motor/drivers/dm_motor_driver.hpp"
#include "device/motor/drivers/lz_motor_driver.hpp"
#include "device/motor/drivers/rm_motor_driver.hpp"

namespace mrobot::motor {

inline constexpr uint8_t kMaxRmMotorDrivers = 8;
inline constexpr uint8_t kMaxDmMotorDrivers = 8;
inline constexpr uint8_t kMaxLzMotorDrivers = 8;

struct MotorHandle {
    IMotor* motor = nullptr;

    IMotor* operator->() const { return motor; }
    IMotor& operator*() const { return *motor; }
    IMotor* Get() const { return motor; }
    explicit operator bool() const { return motor != nullptr; }
};

class MotorFactory {
public:
    static MotorHandle CreateRm(const MOTOR_RM_Param_t& param, const MotorSpec& spec, const MotorInstallSpec& install = kDirectDriveInstall);
    static MotorHandle CreateDm(const MOTOR_DM_Param_t& param, const MotorSpec& spec, const MotorInstallSpec& install = kDirectDriveInstall);
    static MotorHandle CreateLz(const MOTOR_LZ_Param_t& param, const MotorSpec& spec, const MotorInstallSpec& install = kDirectDriveInstall);

private:
    static uint8_t rm_count_;
    static uint8_t dm_count_;
    static uint8_t lz_count_;
};

} // namespace mrobot::motor
