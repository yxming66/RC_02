#include "device/motor/factory/motor_factory.hpp"

namespace mrobot::motor {

template <typename T>
union DriverSlot {
    DriverSlot() {}
    ~DriverSlot() {}
    T value;
};

uint8_t MotorFactory::rm_count_ = 0;
uint8_t MotorFactory::dm_count_ = 0;
uint8_t MotorFactory::lz_count_ = 0;

MotorHandle MotorFactory::CreateRm(const MOTOR_RM_Param_t& param, const MotorSpec& spec, const MotorInstallSpec& install) {
    static DriverSlot<RmMotorDriver> drivers[kMaxRmMotorDrivers];
    if (rm_count_ >= kMaxRmMotorDrivers) {
        return {nullptr};
    }
    RmMotorDriver* driver = new (&drivers[rm_count_].value) RmMotorDriver(param, spec, install);
    ++rm_count_;
    return {driver};
}

MotorHandle MotorFactory::CreateDm(const MOTOR_DM_Param_t& param, const MotorSpec& spec, const MotorInstallSpec& install) {
    static DriverSlot<DmMotorDriver> drivers[kMaxDmMotorDrivers];
    if (dm_count_ >= kMaxDmMotorDrivers) {
        return {nullptr};
    }
    DmMotorDriver* driver = new (&drivers[dm_count_].value) DmMotorDriver(param, spec, install);
    ++dm_count_;
    return {driver};
}

MotorHandle MotorFactory::CreateLz(const MOTOR_LZ_Param_t& param, const MotorSpec& spec, const MotorInstallSpec& install) {
    static DriverSlot<LzMotorDriver> drivers[kMaxLzMotorDrivers];
    if (lz_count_ >= kMaxLzMotorDrivers) {
        return {nullptr};
    }
    LzMotorDriver* driver = new (&drivers[lz_count_].value) LzMotorDriver(param, spec, install);
    ++lz_count_;
    return {driver};
}

} // namespace mrobot::motor
