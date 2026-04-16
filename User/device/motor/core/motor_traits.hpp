#pragma once

// 定义各电机 kind/model 组合的静态能力与参数表。
// 这里集中描述减速比、控制能力、力矩常数、量程等编译期特性。

#include <stdint.h>
#include <type_traits>

#include "device/motor/core/motor_capability.hpp"
#include "device/motor/core/motor_kind.hpp"
#include "device/motor/core/motor_model.hpp"
#include "device/motor_dm.h"
#include "device/motor_lz.h"
#include "device/motor_rm.h"

namespace mrobot::motor {

template <MotorKind Kind, MotorModel Model>
struct MotorModelValid : std::false_type {};

template <MotorKind Kind, MotorModel Model>
inline constexpr bool MotorModelValidV = MotorModelValid<Kind, Model>::value;

template <MotorKind Kind, MotorModel Model>
struct MotorTraits;

template <>
struct MotorModelValid<MotorKind::RM, MotorModel::M2006> : std::true_type {};
template <>
struct MotorModelValid<MotorKind::RM, MotorModel::M3508> : std::true_type {};
template <>
struct MotorModelValid<MotorKind::RM, MotorModel::M6020> : std::true_type {};
template <>
struct MotorModelValid<MotorKind::DM, MotorModel::J4310> : std::true_type {};
template <>
struct MotorModelValid<MotorKind::LZ, MotorModel::RSO0> : std::true_type {};
template <>
struct MotorModelValid<MotorKind::LZ, MotorModel::RSO1> : std::true_type {};
template <>
struct MotorModelValid<MotorKind::LZ, MotorModel::RSO2> : std::true_type {};
template <>
struct MotorModelValid<MotorKind::LZ, MotorModel::RSO3> : std::true_type {};
template <>
struct MotorModelValid<MotorKind::LZ, MotorModel::RSO4> : std::true_type {};
template <>
struct MotorModelValid<MotorKind::LZ, MotorModel::RSO5> : std::true_type {};
template <>
struct MotorModelValid<MotorKind::LZ, MotorModel::RSO6> : std::true_type {};

template <MotorKind KindValue, MotorModel ModelValue>
struct MotorTraitsBase {
    static constexpr MotorKind kKind = KindValue;
    static constexpr MotorModel kModel = ModelValue;
    static constexpr float kGearRatio = 1.0f;
    static constexpr bool kSupportsMit = false;
    static constexpr bool kSupportsTorque = false;
    static constexpr bool kSupportsVelocity = false;
    static constexpr bool kSupportsPosition = false;
    static constexpr bool kIsMultiTurn = false;
    static constexpr bool kHasNativeZeroSet = false;
    static constexpr bool kHasHostId = false;
    static constexpr bool kHasMasterId = false;
    static constexpr float kDefaultKp = 0.0f;
    static constexpr float kDefaultKd = 0.0f;
    static constexpr float kMinPosition = 0.0f;
    static constexpr float kMaxPosition = 0.0f;
    static constexpr float kMaxTorque = 0.0f;
    static constexpr float kMaxVelocity = 0.0f;
    static constexpr float kRecommendedCurrent = 0.0f;
    static constexpr float kRatedCurrent = 0.0f;
    static constexpr float kPeakCurrent = 0.0f;
    static constexpr float kRecommendedVelocity = 0.0f;
    static constexpr float kRatedVelocity = 0.0f;
    static constexpr float kNoLoadVelocity = 0.0f;
    static constexpr float kRatedTorque = 0.0f;
    static constexpr float kPeakTorque = 0.0f;
    static constexpr float kTorqueConstant = 0.0f;
    static constexpr uint16_t kEncoderCpr = 0;
    static constexpr MotorCapability kCapabilities = MotorCapability::None;
    static constexpr const char* kName = "Unknown";
};

template <>
struct MotorTraits<MotorKind::RM, MotorModel::M2006> : MotorTraitsBase<MotorKind::RM, MotorModel::M2006> {
    static constexpr MOTOR_RM_Module_t kVendorModule = MOTOR_M2006;
    static constexpr float kGearRatio = 36.0f;
    static constexpr bool kSupportsTorque = true;
    static constexpr bool kIsMultiTurn = true;
    static constexpr float kMaxTorque = 1.8f;
    static constexpr float kMaxVelocity = 43.56f;
    static constexpr float kRecommendedCurrent = 3.0f;
    static constexpr float kRatedCurrent = 3.0f;
    static constexpr float kPeakCurrent = 10.0f;
    static constexpr float kRecommendedVelocity = 43.56f;
    static constexpr float kRatedVelocity = 43.56f;
    static constexpr float kNoLoadVelocity = 52.36f;
    static constexpr float kRatedTorque = 1.0f;
    static constexpr float kPeakTorque = 1.8f;
    static constexpr float kTorqueConstant = 0.18f;
    static constexpr uint16_t kEncoderCpr = 8192;
    static constexpr MotorCapability kCapabilities = MotorCapability::Current;
    static constexpr const char* kName = "RM-M2006";
};

template <>
struct MotorTraits<MotorKind::RM, MotorModel::M3508> : MotorTraitsBase<MotorKind::RM, MotorModel::M3508> {
    static constexpr MOTOR_RM_Module_t kVendorModule = MOTOR_M3508;
    static constexpr float kGearRatio = 3591.0f / 187.0f;
    static constexpr bool kSupportsTorque = true;
    static constexpr bool kIsMultiTurn = true;
    static constexpr float kMaxTorque = 4.5f;
    static constexpr float kMaxVelocity = 49.11f;
    static constexpr float kRecommendedCurrent = 10.0f;
    static constexpr float kRatedCurrent = 10.0f;
    static constexpr float kPeakCurrent = 20.0f;
    static constexpr float kRecommendedVelocity = 49.11f;
    static constexpr float kRatedVelocity = 49.11f;
    static constexpr float kNoLoadVelocity = 50.47f;
    static constexpr float kRatedTorque = 3.0f;
    static constexpr float kPeakTorque = 4.5f;
    static constexpr float kTorqueConstant = 0.30f;
    static constexpr uint16_t kEncoderCpr = 8192;
    static constexpr MotorCapability kCapabilities = MotorCapability::Current;
    static constexpr const char* kName = "RM-M3508";
};

template <>
struct MotorTraits<MotorKind::RM, MotorModel::M6020> : MotorTraitsBase<MotorKind::RM, MotorModel::M6020> {
    static constexpr MOTOR_RM_Module_t kVendorModule = MOTOR_GM6020;
    static constexpr float kGearRatio = 1.0f;
    static constexpr bool kSupportsTorque = true;
    static constexpr bool kIsMultiTurn = true;
    static constexpr float kMaxTorque = 1.2f;
    static constexpr float kMaxVelocity = 33.51f;
    static constexpr float kRecommendedCurrent = 1.62f;
    static constexpr float kRatedCurrent = 1.62f;
    static constexpr float kPeakCurrent = 3.0f;
    static constexpr float kRecommendedVelocity = 13.82f;
    static constexpr float kRatedVelocity = 13.82f;
    static constexpr float kNoLoadVelocity = 33.51f;
    static constexpr float kRatedTorque = 1.2f;
    static constexpr float kPeakTorque = 1.2f;
    static constexpr float kTorqueConstant = 0.741f;
    static constexpr uint16_t kEncoderCpr = 8192;
    static constexpr MotorCapability kCapabilities = MotorCapability::Current;
    static constexpr const char* kName = "RM-M6020";
};

template <>
struct MotorTraits<MotorKind::DM, MotorModel::J4310> : MotorTraitsBase<MotorKind::DM, MotorModel::J4310> {
    static constexpr MOTOR_DM_Module_t kVendorModule = MOTOR_DM_J4310;
    static constexpr bool kSupportsMit = true;
    static constexpr bool kSupportsTorque = true;
    static constexpr bool kSupportsVelocity = true;
    static constexpr bool kSupportsPosition = true;
    static constexpr bool kHasMasterId = true;
    static constexpr float kDefaultKp = 0.0f;
    static constexpr float kDefaultKd = 0.0f;
    static constexpr float kMinPosition = -12.5f;
    static constexpr float kMaxPosition = 12.5f;
    static constexpr float kMaxTorque = 12.0f;
    static constexpr float kMaxVelocity = 30.0f;
    static constexpr float kRecommendedCurrent = 10.0f;
    static constexpr float kRatedCurrent = 10.0f;
    static constexpr float kPeakCurrent = 10.0f;
    static constexpr float kRecommendedVelocity = 30.0f;
    static constexpr float kRatedVelocity = 30.0f;
    static constexpr float kNoLoadVelocity = 30.0f;
    static constexpr float kRatedTorque = 0.0f;
    static constexpr float kPeakTorque = 12.0f;
    static constexpr float kTorqueConstant = 1.2f;
    static constexpr MotorCapability kCapabilities = MotorCapability::Current | MotorCapability::Velocity | MotorCapability::Position | MotorCapability::MIT;
    static constexpr const char* kName = "DM-J4310";
};

template <MotorModel Model>
struct LzRsoTraits : MotorTraitsBase<MotorKind::LZ, Model> {
    static constexpr bool kSupportsMit = true;
    static constexpr bool kSupportsTorque = true;
    static constexpr bool kSupportsVelocity = true;
    static constexpr bool kSupportsPosition = true;
    static constexpr bool kIsMultiTurn = true;
    static constexpr bool kHasNativeZeroSet = true;
    static constexpr bool kHasHostId = true;
    static constexpr float kMaxPosition = 12.57f;
    static constexpr float kMaxTorque = 60.0f;
    static constexpr float kMaxVelocity = 20.0f;
    static constexpr float kRecommendedCurrent = 60.0f;
    static constexpr float kRatedCurrent = 60.0f;
    static constexpr float kPeakCurrent = 60.0f;
    static constexpr float kRecommendedVelocity = 20.0f;
    static constexpr float kRatedVelocity = 20.0f;
    static constexpr float kNoLoadVelocity = 20.0f;
    static constexpr float kRatedTorque = 0.0f;
    static constexpr float kPeakTorque = 60.0f;
    static constexpr float kTorqueConstant = 1.0f;
    static constexpr MotorCapability kCapabilities = MotorCapability::Current | MotorCapability::Velocity | MotorCapability::Position | MotorCapability::MIT;
};

template <> struct MotorTraits<MotorKind::LZ, MotorModel::RSO0> : LzRsoTraits<MotorModel::RSO0> { static constexpr MOTOR_LZ_Module_t kVendorModule = MOTOR_LZ_RSO0; static constexpr const char* kName = "LZ-RSO0"; };
template <> struct MotorTraits<MotorKind::LZ, MotorModel::RSO1> : LzRsoTraits<MotorModel::RSO1> { static constexpr MOTOR_LZ_Module_t kVendorModule = MOTOR_LZ_RSO1; static constexpr const char* kName = "LZ-RSO1"; };
template <> struct MotorTraits<MotorKind::LZ, MotorModel::RSO2> : LzRsoTraits<MotorModel::RSO2> { static constexpr MOTOR_LZ_Module_t kVendorModule = MOTOR_LZ_RSO2; static constexpr const char* kName = "LZ-RSO2"; };
template <> struct MotorTraits<MotorKind::LZ, MotorModel::RSO3> : LzRsoTraits<MotorModel::RSO3> { static constexpr MOTOR_LZ_Module_t kVendorModule = MOTOR_LZ_RSO3; static constexpr const char* kName = "LZ-RSO3"; };
template <> struct MotorTraits<MotorKind::LZ, MotorModel::RSO4> : LzRsoTraits<MotorModel::RSO4> { static constexpr MOTOR_LZ_Module_t kVendorModule = MOTOR_LZ_RSO4; static constexpr const char* kName = "LZ-RSO4"; };
template <> struct MotorTraits<MotorKind::LZ, MotorModel::RSO5> : LzRsoTraits<MotorModel::RSO5> { static constexpr MOTOR_LZ_Module_t kVendorModule = MOTOR_LZ_RSO5; static constexpr const char* kName = "LZ-RSO5"; };
template <> struct MotorTraits<MotorKind::LZ, MotorModel::RSO6> : LzRsoTraits<MotorModel::RSO6> { static constexpr MOTOR_LZ_Module_t kVendorModule = MOTOR_LZ_RSO6; static constexpr const char* kName = "LZ-RSO6"; };

} // namespace mrobot::motor