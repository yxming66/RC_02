#pragma once

// 定义各电机 kind/model 组合的静态能力与参数表。
// 这里集中描述减速比、控制能力、量程等编译期特性。
// 具体型号只需要覆盖“确实有意义”的参数；未覆盖项沿用基类默认值。

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
struct MotorModelValid<MotorKind::DM, MotorModel::J4310P> : std::true_type {};
template <>
struct MotorModelValid<MotorKind::DM, MotorModel::J4340> : std::true_type {};
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
    static constexpr MotorKind kKind = KindValue;                    // 电机种类（协议类别），如 RM / DM / LZ。
    static constexpr MotorModel kModel = ModelValue;                // 电机型号，如 M3508 / J4310 / RSO3。
    static constexpr float kGearRatio = 1.0f;                       // 电机内部减速比；输出轴物理量换算时会与外部减速比共同参与计算，无减速时为 1.0。
    static constexpr bool kSupportsMit = false;                     // 是否支持 MIT（位置/速度/Kp/Kd/前馈力矩）复合控制接口。
    static constexpr bool kSupportsTorque = false;                  // 是否支持力矩/电流控制接口。
    static constexpr bool kSupportsVelocity = false;                // 是否支持速度控制接口。
    static constexpr bool kSupportsPosition = false;                // 是否支持位置控制接口。
    static constexpr bool kIsMultiTurn = false;                     // 反馈位置是否天然支持多圈连续语义，或可由协议层稳定累计为多圈。
    static constexpr bool kHasNativeZeroSet = false;                // 电机协议是否原生支持设零/保存零点。
    static constexpr bool kHasHostId = false;                       // 运行时配置中是否需要 host_id 一类主机标识。
    static constexpr bool kHasMasterId = false;                     // 运行时配置中是否需要 master_id 一类主控标识。
    static constexpr float kDefaultKp = 0.0f;                       // 默认比例增益 Kp，通常供 MIT 或上层控制器初始化参考。
    static constexpr float kDefaultKd = 0.0f;                       // 默认微分增益 Kd，通常供 MIT 或上层控制器初始化参考。
    static constexpr float kMinPosition = 0.0f;                     // 位置下限，单位 rad；为 0 表示未给出限制。
    static constexpr float kMaxPosition = 0.0f;                     // 位置上限，单位 rad；为 0 表示未给出限制。
    static constexpr float kMaxTorque = 0.0f;                       // 最大输出力矩，单位 N·m；通常用于命令裁剪。
    static constexpr float kMaxVelocity = 0.0f;                     // 最大输出角速度，单位 rad/s；通常用于命令裁剪。
    static constexpr float kRecommendedCurrent = 0.0f;              // 推荐持续工作电流，单位 A。
    static constexpr float kRatedCurrent = 0.0f;                    // 额定电流，单位 A。
    static constexpr float kPeakCurrent = 0.0f;                     // 峰值电流，单位 A。
    static constexpr float kRawCurrentRange = 0.0f;                 // 协议原始电流指令/反馈满量程绝对值，例如 16384 / 10000 / 30000。
    static constexpr float kCurrentRangeAmp = 0.0f;                 // 原始电流满量程映射到的物理电流绝对值，单位 A。
    static constexpr float kRecommendedVelocity = 0.0f;             // 推荐持续工作角速度，单位 rad/s。
    static constexpr float kRatedVelocity = 0.0f;                   // 额定角速度，单位 rad/s。
    static constexpr float kNoLoadVelocity = 0.0f;                  // 空载最大角速度，单位 rad/s。
    static constexpr float kRatedTorque = 0.0f;                     // 额定输出力矩，单位 N·m。
    static constexpr float kPeakTorque = 0.0f;                      // 峰值输出力矩，单位 N·m。
    static constexpr float kTorqueConstant = 0.0f;                  // 电机轴（转子轴）力矩常数；仅适用于需要固定电流-力矩换算的型号。
    static constexpr uint16_t kEncoderCpr = 0;                      // 编码器每圈计数（counts per revolution）；未知时为 0。
    static constexpr MotorCapability kCapabilities = MotorCapability::None; // 编译期能力位掩码，汇总支持的控制能力。
    static constexpr const char* kName = "Unknown";                // 人类可读型号名，便于调试输出。
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
    static constexpr float kRawCurrentRange = 10000.0f;
    static constexpr float kCurrentRangeAmp = 10.0f;
    static constexpr float kRecommendedVelocity = 43.56f;
    static constexpr float kRatedVelocity = 43.56f;
    static constexpr float kNoLoadVelocity = 52.36f;
    static constexpr float kRatedTorque = 1.0f;
    static constexpr float kPeakTorque = 1.8f;
    static constexpr float kTorqueConstant = 0.005f;
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
    static constexpr float kRawCurrentRange = 16384.0f;
    static constexpr float kCurrentRangeAmp = 20.0f;
    static constexpr float kRecommendedVelocity = 49.11f;
    static constexpr float kRatedVelocity = 49.11f;
    static constexpr float kNoLoadVelocity = 50.47f;
    static constexpr float kRatedTorque = 3.0f;
    static constexpr float kPeakTorque = 4.5f;
    static constexpr float kTorqueConstant = 0.015622389f;
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
    static constexpr float kRawCurrentRange = 30000.0f;
    static constexpr float kCurrentRangeAmp = 3.0f;
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
    static constexpr bool kIsMultiTurn = true;
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
    static constexpr float kPeakTorque = 12.0f;
    static constexpr MotorCapability kCapabilities = MotorCapability::Current | MotorCapability::Velocity | MotorCapability::Position | MotorCapability::MIT;
    static constexpr const char* kName = "DM-J4310";
};

template <>
struct MotorTraits<MotorKind::DM, MotorModel::J4310P> : MotorTraitsBase<MotorKind::DM, MotorModel::J4310P> {
    static constexpr MOTOR_DM_Module_t kVendorModule = MOTOR_DM_J4310P;
    static constexpr float kGearRatio = 10.0f;
    static constexpr bool kSupportsMit = true;
    static constexpr bool kSupportsTorque = true;
    static constexpr bool kSupportsVelocity = true;
    static constexpr bool kSupportsPosition = true;
    static constexpr bool kIsMultiTurn = true;
    static constexpr bool kHasMasterId = true;
    static constexpr float kDefaultKp = 0.0f;
    static constexpr float kDefaultKd = 0.0f;
    static constexpr float kMaxTorque = 12.5f;
    static constexpr float kMaxVelocity = 200.0f * 2.0f * 3.14159265358979323846f / 60.0f;
    static constexpr float kRecommendedCurrent = 3.1f;
    static constexpr float kRatedCurrent = 4.9f;
    static constexpr float kPeakCurrent = 20.0f;
    static constexpr float kRecommendedVelocity = 120.0f * 2.0f * 3.14159265358979323846f / 60.0f;
    static constexpr float kRatedVelocity = 120.0f * 2.0f * 3.14159265358979323846f / 60.0f;
    static constexpr float kNoLoadVelocity = 200.0f * 2.0f * 3.14159265358979323846f / 60.0f;
    static constexpr float kRatedTorque = 3.5f;
    static constexpr float kPeakTorque = 12.5f;
    static constexpr float kTorqueConstant = 3.5f / 4.9f / 10.0f;
    static constexpr uint16_t kEncoderCpr = 65535;
    static constexpr MotorCapability kCapabilities = MotorCapability::Current | MotorCapability::Velocity | MotorCapability::Position | MotorCapability::MIT;
    static constexpr const char* kName = "DM-J4310P-24V";
};

template <>
struct MotorTraits<MotorKind::DM, MotorModel::J4340> : MotorTraitsBase<MotorKind::DM, MotorModel::J4340> {
    static constexpr MOTOR_DM_Module_t kVendorModule = MOTOR_DM_J4340;
    static constexpr float kGearRatio = 40.0f;
    static constexpr bool kSupportsMit = true;
    static constexpr bool kSupportsTorque = true;
    static constexpr bool kSupportsVelocity = true;
    static constexpr bool kSupportsPosition = true;
    static constexpr bool kIsMultiTurn = true;
    static constexpr bool kHasMasterId = true;
    static constexpr float kDefaultKp = 0.0f;
    static constexpr float kDefaultKd = 0.0f;
    static constexpr float kMaxTorque = 40.0f;
    static constexpr float kMaxVelocity = 112.0f * 2.0f * 3.14159265358979323846f / 60.0f;
    static constexpr float kRecommendedCurrent = 3.1f;
    static constexpr float kRatedCurrent = 4.11f;
    static constexpr float kPeakCurrent = 19.85f;
    static constexpr float kRecommendedVelocity = 36.0f * 2.0f * 3.14159265358979323846f / 60.0f;
    static constexpr float kRatedVelocity = 36.0f * 2.0f * 3.14159265358979323846f / 60.0f;
    static constexpr float kNoLoadVelocity = 56.0f * 2.0f * 3.14159265358979323846f / 60.0f;
    static constexpr float kRatedTorque = 14.0f;
    static constexpr float kPeakTorque = 40.0f;
    static constexpr float kEncoderCpr = 16384;
    static constexpr MotorCapability kCapabilities = MotorCapability::Current | MotorCapability::Velocity | MotorCapability::Position | MotorCapability::MIT;
    static constexpr const char* kName = "DM-J4340-2EC-24V";
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
    static constexpr float kPeakTorque = 60.0f;
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
