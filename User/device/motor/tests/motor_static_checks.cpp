#include "device/motor/core/motor_t.hpp"
#include "device/motor/core/motor_aliases.hpp"

namespace mr::motor::tests {

using RmM2006 = MotorT<MotorKind::RM, MotorModel::M2006>;
using RmM3508 = MotorT<MotorKind::RM, MotorModel::M3508>;
using RmM6020 = MotorT<MotorKind::RM, MotorModel::M6020>;
using DmJ4310 = MotorT<MotorKind::DM, MotorModel::J4310>;
using DmJ4310P = MotorT<MotorKind::DM, MotorModel::J4310P>;
using DmJ4340 = MotorT<MotorKind::DM, MotorModel::J4340>;
using LzRso3 = MotorT<MotorKind::LZ, MotorModel::RSO3>;

static_assert(MotorModelValidV<MotorKind::RM, MotorModel::M2006>, "RM + M2006 should be valid");
static_assert(MotorModelValidV<MotorKind::RM, MotorModel::M3508>, "RM + M3508 should be valid");
static_assert(MotorModelValidV<MotorKind::RM, MotorModel::M6020>, "RM + M6020 should be valid");
static_assert(MotorModelValidV<MotorKind::DM, MotorModel::J4310>, "DM + J4310 should be valid");
static_assert(MotorModelValidV<MotorKind::DM, MotorModel::J4310P>, "DM + J4310P should be valid");
static_assert(MotorModelValidV<MotorKind::DM, MotorModel::J4340>, "DM + J4340 should be valid");
static_assert(MotorModelValidV<MotorKind::LZ, MotorModel::RSO3>, "LZ + RSO3 should be valid");

static_assert(!MotorModelValidV<MotorKind::RM, MotorModel::J4310>, "RM + J4310 must be invalid");
static_assert(!MotorModelValidV<MotorKind::DM, MotorModel::M3508>, "DM + M3508 must be invalid");
static_assert(!MotorModelValidV<MotorKind::LZ, MotorModel::M6020>, "LZ + M6020 must be invalid");

static_assert(RmM2006::Traits::kSupportsTorque, "RM M2006 should support torque");
static_assert(!RmM2006::Traits::kSupportsMit, "RM M2006 should not support MIT");
static_assert(!RmM2006::Traits::kSupportsPosition, "RM M2006 should not advertise unsupported position API");
static_assert(DmJ4310::Traits::kSupportsMit, "DM J4310 should support MIT");
static_assert(DmJ4310P::Traits::kSupportsMit, "DM J4310P should support MIT");
static_assert(DmJ4340::Traits::kSupportsMit, "DM J4340 should support MIT");
static_assert(LzRso3::Traits::kSupportsPosition, "LZ RSO3 should support position");

static_assert(RmM3508::Traits::kKind == MotorKind::RM, "Traits kind mismatch");
static_assert(DmJ4310::Traits::kModel == MotorModel::J4310, "Traits model mismatch");
static_assert(DmJ4310P::Traits::kModel == MotorModel::J4310P, "Traits model mismatch");
static_assert(DmJ4340::Traits::kModel == MotorModel::J4340, "Traits model mismatch");
static_assert(LzRso3::Traits::kHasHostId, "LZ should expose host id capability");

static_assert(RmM2006::capabilities() == RmM2006::Traits::kCapabilities, "RM2006 capabilities mismatch");
static_assert(RmM3508::capabilities() == RmM3508::Traits::kCapabilities, "RM3508 capabilities mismatch");
static_assert(RmM6020::capabilities() == RmM6020::Traits::kCapabilities, "RM6020 capabilities mismatch");
static_assert(DmJ4310::capabilities() == DmJ4310::Traits::kCapabilities, "DMJ4310 capabilities mismatch");
static_assert(DmJ4310P::capabilities() == DmJ4310P::Traits::kCapabilities, "DMJ4310P capabilities mismatch");
static_assert(DmJ4340::capabilities() == DmJ4340::Traits::kCapabilities, "DMJ4340 capabilities mismatch");

static_assert(RmM2006::peak_current() == RmM2006::Traits::kPeakCurrent, "RM2006 peak current mismatch");
static_assert(RmM3508::peak_current() == RmM3508::Traits::kPeakCurrent, "RM3508 peak current mismatch");
static_assert(DmJ4310::peak_torque() == DmJ4310::Traits::kPeakTorque, "DMJ4310 peak torque mismatch");
static_assert(DmJ4310P::peak_torque() == DmJ4310P::Traits::kPeakTorque, "DMJ4310P peak torque mismatch");
static_assert(DmJ4340::peak_torque() == DmJ4340::Traits::kPeakTorque, "DMJ4340 peak torque mismatch");
static_assert(LzRso3::peak_torque() == LzRso3::Traits::kPeakTorque, "LZ peak torque mismatch");

static_assert(RmM2006::kind() == MotorKind::RM, "RM2006 vendor mismatch");
static_assert(DmJ4310::kind() == MotorKind::DM, "DMJ4310 vendor mismatch");
static_assert(DmJ4310P::kind() == MotorKind::DM, "DMJ4310P vendor mismatch");
static_assert(DmJ4340::kind() == MotorKind::DM, "DMJ4340 vendor mismatch");
static_assert(LzRso3::kind() == MotorKind::LZ, "LZRSO3 vendor mismatch");
static_assert(LzRso3::model() == MotorModel::RSO3, "LZRSO3 model mismatch");

static_assert(std::is_same_v<RmM3508Motor, RmM3508>, "RM alias mismatch");
static_assert(std::is_same_v<DmJ4310Motor, DmJ4310>, "DM alias mismatch");
static_assert(std::is_same_v<DmJ4310PMotor, DmJ4310P>, "DM alias mismatch");
static_assert(std::is_same_v<DmJ4340Motor, DmJ4340>, "DM alias mismatch");
static_assert(std::is_same_v<LzRso3Motor, LzRso3>, "LZ alias mismatch");

} // namespace mr::motor::tests