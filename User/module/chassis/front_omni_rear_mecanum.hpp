#pragma once

#include <array>
#include <cstdint>

#include "device/wheel/wheel.hpp"
#include "module/chassis.h"
#include "robotics/chassis/front_omni_rear_mecanum_chassis.hpp"

namespace mr::module::chassis {

class FrontOmniRearMecanumController final {
 public:
  static constexpr uint8_t kWheelCount = 4U;

  int8_t Init(const Chassis_Params_t *param, float target_freq);
  int8_t UpdateFeedback();
  int8_t Control(const Chassis_CMD_t &cmd, uint32_t now);
  void Output();
  void ResetOutput();

  void SetGimbalYaw(float yaw_rad, float yaw_rate_rad_s = 0.0f) {
    feedback_.encoder_gimbalYawMotor = yaw_rad;
    yaw_rate_rad_s_ = yaw_rate_rad_s;
  }

  Chassis_Mode_t mode() const { return mode_; }
  const Chassis_Feedback_t &feedback() const { return feedback_; }
  const Chassis_Debug_t &debug() const { return debug_; }
  const Chassis_Output_t &output() const { return out_; }

 private:
  using Wheel = mr::wheel::RmM3508Wheel;
  using PlanarVelocity = mr::robotics::chassis::PlanarVelocity;
  using Kinematics = mr::robotics::chassis::FrontOmniRearMecanumChassis;
  using Geometry = mr::robotics::chassis::FrontOmniRearMecanumGeometry;
  using WheelSpeeds = Kinematics::WheelSpeeds;
  using WheelState = mr::wheel::WheelState;

  void ResetRuntime();
  void InitFiltersAndPid(float target_freq);
  void ResetControlStateOnModeChange();
  void ResetWheelVelocityControlState();
  void ResetWheelHoldControlState();
  int8_t SetMode(Chassis_Mode_t mode, uint32_t now);
  void LimitMoveVector();
  void UpdateBodyVelocityFeedback();
  int8_t ComputeWheelSpeeds();
  bool ShouldHoldZeroCommand() const;
  bool ShouldUseLateralYawCorrection(float raw_wz_cmd) const;
  bool ShouldUseLateralHeadingHold(float raw_wz_cmd) const;
  float CalcLateralWzFeedforward() const;
  void EnterWheelHold();
  void ExitWheelHold();
  void EnterLateralHeadingHold();
  void ExitLateralHeadingHold();
  float CalcLateralHeadingHoldWz();
  int8_t ControlWheelHold();
  int8_t RegisterWheels(float target_freq);
  float WheelSpeedFeedback(uint8_t idx) const;
  float FilteredWheelSpeedFeedback(uint8_t idx);
  float FilteredObservedTorque(uint8_t idx, float torque_nm);
  void StoreWheelState(uint8_t idx, const WheelState &state);
  void StoreWheelDebug(uint8_t idx);
  float CalcDt(uint32_t now);

  const Chassis_Params_t *param_ = nullptr;
  Kinematics kinematics_{};
  Chassis_Mode_t mode_ = CHASSIS_MODE_RELAX;
  std::array<Wheel *, kWheelCount> wheels_{};
  WheelSpeeds wheel_speed_ref_{};
  MoveVector_t move_vec_{};
  Chassis_Feedback_t feedback_{};
  Chassis_Debug_t debug_{};
  Chassis_Output_t out_{};
  std::array<KPID_t, kWheelCount> wheel_pid_{};
  std::array<KPID_t, kWheelCount> wheel_hold_pid_{};
  KPID_t follow_pid_{};
  std::array<LowPassFilter2p_t, kWheelCount> wheel_speed_filter_{};
  std::array<LowPassFilter2p_t, kWheelCount> output_filter_{};
  std::array<LowPassFilter2p_t, kWheelCount> torque_filter_{};
  std::array<LowPassFilter2p_t, 3U> body_velocity_filter_{};
  uint32_t last_wakeup_ = 0U;
  float dt_ = 0.0f;
  float mech_zero_ = 0.0f;
  float wz_multi_ = 1.0f;
  float yaw_rate_rad_s_ = 0.0f;
  bool wheel_hold_active_ = false;
  bool lateral_heading_hold_active_ = false;
  float lateral_heading_target_rad_ = 0.0f;
  std::array<float, kWheelCount> wheel_hold_position_rad_{};
  alignas(Wheel) unsigned char wheel_storage_[kWheelCount][sizeof(Wheel)]{};
};

}  // namespace mr::module::chassis
