#pragma once

#include <array>
#include <cstdint>

#include "device/wheel/wheel.hpp"
#include "module/chassis.h"
#include "robotics/chassis/mecanum_chassis.hpp"

namespace mr::module::chassis {

class MecanumController final {
 public:
  static constexpr uint8_t kWheelCount = 4U;

  int8_t Init(const Chassis_Params_t *param, float target_freq);
  int8_t UpdateFeedback();
  int8_t Control(const Chassis_CMD_t &cmd, uint32_t now);
  void Output();
  void ResetOutput();

  void SetGimbalYaw(float yaw_rad) { feedback_.encoder_gimbalYawMotor = yaw_rad; }
  void SetPoleLift(float front_lift_rad, float rear_lift_rad);

  Chassis_Mode_t mode() const { return mode_; }
  const Chassis_Feedback_t &feedback() const { return feedback_; }
  const Chassis_Debug_t &debug() const { return debug_; }
  const Chassis_Output_t &output() const { return out_; }

 private:
  using Wheel = mr::wheel::RmM3508Wheel;
  using MecanumKinematics = mr::robotics::chassis::MecanumChassis;
  using MecanumGeometry = mr::robotics::chassis::MecanumGeometry;
  using WheelSpeeds = MecanumKinematics::WheelSpeeds;
  using WheelState = mr::wheel::WheelState;

  void ResetRuntime();
  void InitFiltersAndPid(float target_freq);
  void ResetControlStateOnModeChange();
  void ResetWheelVelocityControlState();
  void ResetWheelHoldControlState();
  void ResetWheelSpeedPlanner();
  void UpdateWheelPidSelection();
  KPID_t *ActiveWheelSpeedPid(uint8_t idx);
  int8_t SetMode(Chassis_Mode_t mode, uint32_t now);
  void LimitMoveVector();
  void UpdateBodyVelocityFeedback();
  int8_t ComputeWheelSpeeds();
  float PlanWheelStartSpeed(uint8_t idx, float target_speed_mps);
  bool ShouldHoldZeroCommand() const;
  bool UpdateWheelHoldEntryReady();
  void ResetWheelHoldEntryState();
  void EnterWheelHold();
  void ExitWheelHold();
  bool WheelHoldPositionJumped(uint8_t idx, float position_rad) const;
  int8_t ControlWheelHold();
  int8_t RegisterWheels(float target_freq);
  float WheelSpeedFeedback(uint8_t idx) const;
  float FilteredWheelSpeedFeedback(uint8_t idx);
  float FilteredObservedTorque(uint8_t idx, float torque_nm);
  void StoreWheelState(uint8_t idx, const WheelState &state);
  void StoreWheelDebug(uint8_t idx);
  float CalcDt(uint32_t now);

  const Chassis_Params_t *param_ = nullptr;
  MecanumKinematics kinematics_{};
  Chassis_Mode_t mode_ = CHASSIS_MODE_RELAX;
  std::array<Wheel *, kWheelCount> wheels_{};
  WheelSpeeds wheel_speed_ref_{};
  MoveVector_t move_vec_{};
  Chassis_Feedback_t feedback_{};
  Chassis_Debug_t debug_{};
  Chassis_Output_t out_{};
  std::array<KPID_t, kWheelCount> wheel_pid_{};
  std::array<KPID_t, kWheelCount> wheel_high_pole_pid_{};
  std::array<KPID_t, kWheelCount> wheel_hold_pid_{};
  std::array<float, kWheelCount> wheel_speed_ref_planned_{};
  KPID_t follow_pid_{};
  std::array<LowPassFilter2p_t, kWheelCount> wheel_speed_filter_{};
  std::array<LowPassFilter2p_t, kWheelCount> output_filter_{};
  std::array<LowPassFilter2p_t, kWheelCount> torque_filter_{};
  std::array<LowPassFilter2p_t, 3U> body_velocity_filter_{};
  uint64_t last_wakeup_us_ = 0U;
  float dt_ = 0.0f;
  float mech_zero_ = 0.0f;
  float wz_multi_ = 1.0f;
  float pole_lift_max_rad_ = 0.0f;
  bool wheel_high_pole_pid_active_ = false;
  bool wheel_hold_active_ = false;
  float wheel_hold_still_time_s_ = 0.0f;
  std::array<float, kWheelCount> wheel_hold_position_rad_{};
  std::array<float, kWheelCount> wheel_hold_last_position_rad_{};
  alignas(Wheel) unsigned char wheel_storage_[kWheelCount][sizeof(Wheel)]{};
};

}  // namespace mr::module::chassis
