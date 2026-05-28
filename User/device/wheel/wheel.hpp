#pragma once

#include <stdint.h>

#include "device/device.h"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/packages/controller/motor_controller.hpp"
#include "device/motor_rm.h"

namespace mr::wheel {

struct WheelState {
  float position_rad = 0.0f;
  float angular_velocity_rad_s = 0.0f;
  float linear_velocity_mps = 0.0f;
  float torque_nm = 0.0f;
  float temperature_c = 0.0f;
  bool temperature_warning = false;
  bool temperature_over_limit = false;
  bool temperature_limit_latched = false;
  bool online = false;
};

struct WheelDebugSnapshot {
  bool pending_valid = false;
  float pending_torque_current = 0.0f;
  float last_set_torque_nm = 0.0f;
  int8_t last_set_torque_ret = DEVICE_OK;
  int8_t last_commit_ret = DEVICE_OK;
  bool last_commit_skipped = false;
};

struct RmM3508WheelConfig {
  MOTOR_RM_Param_t motor_param{};
  mr::motor::MotorInstallSpec install{};
  mr::motor::MotorTemperatureProtectionConfig temperature_protection{};
  mr::motor::MotorControllerConfig controller{};
  float radius_m = 0.0f;
};

class RmM3508Wheel final {
 public:
  explicit RmM3508Wheel(const RmM3508WheelConfig& config);

  int8_t Register();
  int8_t Enable();
  int8_t Relax();
  int8_t Update();
  int8_t UpdateFeedback();
  int8_t UpdateCommand();
  int8_t SetTorque(float torque_nm);
  int8_t SetPosition(float position_rad, float max_velocity_rad_s = 0.0f);
  int8_t CommitCommand();
  bool HasPendingCommand() const;
  void ClearPendingCommand();

  const WheelState& State() const { return state_; }
  const WheelDebugSnapshot& Debug() const { return debug_; }
  float RadiusM() const { return radius_m_; }
  bool IsValid() const { return controller_ != nullptr; }

 private:
  using Motor = mr::motor::RmM3508Motor;
  using Controller = mr::motor::MotorControllerT<Motor>;

  static float ResolveRadius(float radius_m);

  void RefreshState();
  void RefreshDebug();

  float radius_m_;
  Motor* motor_;
  Controller* controller_;
  WheelState state_;
  WheelDebugSnapshot debug_;
  alignas(Controller) unsigned char controller_storage_[sizeof(Controller)];
};

}  // namespace mr::wheel
