#pragma once

#include "component/trajectory/online_trapezoid.h"
#include "component/trajectory/trajectory_types.hpp"

namespace mr::comp::traj {

struct OnlineTrapezoidAxisConfig {
  Scalar max_velocity = 0.0f;
  Scalar max_acceleration = 0.0f;
  Scalar position_epsilon = 0.002f;
  Scalar velocity_epsilon_floor = 0.02f;
};

struct OnlineTrapezoidAxisSample {
  Scalar position = 0.0f;
  Scalar velocity = 0.0f;
  bool valid = false;
  bool reached = false;
};

class OnlineTrapezoidAxis {
 public:
  OnlineTrapezoidAxis() = default;

  explicit OnlineTrapezoidAxis(const OnlineTrapezoidAxisConfig& config)
      : config_(config) {}

  void Configure(const OnlineTrapezoidAxisConfig& config) {
    config_ = config;
  }

  void Reset(Scalar velocity = 0.0f) {
    velocity_ = velocity;
  }

  OnlineTrapezoidAxisSample Update(Scalar current,
                                   Scalar target,
                                   Scalar dt) {
    const auto sample = comp_update_trapezoid_axis_f(
        current, target, &velocity_, config_.max_velocity,
        config_.max_acceleration, dt, config_.position_epsilon,
        config_.velocity_epsilon_floor);
    return {sample.position, sample.velocity, sample.valid, sample.reached};
  }

  Scalar velocity() const { return velocity_; }
  const OnlineTrapezoidAxisConfig& config() const { return config_; }

 private:
  OnlineTrapezoidAxisConfig config_{};
  Scalar velocity_ = 0.0f;
};

}  // namespace mr::comp::traj
