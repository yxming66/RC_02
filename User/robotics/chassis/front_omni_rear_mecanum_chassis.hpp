#pragma once

#include <array>

#include "component/math/scalar.hpp"
#include "robotics/chassis/mecanum_chassis.hpp"

namespace mr::robotics::chassis {

struct FrontOmniRearMecanumGeometry {
  // 与 MecanumGeometry 约定一致：L/W 为车体中心到车轮接地点中心的距离。
  // 前全向轮只沿车体系 x 方向驱动。
  float wheelbase_m = 0.0f;
  float trackwidth_m = 0.0f;

  float FrontOmniYawRadius() const { return trackwidth_m; }
  float RearMecanumYawRadius() const { return wheelbase_m + trackwidth_m; }

  bool IsValid() const {
    return mr::component::math::is_finite_scalar(wheelbase_m) &&
           mr::component::math::is_finite_scalar(trackwidth_m) &&
           wheelbase_m >= 0.0f && trackwidth_m > 1e-5f &&
           RearMecanumYawRadius() > 1e-5f;
  }
};

enum class FrontOmniRearMecanumWheel : unsigned {
  kFrontRight = 0,
  kFrontLeft = 1,
  kRearLeft = 2,
  kRearRight = 3,
};

class FrontOmniRearMecanumChassis {
 public:
  static constexpr unsigned kWheelCount = 4U;
  using WheelSpeeds = std::array<float, kWheelCount>;

  constexpr FrontOmniRearMecanumChassis() = default;
  explicit constexpr FrontOmniRearMecanumChassis(
      const FrontOmniRearMecanumGeometry& geometry)
      : geometry_(geometry) {}

  void SetGeometry(const FrontOmniRearMecanumGeometry& geometry) {
    geometry_ = geometry;
  }
  const FrontOmniRearMecanumGeometry& geometry() const { return geometry_; }
  bool IsValid() const { return geometry_.IsValid(); }

  bool InverseKinematics(const PlanarVelocity& body_velocity,
                         WheelSpeeds& wheel_speeds) const {
    if (!IsValid() || !IsFinite(body_velocity)) {
      wheel_speeds = {};
      return false;
    }

    const float front_wz_term =
        geometry_.FrontOmniYawRadius() * body_velocity.wz_rad_s;
    const float rear_wz_term =
        geometry_.RearMecanumYawRadius() * body_velocity.wz_rad_s;

    // Wheel Jacobian rows [vx, vy, wz]:
    // FR [-1, 0, -trackwidth], FL [1, 0, -trackwidth],
    // RL [1, 1, -(wheelbase + trackwidth)],
    // RR [-1, 1, -(wheelbase + trackwidth)].
    wheel_speeds[WheelIndex(FrontOmniRearMecanumWheel::kFrontRight)] =
        -body_velocity.vx_mps - front_wz_term;
    wheel_speeds[WheelIndex(FrontOmniRearMecanumWheel::kFrontLeft)] =
        body_velocity.vx_mps - front_wz_term;
    wheel_speeds[WheelIndex(FrontOmniRearMecanumWheel::kRearLeft)] =
        body_velocity.vx_mps + body_velocity.vy_mps - rear_wz_term;
    wheel_speeds[WheelIndex(FrontOmniRearMecanumWheel::kRearRight)] =
        -body_velocity.vx_mps + body_velocity.vy_mps - rear_wz_term;
    return true;
  }

  bool ForwardKinematics(const WheelSpeeds& wheel_speeds,
                         PlanarVelocity& body_velocity) const {
    if (!IsValid() || !IsFinite(wheel_speeds)) {
      body_velocity = {};
      return false;
    }

    const float v0 =
        wheel_speeds[WheelIndex(FrontOmniRearMecanumWheel::kFrontRight)];
    const float v1 =
        wheel_speeds[WheelIndex(FrontOmniRearMecanumWheel::kFrontLeft)];
    const float v2 =
        wheel_speeds[WheelIndex(FrontOmniRearMecanumWheel::kRearLeft)];
    const float v3 =
        wheel_speeds[WheelIndex(FrontOmniRearMecanumWheel::kRearRight)];

    const float front_yaw_radius = geometry_.FrontOmniYawRadius();
    const float rear_yaw_radius = geometry_.RearMecanumYawRadius();

    body_velocity.vx_mps = 0.25f * (-v0 + v1 + v2 - v3);
    body_velocity.wz_rad_s = -0.5f * (v0 + v1) / front_yaw_radius;
    body_velocity.vy_mps = 0.5f * (v2 + v3) +
                           rear_yaw_radius * body_velocity.wz_rad_s;
    return true;
  }

  static float MaxAbsWheelSpeed(const WheelSpeeds& wheel_speeds) {
    return mr::component::math::max_abs_array(wheel_speeds.data(),
                                              kWheelCount);
  }

  static bool ScaleWheelSpeedsToLimit(WheelSpeeds& wheel_speeds,
                                      float max_abs_speed) {
    return mr::component::math::scale_to_abs_limit(
        wheel_speeds.data(), kWheelCount, max_abs_speed);
  }

 private:
  static constexpr unsigned WheelIndex(FrontOmniRearMecanumWheel wheel) {
    return static_cast<unsigned>(wheel);
  }

  static bool IsFinite(const PlanarVelocity& velocity) {
    return mr::component::math::is_finite_scalar(velocity.vx_mps) &&
           mr::component::math::is_finite_scalar(velocity.vy_mps) &&
           mr::component::math::is_finite_scalar(velocity.wz_rad_s);
  }

  static bool IsFinite(const WheelSpeeds& wheel_speeds) {
    return mr::component::math::values_are_finite(wheel_speeds.data(),
                                                  kWheelCount);
  }

  FrontOmniRearMecanumGeometry geometry_{};
};

}  // namespace mr::robotics::chassis
