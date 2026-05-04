#pragma once

#include <array>
#include <cmath>

namespace mr::robotics::chassis {

struct PlanarVelocity {
  float vx_mps = 0.0f;
  float vy_mps = 0.0f;
  float wz_rad_s = 0.0f;
};

struct MecanumGeometry {
  // The existing chassis config stores half of the front-rear and left-right
  // wheel spacing. Their sum is the effective rotation radius used here.
  float wheelbase_m = 0.0f;
  float trackwidth_m = 0.0f;

  float RotationRadius() const { return wheelbase_m + trackwidth_m; }

  bool IsValid() const {
    return std::isfinite(wheelbase_m) && std::isfinite(trackwidth_m) &&
           wheelbase_m >= 0.0f && trackwidth_m >= 0.0f &&
           RotationRadius() > 1e-5f;
  }
};

enum class MecanumWheel : unsigned {
  kFrontRight = 0,
  kFrontLeft = 1,
  kRearLeft = 2,
  kRearRight = 3,
};

class MecanumChassis {
 public:
  static constexpr unsigned kWheelCount = 4U;
  using WheelSpeeds = std::array<float, kWheelCount>;

  constexpr MecanumChassis() = default;
  explicit constexpr MecanumChassis(const MecanumGeometry& geometry)
      : geometry_(geometry) {}

  void SetGeometry(const MecanumGeometry& geometry) { geometry_ = geometry; }
  const MecanumGeometry& geometry() const { return geometry_; }
  bool IsValid() const { return geometry_.IsValid(); }

  bool InverseKinematics(const PlanarVelocity& body_velocity,
                         WheelSpeeds& wheel_speeds) const {
    if (!IsValid() || !IsFinite(body_velocity)) {
      wheel_speeds = {};
      return false;
    }

    const float wz_term = geometry_.RotationRadius() * body_velocity.wz_rad_s;
    wheel_speeds[MecanumIndex(MecanumWheel::kFrontRight)] =
        body_velocity.vx_mps - body_velocity.vy_mps - wz_term;
    wheel_speeds[MecanumIndex(MecanumWheel::kFrontLeft)] =
        body_velocity.vx_mps + body_velocity.vy_mps - wz_term;
    wheel_speeds[MecanumIndex(MecanumWheel::kRearLeft)] =
        -body_velocity.vx_mps + body_velocity.vy_mps - wz_term;
    wheel_speeds[MecanumIndex(MecanumWheel::kRearRight)] =
        -body_velocity.vx_mps - body_velocity.vy_mps - wz_term;
    return true;
  }

  bool ForwardKinematics(const WheelSpeeds& wheel_speeds,
                         PlanarVelocity& body_velocity) const {
    if (!IsValid() || !IsFinite(wheel_speeds)) {
      body_velocity = {};
      return false;
    }

    const float v0 = wheel_speeds[MecanumIndex(MecanumWheel::kFrontRight)];
    const float v1 = wheel_speeds[MecanumIndex(MecanumWheel::kFrontLeft)];
    const float v2 = wheel_speeds[MecanumIndex(MecanumWheel::kRearLeft)];
    const float v3 = wheel_speeds[MecanumIndex(MecanumWheel::kRearRight)];
    const float k = geometry_.RotationRadius();

    body_velocity.vx_mps = 0.25f * (v0 + v1 - v2 - v3);
    body_velocity.vy_mps = 0.25f * (-v0 + v1 + v2 - v3);
    body_velocity.wz_rad_s = -0.25f * (v0 + v1 + v2 + v3) / k;
    return true;
  }

  static float MaxAbsWheelSpeed(const WheelSpeeds& wheel_speeds) {
    float max_abs = 0.0f;
    for (float speed : wheel_speeds) {
      const float abs_speed = std::fabs(speed);
      if (abs_speed > max_abs) {
        max_abs = abs_speed;
      }
    }
    return max_abs;
  }

  static bool ScaleWheelSpeedsToLimit(WheelSpeeds& wheel_speeds,
                                      float max_abs_speed) {
    if (!std::isfinite(max_abs_speed) || max_abs_speed <= 0.0f) {
      return false;
    }
    const float max_abs = MaxAbsWheelSpeed(wheel_speeds);
    if (max_abs <= max_abs_speed || max_abs <= 0.0f) {
      return false;
    }
    const float scale = max_abs_speed / max_abs;
    for (float& speed : wheel_speeds) {
      speed *= scale;
    }
    return true;
  }

 private:
  static constexpr unsigned MecanumIndex(MecanumWheel wheel) {
    return static_cast<unsigned>(wheel);
  }

  static bool IsFinite(const PlanarVelocity& velocity) {
    return std::isfinite(velocity.vx_mps) && std::isfinite(velocity.vy_mps) &&
           std::isfinite(velocity.wz_rad_s);
  }

  static bool IsFinite(const WheelSpeeds& wheel_speeds) {
    for (float speed : wheel_speeds) {
      if (!std::isfinite(speed)) {
        return false;
      }
    }
    return true;
  }

  MecanumGeometry geometry_{};
};

}  // namespace mr::robotics::chassis
