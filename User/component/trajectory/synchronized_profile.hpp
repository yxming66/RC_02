#pragma once

#include "trapezoid.hpp"

namespace mr::component::trajectory {

template <int N>
class SynchronizedTrapezoidProfile {
 public:
  SynchronizedTrapezoidProfile() : duration_(0.0f), valid_(false) {}

  bool configure(const Scalar* distances,
                 const Scalar* max_velocities,
                 const Scalar* max_accelerations,
                 Scalar epsilon = kDefaultEpsilon) {
    valid_ = false;
    duration_ = 0.0f;
    if (distances == nullptr || max_velocities == nullptr ||
        max_accelerations == nullptr) {
      return false;
    }

    for (uint16_t i = 0; i < static_cast<uint16_t>(N); ++i) {
      const Scalar min_time = TrapezoidProfile::min_duration(
          distances[i], max_velocities[i], max_accelerations[i], epsilon);
      if (min_time < 0.0f) {
        return false;
      }
      if (min_time > duration_) {
        duration_ = min_time;
      }
    }

    for (uint16_t i = 0; i < static_cast<uint16_t>(N); ++i) {
      if (!profiles_[i].configure_synchronized(
              distances[i], max_velocities[i], max_accelerations[i],
              duration_, epsilon)) {
        valid_ = false;
        duration_ = 0.0f;
        return false;
      }
    }

    valid_ = true;
    return true;
  }

  ScalarMotionSample sample_axis(uint16_t index, Scalar elapsed) const {
    if (index >= static_cast<uint16_t>(N)) {
      return ScalarMotionSample();
    }
    return profiles_[index].sample(elapsed);
  }

  const TrapezoidProfile& axis(uint16_t index) const {
    return profiles_[index];
  }

  Scalar duration() const { return duration_; }
  bool valid() const { return valid_; }

 private:
  TrapezoidProfile profiles_[N];
  Scalar duration_;
  bool valid_;
};

}  // namespace mr::component::trajectory
