#pragma once

#include "scalar.hpp"

namespace mr::component::math {

class CyclicPositionTracker {
public:
  void Reset() {
    initialized_ = false;
    last_position_ = 0.0f;
    accumulated_position_ = 0.0f;
  }

  void Sync(Scalar position) {
    initialized_ = true;
    last_position_ = position;
    accumulated_position_ = position;
  }

  Scalar Accumulate(Scalar position,
                    Scalar velocity,
                    Scalar range,
                    Scalar default_resync_delta) {
    if (!initialized_) {
      Sync(position);
      return accumulated_position_;
    }

    const Scalar delta = wrap_error(position - last_position_, range);
    const Scalar delta_limit = (abs_scalar(velocity) > 0.0f)
        ? abs_scalar(velocity) + kPi
        : default_resync_delta;

    if (abs_scalar(delta) > delta_limit) {
      Sync(position);
      return accumulated_position_;
    }

    accumulated_position_ += delta;
    last_position_ = position;
    return accumulated_position_;
  }

  bool initialized() const { return initialized_; }
  Scalar last_position() const { return last_position_; }
  Scalar accumulated_position() const { return accumulated_position_; }

private:
  bool initialized_ = false;
  Scalar last_position_ = 0.0f;
  Scalar accumulated_position_ = 0.0f;
};

using ContinuousAngleTracker = CyclicPositionTracker;

}  // namespace mr::component::math
