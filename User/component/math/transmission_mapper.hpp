#pragma once

#include "scalar.hpp"

namespace mr::component::math {

struct TransmissionRatioMapper {
  Scalar ratio = 1.0f;

  static TransmissionRatioMapper FromRatio(Scalar ratio_value) {
    return {positive_or(ratio_value, 1.0f)};
  }

  Scalar ToInput(Scalar output_value) const {
    return output_value * ratio;
  }

  Scalar ToInputLimit(Scalar output_limit) const {
    return abs_scalar(output_limit) * ratio;
  }

  Scalar ToOutput(Scalar input_value) const {
    return input_value / ratio;
  }
};

}  // namespace mr::component::math
