#pragma once

/*
 * Scalar Kalman filter.
 *
 * Use it for one-dimensional measurements when process and measurement noise
 * can be estimated:
 *
 *   auto filter = mr::comp::filter::kalman::Build(
 *       0.01f,  // process_noise
 *       0.10f,  // measurement_noise
 *       1.0f    // estimate_error
 *   );
 *
 *   float y = filter.Update(measurement);
 *
 * Optional prediction can be applied before a measurement update:
 *
 *   filter.Predict(control_input, control_gain);
 *   float y = filter.Update(measurement);
 *
 * The first valid measurement initializes the estimate by default. Call
 * Reset(value) to set the estimate explicitly.
 */

#include "component/filter/filter_types.hpp"

namespace mr::comp::filter {

struct kalman_config {
  Scalar process_noise = 1.0e-3f;
  Scalar measurement_noise = 1.0e-2f;
  Scalar estimate_error = 1.0f;
  Scalar initial_estimate = 0.0f;
  bool reset_on_first_measurement = true;
};

class kalman {
 public:
  using Config = kalman_config;

  static kalman Build(const Config& config) { return kalman(config); }

  static kalman Build(Scalar process_noise,
                      Scalar measurement_noise,
                      Scalar estimate_error = 1.0f) {
    Config config{};
    config.process_noise = process_noise;
    config.measurement_noise = measurement_noise;
    config.estimate_error = estimate_error;
    return kalman(config);
  }

  kalman() { Configure(Config{}); }
  explicit kalman(const Config& config) { Configure(config); }

  void Configure(const Config& config) {
    config_ = SanitizeConfig(config);
    estimate_ = is_usable_scalar(config_.initial_estimate)
                    ? config_.initial_estimate
                    : 0.0f;
    covariance_ = config_.estimate_error;
    gain_ = 0.0f;
    initialized_ = !config_.reset_on_first_measurement;
  }

  void Reset(Scalar estimate = 0.0f) {
    estimate_ = is_usable_scalar(estimate) ? estimate : 0.0f;
    covariance_ = config_.estimate_error;
    gain_ = 0.0f;
    initialized_ = true;
  }

  void ResetOnNextMeasurement() { initialized_ = false; }

  Scalar Predict(Scalar control_input = 0.0f,
                 Scalar control_gain = 0.0f) {
    if (!initialized_) {
      return estimate_;
    }
    if (is_usable_scalar(control_input) && is_usable_scalar(control_gain)) {
      estimate_ += control_gain * control_input;
    }
    covariance_ += config_.process_noise;
    return estimate_;
  }

  Scalar Update(Scalar measurement) {
    if (!is_usable_scalar(measurement)) {
      return estimate_;
    }
    if (!initialized_) {
      Reset(measurement);
      return estimate_;
    }

    covariance_ += config_.process_noise;
    const Scalar innovation_covariance =
        covariance_ + config_.measurement_noise;
    if (!mr::component::math::is_positive_scalar(innovation_covariance)) {
      return estimate_;
    }

    gain_ = covariance_ / innovation_covariance;
    estimate_ += gain_ * (measurement - estimate_);
    covariance_ = (1.0f - gain_) * covariance_;

    if (!is_usable_scalar(estimate_) || !is_usable_scalar(covariance_)) {
      Configure(config_);
    }
    return estimate_;
  }

  Scalar Apply(Scalar measurement) { return Update(measurement); }

  const Config& config() const { return config_; }
  Scalar estimate() const { return estimate_; }
  Scalar covariance() const { return covariance_; }
  Scalar gain() const { return gain_; }
  bool initialized() const { return initialized_; }

 private:
  static Config SanitizeConfig(Config config) {
    config.process_noise = mr::component::math::sanitize_positive(
        config.process_noise, kalman_config{}.process_noise);
    config.measurement_noise = mr::component::math::sanitize_positive(
        config.measurement_noise, kalman_config{}.measurement_noise);
    config.estimate_error = mr::component::math::sanitize_positive(
        config.estimate_error, kalman_config{}.estimate_error);
    return config;
  }

  Config config_{};
  Scalar estimate_ = 0.0f;
  Scalar covariance_ = 1.0f;
  Scalar gain_ = 0.0f;
  bool initialized_ = false;
};

}  // namespace mr::comp::filter
