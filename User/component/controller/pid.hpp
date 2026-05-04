/*
 * 通用 PID 控制器
 *
 * 类型:
 *   mr::comp::cntlr::pid
 *
 * 控制律:
 *   error = setpoint - feedback
 *   output = kp * error + integral + kd * derivative + kff * setpoint
 *
 * 最常用的一句构建:
 *   auto ctrl = mr::comp::cntlr::pid::Build(
 *       2.0f,    // kp
 *       0.1f,    // ki
 *       0.01f,   // kd
 *       6.0f,    // output_limit, 0 表示不限幅
 *       500.0f   // sample_freq_hz, 不写则默认 1000 Hz
 *   );
 *
 *   float out = ctrl.Update(target, feedback, dt_s);
 *
 * 一句完整构建:
 *   auto ctrl = mr::comp::cntlr::pid::Build(
 *       {2.0f, 0.1f, 0.01f, 0.0f},              // kp, ki, kd, kff
 *       mr::comp::cntlr::SymmetricLimit::Abs(6.0f),   // output_limit
 *       mr::comp::cntlr::SymmetricLimit::Abs(1.0f),   // integral_limit
 *       mr::comp::cntlr::SymmetricLimit::Abs(20.0f),  // output_rate_limit
 *       mr::comp::cntlr::RangeWrap::Periodic(2.0f * pi),
 *       mr::comp::cntlr::DerivativeMode::kOnMeasurement,
 *       500.0f,  // sample_freq_hz
 *       30.0f,   // derivative_cutoff_hz
 *       true     // hold_integral_on_saturation
 *   );
 *
 * 配置对象构建:
 *   mr::comp::cntlr::pid::Config cfg{};
 *   cfg.gains = {2.0f, 0.1f, 0.01f, 0.0f};
 *   cfg.output_limit = mr::comp::cntlr::SymmetricLimit::Abs(6.0f);
 *   cfg.integral_limit = mr::comp::cntlr::SymmetricLimit::Abs(1.0f);
 *   cfg.output_rate_limit = mr::comp::cntlr::SymmetricLimit::Abs(20.0f);
 *   cfg.error_wrap = mr::comp::cntlr::RangeWrap::Periodic(2.0f * pi);
 *   cfg.derivative_mode = mr::comp::cntlr::DerivativeMode::kOnMeasurement;
 *   cfg.sample_freq_hz = 500.0f;
 *   cfg.derivative_cutoff_hz = 30.0f;
 *   cfg.hold_integral_on_saturation = true;
 *
 *   auto ctrl = mr::comp::cntlr::pid::Build(cfg);
 *
 * 参数说明:
 *   gains.kp
 *     比例增益。直接放大当前误差。
 *
 *   gains.ki
 *     积分增益。按 error * dt 累加，用于消除稳态误差。
 *
 *   gains.kd
 *     微分增益。根据反馈变化率或外部反馈速度抑制快速变化。
 *
 *   gains.kff
 *     前馈增益。当前实现为 kff * setpoint。
 *
 *   output_limit
 *     输出对称限幅。值为 0 表示不限幅。
 *
 *   integral_limit
 *     积分项对称限幅。值为 0 表示不限幅。
 *
 *   output_rate_limit
 *     输出变化率限幅，单位是 output/s。值为 0 表示不限速率。
 *
 *   error_wrap
 *     周期误差计算。角度控制可设置为 RangeWrap::Periodic(2*pi)，
 *     这样 179 deg 到 -179 deg 会按最短 2 deg 误差计算。
 *
 *   derivative_mode
 *     kNone          : 不使用微分项。
 *     kOnMeasurement : 对反馈微分，derivative = -(feedback-last_feedback)/dt。
 *                      这是默认值，能避免目标突变时产生微分冲击。
 *     kExternal      : 使用外部反馈速度，derivative = -feedback_dot。
 *
 *   sample_freq_hz
 *     默认采样频率。当传入 dt_s 无效时，用 1/sample_freq_hz 作为兜底 dt。
 *     不配置时默认 kDefaultSampleFreqHz，也就是 1000 Hz。
 *
 *   derivative_cutoff_hz
 *     微分项一阶低通截止频率。值 <= 0 表示不滤波。
 *
 *   hold_integral_on_saturation
 *     输出饱和且误差仍会推动输出继续饱和时，暂停积分，降低积分饱和风险。
 *
 * 调试输出:
 *   Step() 返回 mr::comp::cntlr::Step，包含 output、error、p、i、d、
 *   feedforward、valid、saturated。只需要输出值时可直接调用 Update()。
 */

#pragma once

#include "component/controller/controller_types.hpp"

namespace mr::comp::cntlr {

struct pid_gains {
  Scalar kp = 0.0f;
  Scalar ki = 0.0f;
  Scalar kd = 0.0f;
  Scalar kff = 0.0f;
};

struct pid_config {
  pid_gains gains{};
  SymmetricLimit output_limit{};
  SymmetricLimit integral_limit{};
  SymmetricLimit output_rate_limit{};
  RangeWrap error_wrap{};
  DerivativeMode derivative_mode = DerivativeMode::kOnMeasurement;
  Scalar sample_freq_hz = kDefaultSampleFreqHz;
  Scalar derivative_cutoff_hz = 0.0f;
  bool hold_integral_on_saturation = true;
};

class pid {
 public:
  using Gains = pid_gains;
  using Config = pid_config;

  static pid Build(const Config& config) { return pid(config); }

  static pid Build(Scalar kp,
                   Scalar ki,
                   Scalar kd,
                   Scalar output_limit = kUnlimited,
                   Scalar sample_freq_hz = kDefaultSampleFreqHz) {
    return Build({kp, ki, kd, 0.0f}, SymmetricLimit::Abs(output_limit),
                 SymmetricLimit::Unlimited(), SymmetricLimit::Unlimited(),
                 RangeWrap::Disabled(), DerivativeMode::kOnMeasurement,
                 sample_freq_hz);
  }

  static pid Build(const Gains& gains,
                   SymmetricLimit output_limit = SymmetricLimit::Unlimited(),
                   SymmetricLimit integral_limit = SymmetricLimit::Unlimited(),
                   SymmetricLimit output_rate_limit =
                       SymmetricLimit::Unlimited(),
                   RangeWrap error_wrap = RangeWrap::Disabled(),
                   DerivativeMode derivative_mode =
                       DerivativeMode::kOnMeasurement,
                   Scalar sample_freq_hz = kDefaultSampleFreqHz,
                   Scalar derivative_cutoff_hz = 0.0f,
                   bool hold_integral_on_saturation = true) {
    Config config{};
    config.gains = gains;
    config.output_limit = output_limit;
    config.integral_limit = integral_limit;
    config.output_rate_limit = output_rate_limit;
    config.error_wrap = error_wrap;
    config.derivative_mode = derivative_mode;
    config.sample_freq_hz = sample_freq_hz;
    config.derivative_cutoff_hz = derivative_cutoff_hz;
    config.hold_integral_on_saturation = hold_integral_on_saturation;
    return pid(config);
  }

  constexpr pid() = default;
  explicit pid(const Config& config) { Configure(config); }

  void Configure(const Config& config) {
    config_ = config;
    default_dt_ =
        (config_.sample_freq_hz > 0.0f) ? 1.0f / config_.sample_freq_hz
                                        : kDefaultDt;
    Reset();
  }

  void Reset(Scalar output = 0.0f) {
    integral_ = 0.0f;
    last_error_ = 0.0f;
    last_feedback_ = 0.0f;
    derivative_state_ = 0.0f;
    output_limiter_ =
        SlewRateLimiter::Build(config_.output_rate_limit.value, output);
    last_ = {};
    last_.output = output;
    initialized_ = false;
  }

  void ResetIntegral(Scalar integral = 0.0f) {
    integral_ = config_.integral_limit.Apply(integral);
  }

  Scalar Update(Scalar setpoint, Scalar feedback, Scalar dt_s) {
    return Step(setpoint, feedback, 0.0f, dt_s).output;
  }

  Scalar Update(Scalar setpoint,
                Scalar feedback,
                Scalar feedback_dot,
                Scalar dt_s) {
    return Step(setpoint, feedback, feedback_dot, dt_s).output;
  }

  ::mr::comp::cntlr::Step Step(Scalar setpoint,
                              Scalar feedback,
                              Scalar feedback_dot,
                              Scalar dt_s) {
    if (!IsUsableScalar(setpoint) || !IsUsableScalar(feedback) ||
        !IsUsableScalar(feedback_dot)) {
      last_.valid = false;
      return last_;
    }

    const Scalar dt = SanitizeDt(dt_s, default_dt_);
    const Scalar error = config_.error_wrap.Error(setpoint, feedback);
    if (!initialized_) {
      last_error_ = error;
      last_feedback_ = feedback;
      initialized_ = true;
    }

    const Scalar p_term = config_.gains.kp * error;
    const Scalar d_term = CalcDerivativeTerm(feedback, feedback_dot, dt);
    const Scalar ff_term = config_.gains.kff * setpoint;

    const Scalar candidate_integral =
        config_.integral_limit.Apply(integral_ + config_.gains.ki * error * dt);
    const Scalar unsat_with_candidate =
        p_term + candidate_integral + d_term + ff_term;
    const Scalar saturated_candidate =
        config_.output_limit.Apply(unsat_with_candidate);

    const bool saturated =
        config_.output_limit.enabled() &&
        mr::component::math::abs_scalar(saturated_candidate -
                                            unsat_with_candidate) > kEpsilon;
    if (!config_.hold_integral_on_saturation || !saturated ||
        DrivesTowardUnsaturation(error, saturated_candidate)) {
      integral_ = candidate_integral;
    }

    const Scalar unsat_output = p_term + integral_ + d_term + ff_term;
    Scalar output = config_.output_limit.Apply(unsat_output);
    output = output_limiter_.Update(output, dt);

    last_error_ = error;
    last_feedback_ = feedback;
    last_ = {};
    last_.output = output;
    last_.setpoint = setpoint;
    last_.feedback = feedback;
    last_.error = error;
    last_.feedforward = ff_term;
    last_.p = p_term;
    last_.i = integral_;
    last_.d = d_term;
    last_.valid = true;
    last_.saturated =
        saturated ||
        (mr::component::math::abs_scalar(output - unsat_output) > kEpsilon);
    return last_;
  }

  const ::mr::comp::cntlr::Step& last() const { return last_; }
  const Config& config() const { return config_; }
  Scalar integral() const { return integral_; }

 private:
  Scalar CalcDerivativeTerm(Scalar feedback, Scalar feedback_dot, Scalar dt) {
    Scalar derivative = 0.0f;
    switch (config_.derivative_mode) {
      case DerivativeMode::kNone:
        derivative = 0.0f;
        break;
      case DerivativeMode::kExternal:
        derivative = -feedback_dot;
        break;
      case DerivativeMode::kOnMeasurement:
      default:
        derivative = -(feedback - last_feedback_) / dt;
        break;
    }

    if (!IsUsableScalar(derivative)) {
      derivative = 0.0f;
    }
    derivative = FilterDerivative(derivative, dt);
    return config_.gains.kd * derivative;
  }

  Scalar FilterDerivative(Scalar derivative, Scalar dt) {
    if (config_.derivative_cutoff_hz <= 0.0f ||
        !IsUsableScalar(config_.derivative_cutoff_hz)) {
      derivative_state_ = derivative;
      return derivative;
    }

    const Scalar rc = 1.0f / (6.28318530718f * config_.derivative_cutoff_hz);
    const Scalar alpha = dt / (rc + dt);
    derivative_state_ += alpha * (derivative - derivative_state_);
    return derivative_state_;
  }

  bool DrivesTowardUnsaturation(Scalar error, Scalar saturated_output) const {
    if (!config_.output_limit.enabled()) {
      return true;
    }
    if (saturated_output >= config_.output_limit.value && error < 0.0f) {
      return true;
    }
    if (saturated_output <= -config_.output_limit.value && error > 0.0f) {
      return true;
    }
    return false;
  }

  Config config_{};
  Scalar default_dt_ = kDefaultDt;
  Scalar integral_ = 0.0f;
  Scalar last_error_ = 0.0f;
  Scalar last_feedback_ = 0.0f;
  Scalar derivative_state_ = 0.0f;
  SlewRateLimiter output_limiter_{};
  ::mr::comp::cntlr::Step last_{};
  bool initialized_ = false;
};

}  // namespace mr::comp::cntlr
