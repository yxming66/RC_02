#pragma once

#include "component/controller/pid.hpp"

namespace mr::comp::cntlr {

struct cascade_pid_config {
  pid::Config outer{};
  pid::Config inner{};
  SymmetricLimit intermediate_limit{};
};

struct cascade_pid_step {
  ::mr::comp::cntlr::Step outer{};
  ::mr::comp::cntlr::Step inner{};
  Scalar intermediate = 0.0f;
  Scalar output = 0.0f;
  bool valid = false;
};

class cascade_pid {
 public:
  using Config = cascade_pid_config;
  using Step = cascade_pid_step;

  static cascade_pid Build(const Config& config) { return cascade_pid(config); }

  static cascade_pid Build(const pid::Config& outer,
                           const pid::Config& inner,
                           Scalar intermediate_limit = kUnlimited) {
    Config config{};
    config.outer = outer;
    config.inner = inner;
    config.intermediate_limit = SymmetricLimit::Abs(intermediate_limit);
    return cascade_pid(config);
  }

  constexpr cascade_pid() = default;
  explicit cascade_pid(const Config& config)
      : config_(config),
        outer_(pid::Build(config.outer)),
        inner_(pid::Build(config.inner)) {}

  void Configure(const Config& config) {
    config_ = config;
    outer_.Configure(config.outer);
    inner_.Configure(config.inner);
  }

  void Reset() {
    outer_.Reset();
    inner_.Reset();
    last_ = {};
  }

  Scalar Update(Scalar outer_setpoint,
                Scalar outer_feedback,
                Scalar inner_feedback,
                Scalar dt_s) {
    return StepOnce(outer_setpoint, outer_feedback, inner_feedback, dt_s).output;
  }

  Scalar UpdateWithIntermediate(Scalar intermediate_setpoint,
                                Scalar inner_feedback,
                                Scalar dt_s) {
    return StepWithIntermediate(intermediate_setpoint, inner_feedback, dt_s)
        .output;
  }

  Step StepOnce(Scalar outer_setpoint,
                Scalar outer_feedback,
                Scalar inner_feedback,
                Scalar dt_s) {
    last_ = {};
    last_.outer = outer_.Step(outer_setpoint, outer_feedback, 0.0f, dt_s);
    last_.intermediate = config_.intermediate_limit.Apply(last_.outer.output);
    last_.inner = inner_.Step(last_.intermediate, inner_feedback, 0.0f, dt_s);
    last_.output = last_.inner.output;
    last_.valid = last_.outer.valid && last_.inner.valid;
    return last_;
  }

  Step StepWithIntermediate(Scalar intermediate_setpoint,
                            Scalar inner_feedback,
                            Scalar dt_s) {
    last_ = {};
    last_.intermediate = config_.intermediate_limit.Apply(intermediate_setpoint);
    last_.inner = inner_.Step(last_.intermediate, inner_feedback, 0.0f, dt_s);
    last_.output = last_.inner.output;
    last_.valid = last_.inner.valid;
    return last_;
  }

  pid& outer() { return outer_; }
  pid& inner() { return inner_; }
  const Step& last() const { return last_; }

 private:
  Config config_{};
  pid outer_{};
  pid inner_{};
  Step last_{};
};

}  // namespace mr::comp::cntlr
