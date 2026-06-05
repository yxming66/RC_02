#pragma once

#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>

namespace cmd {

using TypeId = const void*;

template <typename T>
struct TypeKey {
  static TypeId id() {
    static const char tag = 0;
    return &tag;
  }
};

enum class Priority : uint8_t {
  Disabled = 0,
  Fallback = 10,
  Manual = 40,
  Remote = 60,
  Auto = 80,
  CriticalAuto = 100,
};

enum class TieBreak : uint8_t {
  KeepLastWinner,
  RouteOrder,
  Freshest,
  RejectConflict,
};

struct HighestPriority {
  TieBreak tie_break = TieBreak::KeepLastWinner;
};

class Context;

namespace detail {

template <typename T>
struct AlwaysFalse : std::false_type {};

struct DefaultCondition {
  bool operator()(const Context&) const { return true; }
};

struct FixedPriority {
  Priority value = Priority::Manual;
  Priority operator()(const Context&) const { return value; }
};

template <typename T>
struct IsPriority : std::is_same<std::decay_t<T>, Priority> {};

template <typename Callable, typename... Args>
bool InvokeBoolOrVoid(Callable& fn, Args&&... args) {
  using Result = decltype(fn(std::forward<Args>(args)...));
  if constexpr (std::is_same_v<Result, bool>) {
    return fn(std::forward<Args>(args)...);
  } else {
    fn(std::forward<Args>(args)...);
    return true;
  }
}

template <typename Cond>
bool EvalCondition(Cond& cond, const Context& ctx) {
  if constexpr (std::is_invocable_r_v<bool, Cond&, const Context&>) {
    return cond(ctx);
  } else if constexpr (std::is_invocable_r_v<bool, Cond&>) {
    return cond();
  } else {
    static_assert(AlwaysFalse<Cond>::value,
                  "condition must return bool and accept Context or no args");
  }
}

template <typename PriorityRule>
Priority EvalPriority(PriorityRule& rule, const Context& ctx) {
  if constexpr (std::is_same_v<std::decay_t<PriorityRule>, Priority>) {
    return rule;
  } else if constexpr (std::is_invocable_r_v<Priority, PriorityRule&,
                                             const Context&>) {
    return rule(ctx);
  } else if constexpr (std::is_invocable_r_v<Priority, PriorityRule&>) {
    return rule();
  } else {
    static_assert(AlwaysFalse<PriorityRule>::value,
                  "priority rule must return cmd::Priority");
  }
}

template <typename T, typename... Ts>
struct LastType {
  using type = typename LastType<Ts...>::type;
};

template <typename T>
struct LastType<T> {
  using type = T;
};

template <typename... Ts>
using LastTypeT = typename LastType<Ts...>::type;

template <typename... Ts>
struct TypePack {};

template <typename T, typename Pack>
struct PrependType;

template <typename T, typename... Ts>
struct PrependType<T, TypePack<Ts...>> {
  using type = TypePack<T, Ts...>;
};

template <typename... Ts>
struct PopBack;

template <typename T>
struct PopBack<T> {
  using type = TypePack<>;
};

template <typename T, typename U, typename... Ts>
struct PopBack<T, U, Ts...> {
  using type = typename PrependType<
      T, typename PopBack<U, Ts...>::type>::type;
};

template <typename Pack>
struct RouteInputPack;

template <typename... Inputs>
struct RouteInputPack<TypePack<Inputs...>> {
  template <typename Route, typename Cmd>
  static bool Build(Route& route, Context& ctx, Cmd& out);
};

template <typename Builder, typename Condition, typename PriorityRule,
          bool Locked, typename... Inputs>
struct RouteSpec {
  Builder builder{};
  Condition condition{};
  PriorityRule priority_rule{};

  template <typename NewCondition>
  auto when(NewCondition condition_value = NewCondition{}) const {
    RouteSpec<Builder, NewCondition, PriorityRule, Locked, Inputs...> next{};
    next.builder = builder;
    next.condition = condition_value;
    next.priority_rule = priority_rule;
    return next;
  }

  auto priority(Priority value) const {
    RouteSpec<Builder, Condition, FixedPriority, Locked, Inputs...> next{};
    next.builder = builder;
    next.condition = condition;
    next.priority_rule = FixedPriority{value};
    return next;
  }

  template <typename NewPriorityRule>
  auto priority(NewPriorityRule rule) const {
    RouteSpec<Builder, Condition, NewPriorityRule, Locked, Inputs...> next{};
    next.builder = builder;
    next.condition = condition;
    next.priority_rule = rule;
    return next;
  }

  auto lock() const {
    RouteSpec<Builder, Condition, PriorityRule, true, Inputs...> next{};
    next.builder = builder;
    next.condition = condition;
    next.priority_rule = priority_rule;
    return next;
  }
};

template <typename T, typename = void>
struct HasUpdate : std::false_type {};

template <typename T>
struct HasUpdate<T, std::void_t<decltype(std::declval<T&>().update(
                        std::declval<Context&>()))>> : std::true_type {};

template <typename T, typename = void>
struct HasOnline : std::false_type {};

template <typename T>
struct HasOnline<T, std::void_t<decltype(std::declval<const T&>().online())>>
    : std::true_type {};

template <typename T, typename Cmd, typename = void>
struct HasPublish : std::false_type {};

template <typename T, typename Cmd>
struct HasPublish<T, Cmd, std::void_t<decltype(std::declval<T&>()(
                             std::declval<const Cmd&>()))>> : std::true_type {};

}  // namespace detail

template <typename... InputAndBuilder>
auto from() {
  static_assert(sizeof...(InputAndBuilder) >= 2,
                "from<...>() expects at least one input type and one builder");
  using Builder = detail::LastTypeT<InputAndBuilder...>;
  using Inputs = typename detail::PopBack<InputAndBuilder...>::type;
  return []<typename... InputTypes>(detail::TypePack<InputTypes...>) {
    return detail::RouteSpec<Builder, detail::DefaultCondition,
                             detail::FixedPriority, false, InputTypes...>{};
  }(Inputs{});
}

template <typename Cmd>
struct Candidate {
  Cmd command{};
  const char* source = nullptr;
  Priority priority = Priority::Disabled;
  bool valid = false;
  bool locked = false;
  uint32_t sequence = 0;
};

class Context {
 public:
  Context() = default;

  explicit Context(uint32_t now_ms) : now_ms_(now_ms) {}

  uint32_t now_ms() const { return now_ms_; }
  float dt_s() const { return dt_s_; }

  void set_time(uint32_t now_ms, float dt_s) {
    now_ms_ = now_ms;
    dt_s_ = dt_s;
  }

  template <typename Input>
  Input* input() const {
    return static_cast<Input*>(find_input(TypeKey<Input>::id()));
  }

 private:
  template <size_t, size_t, size_t, size_t, size_t, size_t, size_t>
  friend class Center;

  void* (*find_input_)(void*, TypeId) = nullptr;
  void* find_input_ctx_ = nullptr;
  uint32_t now_ms_ = 0;
  float dt_s_ = 0.0f;

  void* find_input(TypeId type) const {
    return find_input_ ? find_input_(find_input_ctx_, type) : nullptr;
  }
};

namespace detail {

struct IInputSlot {
  const char* name = nullptr;
  TypeId type = nullptr;

  virtual bool update(Context& ctx) = 0;
  virtual bool online() const = 0;
  virtual void* raw() = 0;

 protected:
  ~IInputSlot() = default;
};

template <typename Input>
class InputSlot final : public IInputSlot {
 public:
  template <typename... Args>
  InputSlot(const char* input_name, Args&&... args)
      : input_(std::forward<Args>(args)...) {
    name = input_name;
    type = TypeKey<Input>::id();
  }

  bool update(Context& ctx) override {
    if constexpr (HasUpdate<Input>::value) {
      return input_.update(ctx);
    } else {
      (void)ctx;
      return true;
    }
  }

  bool online() const override {
    if constexpr (HasOnline<Input>::value) {
      return input_.online();
    } else {
      return true;
    }
  }

  void* raw() override { return &input_; }

 private:
  Input input_;
};

template <typename Cmd>
struct IRoute {
  virtual bool run(Context& ctx, Candidate<Cmd>& candidate) = 0;

 protected:
  ~IRoute() = default;
};

template <typename Cmd, typename Spec>
class RouteModel;

template <typename Cmd, typename Builder, typename Condition,
          typename PriorityRule, bool Locked, typename... Inputs>
class RouteModel<Cmd,
                 RouteSpec<Builder, Condition, PriorityRule, Locked,
                           Inputs...>> final : public IRoute<Cmd> {
 public:
  using Spec = RouteSpec<Builder, Condition, PriorityRule, Locked, Inputs...>;

  explicit RouteModel(const Spec& spec) : spec_(spec) {}

  bool run(Context& ctx, Candidate<Cmd>& candidate) override {
    if (!EvalCondition(spec_.condition, ctx)) {
      return false;
    }

    const Priority priority = EvalPriority(spec_.priority_rule, ctx);
    if (priority == Priority::Disabled) {
      return false;
    }

    Cmd command{};
    if (!InvokeBuilder(ctx, command, static_cast<Inputs*>(ctx.input<Inputs>())...)) {
      return false;
    }

    candidate.command = command;
    candidate.source = nullptr;
    candidate.priority = priority;
    candidate.valid = true;
    candidate.locked = Locked;
    return true;
  }

 private:
  bool InvokeBuilder(Context& ctx, Cmd& out, Inputs*... inputs) {
    if (((inputs == nullptr) || ...)) {
      return false;
    }

    if constexpr (std::is_invocable_v<Builder&, const Inputs&..., Context&,
                                      Cmd&>) {
      return InvokeBoolOrVoid(spec_.builder, *inputs..., ctx, out);
    } else if constexpr (std::is_invocable_v<Builder&, const Inputs&..., Cmd&>) {
      return InvokeBoolOrVoid(spec_.builder, *inputs..., out);
    } else if constexpr (std::is_invocable_v<Builder&, Context&, Cmd&>) {
      return InvokeBoolOrVoid(spec_.builder, ctx, out);
    } else if constexpr (std::is_invocable_v<Builder&, Cmd&>) {
      return InvokeBoolOrVoid(spec_.builder, out);
    } else {
      static_assert(AlwaysFalse<Builder>::value,
                    "builder must accept inputs plus (Context&, Cmd&) or Cmd&");
    }
  }

  Spec spec_;
};

template <typename Cmd>
struct ISafe {
  virtual bool build(Context& ctx, Cmd& out) = 0;

 protected:
  ~ISafe() = default;
};

template <typename Cmd, typename Builder>
class SafeModel final : public ISafe<Cmd> {
 public:
  explicit SafeModel(const Builder& builder) : builder_(builder) {}

  bool build(Context& ctx, Cmd& out) override {
    if constexpr (std::is_invocable_v<Builder&, Context&, Cmd&>) {
      return InvokeBoolOrVoid(builder_, ctx, out);
    } else if constexpr (std::is_invocable_v<Builder&, Cmd&>) {
      return InvokeBoolOrVoid(builder_, out);
    } else {
      static_assert(AlwaysFalse<Builder>::value,
                    "safe/hold builder must accept (Context&, Cmd&) or Cmd&");
    }
  }

 private:
  Builder builder_;
};

struct IOutputSlot {
  const char* name = nullptr;

  virtual void update(Context& ctx) = 0;
  virtual void force_safe(Context& ctx) = 0;
  virtual void publish(Context& ctx) = 0;

 protected:
  ~IOutputSlot() = default;
};

struct ISafetyRule {
  virtual bool active(Context& ctx) = 0;

 protected:
  ~ISafetyRule() = default;
};

template <typename Rule>
class SafetyRuleModel final : public ISafetyRule {
 public:
  explicit SafetyRuleModel(const Rule& rule) : rule_(rule) {}

  bool active(Context& ctx) override {
    if constexpr (std::is_invocable_r_v<bool, Rule&, Context&>) {
      return rule_(ctx);
    } else if constexpr (std::is_invocable_r_v<bool, Rule&, const Context&>) {
      return rule_(ctx);
    } else if constexpr (std::is_invocable_r_v<bool, Rule&>) {
      return rule_();
    } else {
      static_assert(AlwaysFalse<Rule>::value,
                    "safety rule must return bool");
    }
  }

 private:
  Rule rule_;
};

template <typename Cmd, typename Sink, size_t MaxRoutesPerOutput>
class OutputSlot final : public IOutputSlot {
 public:
  OutputSlot(const char* output_name, Sink sink) : sink_(sink) {
    name = output_name;
  }

  bool add_route(IRoute<Cmd>* route) {
    if (route_count_ >= MaxRoutesPerOutput) {
      return false;
    }
    routes_[route_count_++] = route;
    return true;
  }

  void set_safe(ISafe<Cmd>* safe) { safe_ = safe; }
  void set_hold(ISafe<Cmd>* hold) { hold_ = hold; }
  void set_arbitration(HighestPriority arbitration) {
    arbitration_ = arbitration;
  }

  const Cmd& command() const { return command_; }

  void update(Context& ctx) override {
    Candidate<Cmd> winner{};
    bool conflict = false;

    for (size_t i = 0; i < route_count_; ++i) {
      Candidate<Cmd> candidate{};
      candidate.sequence = static_cast<uint32_t>(i);
      if (!routes_[i]->run(ctx, candidate)) {
        continue;
      }

      if (candidate.locked && (!winner.valid || !winner.locked)) {
        winner = candidate;
        conflict = false;
        continue;
      }
      if (winner.locked && !candidate.locked) {
        continue;
      }

      const int cmp = Compare(candidate, winner);
      if (cmp > 0) {
        winner = candidate;
        conflict = false;
      } else if (cmp == 0) {
        conflict = true;
      }
    }

    if (winner.valid && !(conflict &&
                          arbitration_.tie_break == TieBreak::RejectConflict)) {
      command_ = winner.command;
      last_command_ = command_;
      last_winner_sequence_ = winner.sequence;
      has_last_command_ = true;
      return;
    }

    if (hold_ != nullptr && hold_->build(ctx, command_)) {
      last_command_ = command_;
      has_last_command_ = true;
      return;
    }

    if (safe_ != nullptr && safe_->build(ctx, command_)) {
      last_command_ = command_;
      has_last_command_ = true;
      return;
    }

    command_ = Cmd{};
    last_command_ = command_;
    has_last_command_ = true;
  }

  void force_safe(Context& ctx) override {
    if (safe_ != nullptr && safe_->build(ctx, command_)) {
      last_command_ = command_;
      has_last_command_ = true;
      return;
    }
    command_ = Cmd{};
    last_command_ = command_;
    has_last_command_ = true;
  }

  void publish(Context& ctx) override {
    (void)ctx;
    if constexpr (HasPublish<Sink, Cmd>::value) {
      sink_(command_);
    }
  }

 private:
  int Compare(const Candidate<Cmd>& candidate,
              const Candidate<Cmd>& winner) const {
    if (!winner.valid) {
      return 1;
    }
    if (static_cast<uint8_t>(candidate.priority) >
        static_cast<uint8_t>(winner.priority)) {
      return 1;
    }
    if (static_cast<uint8_t>(candidate.priority) <
        static_cast<uint8_t>(winner.priority)) {
      return -1;
    }

    switch (arbitration_.tie_break) {
      case TieBreak::KeepLastWinner:
        if (has_last_command_) {
          if (candidate.sequence == last_winner_sequence_) {
            return 1;
          }
          if (winner.sequence == last_winner_sequence_) {
            return -1;
          }
        }
        return 0;
      case TieBreak::RouteOrder:
        return candidate.sequence < winner.sequence ? 1 : -1;
      case TieBreak::Freshest:
        return candidate.sequence > winner.sequence ? 1 : -1;
      case TieBreak::RejectConflict:
      default:
        return 0;
    }
  }

  Sink sink_;
  IRoute<Cmd>* routes_[MaxRoutesPerOutput]{};
  size_t route_count_ = 0;
  ISafe<Cmd>* safe_ = nullptr;
  ISafe<Cmd>* hold_ = nullptr;
  HighestPriority arbitration_{};
  Cmd command_{};
  Cmd last_command_{};
  uint32_t last_winner_sequence_ = 0;
  bool has_last_command_ = false;
};

}  // namespace detail

template <size_t MaxInputs = 8, size_t MaxOutputs = 8, size_t MaxRoutes = 32,
          size_t MaxRoutesPerOutput = 8, size_t InputBytes = 64,
          size_t OutputBytes = 256, size_t RouteBytes = 128>
class Center {
 public:
  Center() {
    context_.find_input_ = &Center::FindInputThunk;
    context_.find_input_ctx_ = this;
  }

  template <typename Input, typename... Args>
  Center& input(const char* name, Args&&... args) {
    using Slot = detail::InputSlot<Input>;
    static_assert(sizeof(Slot) <= InputBytes,
                  "input slot too large; increase Center InputBytes");
    static_assert(alignof(Slot) <= alignof(InputStorage),
                  "input slot alignment too large");
    if (input_count_ >= MaxInputs) {
      overflow_ = true;
      return *this;
    }
    void* storage = input_storage_[input_count_].bytes;
    inputs_[input_count_++] =
        new (storage) Slot(name, std::forward<Args>(args)...);
    return *this;
  }

  template <typename Cmd, typename Sink>
  class OutputRef {
   public:
    OutputRef(Center& center,
              detail::OutputSlot<Cmd, Sink, MaxRoutesPerOutput>& output)
        : center_(center), output_(output) {}

    template <typename SafeBuilder>
    OutputRef& safe(SafeBuilder builder = SafeBuilder{}) {
      output_.set_safe(center_.template make_safe<Cmd>(builder));
      return *this;
    }

    template <typename HoldBuilder>
    OutputRef& hold(HoldBuilder builder = HoldBuilder{}) {
      output_.set_hold(center_.template make_safe<Cmd>(builder));
      return *this;
    }

    template <typename Arbitration>
    OutputRef& arbitrate(Arbitration arbitration = Arbitration{}) {
      if constexpr (std::is_same_v<Arbitration, HighestPriority>) {
        output_.set_arbitration(arbitration);
      }
      return *this;
    }

    template <typename... RouteSpecs>
    OutputRef& routes(RouteSpecs... specs) {
      (add_route(specs), ...);
      return *this;
    }

    template <typename RouteSpec>
    OutputRef& route(RouteSpec spec) {
      add_route(spec);
      return *this;
    }

   private:
    template <typename RouteSpec>
    void add_route(RouteSpec spec) {
      auto* route = center_.template make_route<Cmd>(spec);
      if (route == nullptr || !output_.add_route(route)) {
        center_.overflow_ = true;
      }
    }

    Center& center_;
    detail::OutputSlot<Cmd, Sink, MaxRoutesPerOutput>& output_;
  };

  template <typename Cmd, typename Sink>
  OutputRef<Cmd, Sink> output(const char* name, Sink sink) {
    using Slot = detail::OutputSlot<Cmd, Sink, MaxRoutesPerOutput>;
    static_assert(sizeof(Slot) <= OutputBytes,
                  "output slot too large; increase Center OutputBytes");
    static_assert(alignof(Slot) <= alignof(OutputStorage),
                  "output slot alignment too large");
    if (output_count_ >= MaxOutputs) {
      overflow_ = true;
      return OutputRef<Cmd, Sink>(*this, *reinterpret_cast<Slot*>(
                                             output_storage_[0].bytes));
    }
    void* storage = output_storage_[output_count_].bytes;
    auto* slot = new (storage) Slot(name, sink);
    outputs_[output_count_++] = slot;
    return OutputRef<Cmd, Sink>(*this, *slot);
  }

  class GlobalRef {
   public:
    explicit GlobalRef(Center& center) : center_(center) {}

    template <typename SafetyRule>
    GlobalRef& safety(SafetyRule rule = SafetyRule{}) {
      center_.add_safety(rule);
      return *this;
    }

   private:
    Center& center_;
  };

  GlobalRef global() { return GlobalRef(*this); }

  Center& tick(uint32_t now_ms) {
    const float dt_s =
        last_now_ms_valid_ ? static_cast<float>(now_ms - last_now_ms_) * 0.001f
                           : 0.0f;
    last_now_ms_ = now_ms;
    last_now_ms_valid_ = true;
    context_.set_time(now_ms, dt_s);

    for (size_t i = 0; i < input_count_; ++i) {
      inputs_[i]->update(context_);
    }

    bool force_safe = false;
    for (size_t i = 0; i < safety_count_; ++i) {
      if (safeties_[i]->active(context_)) {
        force_safe = true;
        break;
      }
    }

    for (size_t i = 0; i < output_count_; ++i) {
      if (force_safe) {
        outputs_[i]->force_safe(context_);
      } else {
        outputs_[i]->update(context_);
      }
    }
    return *this;
  }

  Center& publish() {
    for (size_t i = 0; i < output_count_; ++i) {
      outputs_[i]->publish(context_);
    }
    return *this;
  }

  bool overflowed() const { return overflow_; }
  Context& context() { return context_; }
  const Context& context() const { return context_; }

 private:
  struct InputStorage {
    alignas(std::max_align_t) uint8_t bytes[InputBytes];
  };

  struct OutputStorage {
    alignas(std::max_align_t) uint8_t bytes[OutputBytes];
  };

  struct RouteStorage {
    alignas(std::max_align_t) uint8_t bytes[RouteBytes];
  };

  template <typename Cmd, typename RouteSpec>
  detail::IRoute<Cmd>* make_route(RouteSpec spec) {
    using Route = detail::RouteModel<Cmd, RouteSpec>;
    static_assert(sizeof(Route) <= RouteBytes,
                  "route slot too large; increase Center RouteBytes");
    static_assert(alignof(Route) <= alignof(RouteStorage),
                  "route slot alignment too large");
    if (route_count_ >= MaxRoutes) {
      overflow_ = true;
      return nullptr;
    }
    void* storage = route_storage_[route_count_++].bytes;
    return new (storage) Route(spec);
  }

  template <typename Cmd, typename Builder>
  detail::ISafe<Cmd>* make_safe(Builder builder) {
    using Model = detail::SafeModel<Cmd, Builder>;
    static_assert(sizeof(Model) <= RouteBytes,
                  "safe/hold slot too large; increase Center RouteBytes");
    static_assert(alignof(Model) <= alignof(RouteStorage),
                  "safe/hold slot alignment too large");
    if (route_count_ >= MaxRoutes) {
      overflow_ = true;
      return nullptr;
    }
    void* storage = route_storage_[route_count_++].bytes;
    return new (storage) Model(builder);
  }

  template <typename SafetyRule>
  void add_safety(SafetyRule rule) {
    using Model = detail::SafetyRuleModel<SafetyRule>;
    static_assert(sizeof(Model) <= RouteBytes,
                  "safety rule too large; increase Center RouteBytes");
    static_assert(alignof(Model) <= alignof(RouteStorage),
                  "safety rule alignment too large");
    if (safety_count_ >= MaxRoutes || route_count_ >= MaxRoutes) {
      overflow_ = true;
      return;
    }
    void* storage = route_storage_[route_count_++].bytes;
    safeties_[safety_count_++] = new (storage) Model(rule);
  }

  static void* FindInputThunk(void* self, TypeId type) {
    return static_cast<Center*>(self)->find_input(type);
  }

  void* find_input(TypeId type) {
    for (size_t i = 0; i < input_count_; ++i) {
      if (inputs_[i]->type == type) {
        return inputs_[i]->raw();
      }
    }
    return nullptr;
  }

  detail::IInputSlot* inputs_[MaxInputs]{};
  detail::IOutputSlot* outputs_[MaxOutputs]{};
  detail::ISafetyRule* safeties_[MaxRoutes]{};
  InputStorage input_storage_[MaxInputs]{};
  OutputStorage output_storage_[MaxOutputs]{};
  RouteStorage route_storage_[MaxRoutes]{};
  size_t input_count_ = 0;
  size_t output_count_ = 0;
  size_t route_count_ = 0;
  size_t safety_count_ = 0;
  Context context_{};
  uint32_t last_now_ms_ = 0;
  bool last_now_ms_valid_ = false;
  bool overflow_ = false;
};

template <typename Fn>
class FunctionSink {
 public:
  explicit FunctionSink(Fn fn) : fn_(fn) {}

  template <typename Cmd>
  bool operator()(const Cmd& cmd) {
    if constexpr (std::is_invocable_r_v<bool, Fn&, const Cmd&>) {
      return fn_(cmd);
    } else if constexpr (std::is_invocable_r_v<bool, Fn&, const Cmd*>) {
      return fn_(&cmd);
    } else if constexpr (std::is_invocable_v<Fn&, const Cmd&>) {
      fn_(cmd);
      return true;
    } else if constexpr (std::is_invocable_v<Fn&, const Cmd*>) {
      fn_(&cmd);
      return true;
    } else {
      static_assert(detail::AlwaysFalse<Fn>::value,
                    "function sink must accept const Cmd& or const Cmd*");
    }
  }

 private:
  Fn fn_;
};

template <typename Fn>
FunctionSink<Fn> function(Fn fn) {
  return FunctionSink<Fn>(fn);
}

}  // namespace cmd
