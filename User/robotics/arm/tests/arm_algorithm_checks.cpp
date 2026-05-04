#include <cmath>
#include <cstdio>

#include "robotics/arm/application/three_pit_cartesian_app.h"
#include "robotics/arm/controller/arm_controller.h"
#include "robotics/arm/kinematics/analytic_solvers/ik_3r_planar.h"
#include "robotics/arm/kinematics/analytic_solvers/ik_6r_pieper.h"
#include "robotics/arm/kinematics/ik_dispatch.h"
#include "robotics/arm/kinematics/ik_solution_scoring.h"
#include "robotics/arm/kinematics/pose_error.h"
#include "robotics/arm/model/chain_builder.h"
#include "robotics/arm/planning/move_j_planner.h"

namespace {

using ::mr::robotics::arm::ChainJointSpec;
using ::mr::robotics::arm::ChainJointType;
using ::mr::robotics::arm::DhParams;
using ::mr::robotics::arm::JointState;
using ::mr::robotics::arm::JointVec;
using ::mr::robotics::arm::Link;
using ::mr::robotics::arm::Scalar;
using ::mr::robotics::arm::SerialChain;
using ::mr::robotics::arm::ToolFrame;
using ::mr::robotics::arm::Transform;

constexpr float kPi = 3.14159265358979323846f;

int g_failures = 0;

void check(bool condition, const char* name) {
  if (condition) {
    std::printf("[PASS] %s\n", name);
  } else {
    std::printf("[FAIL] %s\n", name);
    ++g_failures;
  }
}

bool near(float lhs, float rhs, float tolerance) {
  return std::fabs(lhs - rhs) <= tolerance;
}

::mr::robotics::arm::Vec3 vec3(float x, float y, float z) {
  ::mr::robotics::arm::Vec3 out = ::mr::robotics::arm::toolbox_adapter::zero_vec3();
  out[0][0] = x;
  out[1][0] = y;
  out[2][0] = z;
  return out;
}

Transform translated(float x, float y, float z) {
  Transform out = ::mr::robotics::arm::toolbox_adapter::identity_transform();
  out[0][3] = x;
  out[1][3] = y;
  out[2][3] = z;
  return out;
}

Transform urdf_transform(float x,
                         float y,
                         float z,
                         float roll,
                         float pitch,
                         float yaw) {
  return ::mr::robotics::arm::toolbox_adapter::
      transform_from_rpy_translation(vec3(yaw, pitch, roll), vec3(x, y, z));
}

template <int N>
ChainJointSpec revolute_joint(float lower = -kPi, float upper = kPi) {
  ChainJointSpec joint;
  joint.type = ChainJointType::kRevolute;
  joint.limit_enabled = true;
  joint.lower = lower;
  joint.upper = upper;
  joint.participate_in_ik = true;
  return joint;
}

SerialChain<3> make_planar_3r_chain() {
  DhParams dh[3];
  dh[0].a = 0.30f;
  dh[1].a = 0.22f;
  dh[2].a = 0.12f;

  ChainJointSpec joints[3] = {
      revolute_joint<3>(), revolute_joint<3>(), revolute_joint<3>()};
  return ::mr::robotics::arm::load_chain_from_dh(dh, joints);
}

SerialChain<6> make_pieper_6r_chain() {
  ChainJointSpec joint = revolute_joint<6>(-2.8f, 2.8f);
  Link links[6];
  links[0] = Link(translated(0.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, 1.0f), joint);
  links[1] = Link(translated(0.0f, 0.0f, 0.25f), vec3(0.0f, 1.0f, 0.0f), joint);
  links[2] = Link(translated(0.30f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), joint);
  links[3] = Link(translated(0.25f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), joint);
  links[4] = Link(translated(0.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), joint);
  links[5] = Link(translated(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), joint);

  Transform tool = ::mr::robotics::arm::toolbox_adapter::identity_transform();
  tool[0][3] = 0.12f;
  return SerialChain<6>(links,
                        ::mr::robotics::arm::toolbox_adapter::identity_transform(),
                        ToolFrame(tool));
}

SerialChain<3> make_three_pit_chain() {
  Link links[3];
  const ::mr::robotics::arm::Vec3 negative_z_axis =
      vec3(0.0f, 0.0f, -1.0f);

  links[0] = Link(
      urdf_transform(0.0f, 0.0f, 0.07f, 1.5708f, 0.0f, -1.5708f),
      negative_z_axis, revolute_joint<3>(-1.57f, 1.57f));
  links[1] = Link(
      urdf_transform(-0.0005f, 0.35f, 0.0f, 0.0f, 0.0f, 0.0f),
      negative_z_axis, revolute_joint<3>(-2.32f, 2.32f));
  links[2] = Link(
      urdf_transform(-0.000357142857063231f, 0.24999974489785f,
                     -0.00262500000000032f, 0.0f, 0.0f, 0.0f),
      negative_z_axis, revolute_joint<3>(-1.95f, 1.95f));

  const Transform tool =
      urdf_transform(0.0034791f, 0.14696f, -0.000025f, 0.0f, 0.0f,
                     0.0f);
  return SerialChain<3>(
      links, ::mr::robotics::arm::toolbox_adapter::identity_transform(),
      ToolFrame(tool));
}

template <int N>
JointVec<N> joint_vec(const float (&values)[N]) {
  JointVec<N> q = ::mr::robotics::arm::toolbox_adapter::zero_joint_vec<N>();
  for (int i = 0; i < N; ++i) {
    q[i][0] = values[i];
  }
  return q;
}

JointState<3> joint_state3(const JointVec<3>& q) {
  JointState<3> state;
  state.valid = true;
  state.q = q;
  state.qd = ::mr::robotics::arm::toolbox_adapter::zero_joint_vec<3>();
  state.torque = ::mr::robotics::arm::toolbox_adapter::zero_joint_vec<3>();
  for (uint16_t i = 0; i < 3U; ++i) {
    state.online[i] = true;
  }
  return state;
}

bool joint_vec_close3(const JointVec<3>& lhs,
                      const JointVec<3>& rhs,
                      float tolerance) {
  for (uint16_t i = 0; i < 3U; ++i) {
    if (std::fabs(lhs[i][0] - rhs[i][0]) > tolerance) {
      return false;
    }
  }
  return true;
}

::mr::robotics::arm::application::ThreePitCartesianAppConfig
make_three_pit_app_config(const SerialChain<3>& chain) {
  namespace app = ::mr::robotics::arm::application;
  app::ThreePitCartesianAppConfig config;
  config.input_deadzone = 0.0f;
  config.workspace.y_min = -2.0f;
  config.workspace.y_max = 2.0f;
  config.workspace.z_min = -2.0f;
  config.workspace.z_max = 3.0f;
  config.workspace.pitch_min = -3.1f;
  config.workspace.pitch_max = 3.1f;
  config.max_y_velocity = 0.5f;
  config.max_z_velocity = 0.5f;
  config.max_pitch_velocity = 1.0f;
  config.max_linear_velocity = 1.0f;
  config.max_angular_velocity = 2.0f;
  config.max_linear_acceleration = 10.0f;
  config.max_angular_acceleration = 10.0f;
  config.joint_limits = chain.joint_limits();
  config.max_joint_step = 0.20f;
  for (uint16_t i = 0; i < 3U; ++i) {
    config.max_joint_velocity[i][0] = 5.0f;
    config.max_joint_acceleration[i][0] = 50.0f;
  }
  config.ik_options = ::mr::robotics::arm::kinematics::make_ik_options(
      ::mr::robotics::arm::kinematics::IkProfile::kEmbeddedSafe);
  config.ik_options.strategy =
      ::mr::robotics::arm::kinematics::IkSolveStrategy::kNumericOnly;
  config.ik_options.analytic_solver_preset =
      ::mr::robotics::arm::kinematics::AnalyticSolverPreset::kDisabled;
  config.ik_options.error_tolerance = 2.0e-3f;
  config.ik_options.max_iterations = 80U;
  config.ik_options.max_retries = 3U;
  config.ik_options.enable_multi_start = true;
  return config;
}

template <int N>
bool pose_close(const SerialChain<N>& chain,
                const Transform& target,
                const JointVec<N>& q,
                float position_tolerance,
                float orientation_tolerance) {
  const ::mr::robotics::arm::kinematics::PoseError6 error =
      ::mr::robotics::arm::kinematics::evaluate_pose_error(target,
                                               ::mr::robotics::arm::kinematics::fk(chain, q));
  return error.position_norm <= position_tolerance &&
         error.orientation_norm <= orientation_tolerance;
}

void test_planar_3r_round_trip() {
  const SerialChain<3> chain = make_planar_3r_chain();
  check(::mr::robotics::arm::kinematics::analytic::is_planar_3r_compatible(chain),
        "Planar3R compatibility");

  const float samples[][3] = {
      {0.0f, 0.0f, 0.0f},
      {0.35f, -0.55f, 0.25f},
      {-0.75f, 0.45f, -0.30f},
      {0.90f, -0.65f, 0.40f},
  };

  bool ok = true;
  for (const auto& sample : samples) {
    const JointVec<3> q = joint_vec(sample);
    ::mr::robotics::arm::kinematics::IKRequest<3> request;
    request.target = ::mr::robotics::arm::kinematics::fk(chain, q);
    request.seed = q;
    request.reference = q;
    request.current = q;
    request.use_seed = true;
    request.use_reference = true;
    request.use_current = true;

    ::mr::robotics::arm::kinematics::IKOptions options =
        ::mr::robotics::arm::kinematics::make_ik_options(
            ::mr::robotics::arm::kinematics::IkProfile::kRobust);
    options.analytic_solver_preset =
        ::mr::robotics::arm::kinematics::AnalyticSolverPreset::kPlanar3R;

    const ::mr::robotics::arm::kinematics::IKResult<3> result =
        ::mr::robotics::arm::kinematics::solve_ik(chain, request, options);
    ok = ok && ::mr::robotics::arm::is_success(result.status) &&
         pose_close(chain, request.target, result.q, 1.0e-4f, 1.0e-4f);
  }
  check(ok, "Planar3R FK->IK round trip");
}

void test_pieper_6r_round_trip() {
  const SerialChain<6> chain = make_pieper_6r_chain();
  check(::mr::robotics::arm::kinematics::analytic::is_pieper_6r_compatible(chain),
        "Pieper6R compatibility");

  const float samples[][6] = {
      {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      {0.20f, 0.30f, -0.35f, 0.25f, -0.30f, 0.35f},
      {-0.35f, 0.20f, 0.30f, -0.25f, 0.25f, -0.20f},
  };

  bool ok = true;
  for (const auto& sample : samples) {
    const JointVec<6> q = joint_vec(sample);
    ::mr::robotics::arm::kinematics::IKRequest<6> request;
    request.target = ::mr::robotics::arm::kinematics::fk(chain, q);
    request.seed = q;
    request.reference = q;
    request.current = q;
    request.use_seed = true;
    request.use_reference = true;
    request.use_current = true;

    ::mr::robotics::arm::kinematics::IKOptions options =
        ::mr::robotics::arm::kinematics::make_ik_options(
            ::mr::robotics::arm::kinematics::IkProfile::kRobust);
    options.analytic_solver_preset =
        ::mr::robotics::arm::kinematics::AnalyticSolverPreset::kPieper6R;
    options.error_tolerance = 1.0e-3f;

    const ::mr::robotics::arm::kinematics::IKResult<6> result =
        ::mr::robotics::arm::kinematics::solve_ik(chain, request, options);
    ok = ok && ::mr::robotics::arm::is_success(result.status) &&
         pose_close(chain, request.target, result.q, 2.0e-3f, 2.0e-3f);
  }
  check(ok, "Pieper6R FK->IK round trip");
}

void test_move_l_continuity() {
  const SerialChain<6> chain = make_pieper_6r_chain();
  ::mr::robotics::arm::controller::ArmController<6> controller;
  controller.set_chain(chain);
  controller.set_enabled(true);

  const float start_values[6] = {0.15f, 0.20f, -0.25f, 0.20f, -0.20f, 0.15f};
  const float goal_values[6] = {0.25f, 0.25f, -0.20f, 0.25f, -0.25f, 0.20f};
  JointVec<6> q = joint_vec(start_values);
  const JointVec<6> q_goal = joint_vec(goal_values);

  JointState<6> state;
  state.valid = true;
  state.q = q;
  for (uint16_t i = 0; i < 6U; ++i) {
    state.online[i] = true;
  }
  controller.update_feedback(state);

  ::mr::robotics::arm::controller::MoveLControllerRequest<6> request;
  request.cartesian.start = ::mr::robotics::arm::kinematics::fk(chain, q);
  request.cartesian.goal = ::mr::robotics::arm::kinematics::fk(chain, q_goal);
  request.cartesian.max_linear_velocity = 0.08f;
  request.cartesian.max_angular_velocity = 0.6f;
  request.cartesian.max_linear_acceleration = 0.3f;
  request.cartesian.max_angular_acceleration = 1.2f;
  request.limits = chain.joint_limits();
  request.enforce_limits = true;
  request.limit_joint_step = true;
  request.max_joint_step = 0.12f;
  request.ik_options = ::mr::robotics::arm::kinematics::make_ik_options(
      ::mr::robotics::arm::kinematics::IkProfile::kRobust);
  for (uint16_t i = 0; i < 6U; ++i) {
    request.max_velocity[i][0] = 10.0f;
    request.max_acceleration[i][0] = 100.0f;
  }

  bool ok = ::mr::robotics::arm::is_ok(controller.start_move_l(request));
  const char* reason = ok ? "" : "start_move_l failed";
  JointVec<6> previous = q;
  for (uint16_t step = 0; ok && step < 220U; ++step) {
    controller.update_feedback(state);
    ok = ::mr::robotics::arm::is_ok(controller.update(0.01f));
    if (!ok) {
      reason = "controller.update failed";
      break;
    }
    const ::mr::robotics::arm::JointCommand<6>& command = controller.command();
    if (!command.valid) {
      ok = false;
      reason = "invalid command";
      break;
    }
    const Scalar max_delta =
        ::mr::robotics::arm::kinematics::max_ik_joint_delta(chain, previous, command.q);
    if (max_delta > request.max_joint_step + 1.0e-3f ||
        !::mr::robotics::arm::joint_command_is_finite(command)) {
      ok = false;
      reason = "joint step or finite command check failed";
      break;
    }

    state.q = command.q;
    state.qd = command.qd;
    q = command.q;
    previous = command.q;
    if (controller.motion_state() ==
        ::mr::robotics::arm::controller::ArmMotionState::kReached) {
      break;
    }
  }

  ok = ok && controller.motion_state() ==
                 ::mr::robotics::arm::controller::ArmMotionState::kReached;
  if (!ok && reason[0] == '\0') {
    reason = "motion did not reach within test horizon";
  }
  if (!ok) {
    std::printf("MoveL continuity detail: %s, status=%u, safety=%u, ik=%u\n",
                reason,
                static_cast<unsigned>(controller.diagnostics().status),
                static_cast<unsigned>(controller.diagnostics().safety_reason),
                static_cast<unsigned>(controller.diagnostics().ik_status));
  }
  check(ok, "MoveL command continuity");
}

void test_move_j_synchronized_profile() {
  ::mr::robotics::arm::planning::MoveJRequest<3> request;
  const float start_values[3] = {0.0f, 0.0f, 0.0f};
  const float goal_values[3] = {1.0f, 0.25f, -0.5f};
  request.start = joint_vec(start_values);
  request.goal = joint_vec(goal_values);
  for (uint16_t i = 0; i < 3U; ++i) {
    request.max_velocity[i][0] = 1.0f;
    request.max_acceleration[i][0] = 1.0f;
  }

  ::mr::robotics::arm::planning::MoveJPlanner<3> planner;
  bool ok = planner.configure(request);
  ok = ok && near(planner.duration(), 2.0f, 1.0e-4f);

  const ::mr::robotics::arm::planning::MoveJSample<3> mid =
      planner.sample(0.5f * planner.duration());
  ok = ok && mid.valid && !mid.finished;
  for (uint16_t i = 0; i < 3U; ++i) {
    ok = ok && std::fabs(mid.velocity[i][0]) <=
                   request.max_velocity[i][0] + 1.0e-5f;
    ok = ok && std::fabs(mid.acceleration[i][0]) <=
                   request.max_acceleration[i][0] + 1.0e-5f;
  }

  const ::mr::robotics::arm::planning::MoveJSample<3> end =
      planner.sample(planner.duration());
  ok = ok && end.valid && end.finished;
  for (uint16_t i = 0; i < 3U; ++i) {
    ok = ok && near(end.position[i][0], request.goal[i][0], 1.0e-5f);
    ok = ok && near(end.velocity[i][0], 0.0f, 1.0e-6f);
    ok = ok && near(end.acceleration[i][0], 0.0f, 1.0e-6f);
  }

  check(ok, "MoveJ synchronized trapezoid profile");
}

void test_joint_limit_application_modes() {
  const float q_values[1] = {7.0f};
  const JointVec<1> q = joint_vec(q_values);

  ::mr::robotics::arm::JointLimits<1> tight_limits;
  tight_limits.set(0, -2.0f, 2.0f);

  const JointVec<1> clamp_only =
      ::mr::robotics::arm::apply_joint_limits(tight_limits, q);
  bool ok = near(clamp_only[0][0], 2.0f, 1.0e-6f);

  ::mr::robotics::arm::JointRangeProjection<1> modulo_range;
  modulo_range.set(0, -kPi, kPi);
  const JointVec<1> wrap_then_clamp =
      ::mr::robotics::arm::apply_joint_limits(
          tight_limits, q,
          ::mr::robotics::arm::JointLimitApplicationMode::kWrapThenClamp,
          &modulo_range);
  ok = ok && near(wrap_then_clamp[0][0], 7.0f - 2.0f * kPi, 1.0e-5f);

  ::mr::robotics::arm::JointLimits<1> multi_turn_limits;
  multi_turn_limits.set(0, -4.0f * kPi, 4.0f * kPi);
  const JointVec<1> absolute_multi_turn =
      ::mr::robotics::arm::apply_joint_limits(multi_turn_limits, q);
  ok = ok && near(absolute_multi_turn[0][0], 7.0f, 1.0e-6f);

  check(ok, "Joint limit clamp and wrap-then-clamp modes");
}

void test_servo_l_singularity_scaling() {
  const SerialChain<6> chain = make_pieper_6r_chain();
  const float q_values[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  JointState<6> state;
  state.valid = true;
  state.q = joint_vec(q_values);
  for (uint16_t i = 0; i < 6U; ++i) {
    state.online[i] = true;
  }

  ::mr::robotics::arm::servo::ServoLController<6> controller;
  ::mr::robotics::arm::servo::ServoLRequest<6> request;
  request.target_pose = ::mr::robotics::arm::kinematics::fk(chain, state.q);
  request.config.dls.damping = 1.0e-3f;
  request.config.dls.singularity_threshold = 100.0f;
  request.config.singularity_slowdown_threshold = 100.0f;
  request.config.enable_singularity_velocity_scaling = true;

  bool ok = controller.configure(request);
  const ::mr::robotics::arm::servo::ServoLResult<6> result =
      controller.step(chain, state, 0.001f);
  ok = ok && result.ok && result.singularity_velocity_scaled &&
       result.singularity_velocity_scale < 1.0f &&
       result.singularity_velocity_scale >= 0.0f;
  check(ok, "ServoL singularity velocity scaling");
}

void test_three_pit_cartesian_app_reachable_target_updates_output() {
  namespace app = ::mr::robotics::arm::application;
  const SerialChain<3> chain = make_three_pit_chain();
  app::ThreePitCartesianApp controller;
  app::ThreePitCartesianAppConfig config = make_three_pit_app_config(chain);
  const float q_values[3] = {0.10f, -0.25f, 0.15f};
  JointState<3> state = joint_state3(joint_vec(q_values));

  bool ok = controller.configure(chain, config);
  ok = ok && controller.reset_from_feedback(state);
  const app::YzPitchPose initial_target = controller.target_yz_pitch();

  app::ThreePitRemoteInput input;
  input.enabled = true;
  input.y_axis = 1.0f;
  const app::ThreePitCartesianAppResult result =
      controller.update(state, input, 0.10f);

  ok = ok && result.command.valid &&
       result.state == app::ThreePitCartesianAppState::kTracking &&
       result.target_yz_pitch.y > initial_target.y + 1.0e-4f &&
       !joint_vec_close3(result.command.q, state.q, 1.0e-6f);
  if (!ok) {
    std::printf(
        "3pit reachable detail: state=%u hold=%u target_ik=%u step_ik=%u "
        "target_y=%.6f initial_y=%.6f dq=(%.9g,%.9g,%.9g)\n",
        static_cast<unsigned>(result.state),
        static_cast<unsigned>(result.hold_reason),
        static_cast<unsigned>(result.target_ik_result.status),
        static_cast<unsigned>(result.step_ik_result.status),
        result.target_yz_pitch.y, initial_target.y,
        result.command.q[0][0] - state.q[0][0],
        result.command.q[1][0] - state.q[1][0],
        result.command.q[2][0] - state.q[2][0]);
  }
  check(ok, "3pit Cartesian app reachable target updates output");
}

void test_three_pit_cartesian_app_rejects_workspace_without_target_change() {
  namespace app = ::mr::robotics::arm::application;
  const SerialChain<3> chain = make_three_pit_chain();
  app::ThreePitCartesianApp controller;
  app::ThreePitCartesianAppConfig config = make_three_pit_app_config(chain);
  const float q_values[3] = {0.10f, -0.25f, 0.15f};
  JointState<3> state = joint_state3(joint_vec(q_values));

  bool ok = controller.configure(chain, config);
  ok = ok && controller.reset_from_feedback(state);
  const app::YzPitchPose initial_target = controller.target_yz_pitch();
  config.workspace.y_max = initial_target.y + 0.001f;
  controller.set_config(config);

  app::ThreePitRemoteInput input;
  input.enabled = true;
  input.y_axis = 1.0f;
  const app::ThreePitCartesianAppResult result =
      controller.update(state, input, 0.02f);

  ok = ok && result.candidate_rejected_out_of_range &&
       near(controller.target_yz_pitch().y, initial_target.y, 1.0e-6f) &&
       near(controller.target_yz_pitch().z, initial_target.z, 1.0e-6f) &&
       near(controller.target_yz_pitch().pitch, initial_target.pitch,
            1.0e-6f);
  check(ok, "3pit Cartesian app rejects workspace target change");
}

void test_three_pit_cartesian_app_unreachable_target_holds_output() {
  namespace app = ::mr::robotics::arm::application;
  const SerialChain<3> chain = make_three_pit_chain();
  app::ThreePitCartesianApp controller;
  app::ThreePitCartesianAppConfig config = make_three_pit_app_config(chain);
  config.max_z_velocity = 20.0f;
  config.workspace.z_max = 3.0f;
  const float q_values[3] = {0.10f, -0.25f, 0.15f};
  JointState<3> state = joint_state3(joint_vec(q_values));

  bool ok = controller.configure(chain, config);
  ok = ok && controller.reset_from_feedback(state);
  const app::YzPitchPose initial_target = controller.target_yz_pitch();
  const JointVec<3> hold_q = controller.last_valid_q();

  app::ThreePitRemoteInput input;
  input.enabled = true;
  input.z_axis = 1.0f;
  const app::ThreePitCartesianAppResult result =
      controller.update(state, input, 0.10f);

  ok = ok && result.target_updated &&
       result.state == app::ThreePitCartesianAppState::kTargetUnreachableHold &&
       controller.target_yz_pitch().z > initial_target.z + 1.0f &&
       joint_vec_close3(result.command.q, hold_q, 1.0e-6f) &&
       joint_vec_close3(controller.last_valid_q(), hold_q, 1.0e-6f);
  check(ok, "3pit Cartesian app in-range unreachable target holds output");
}

void test_three_pit_cartesian_app_recovers_after_target_reachable() {
  namespace app = ::mr::robotics::arm::application;
  const SerialChain<3> chain = make_three_pit_chain();
  app::ThreePitCartesianApp controller;
  app::ThreePitCartesianAppConfig config = make_three_pit_app_config(chain);
  config.max_z_velocity = 20.0f;
  config.workspace.z_max = 3.0f;
  const float q_values[3] = {0.10f, -0.25f, 0.15f};
  JointState<3> state = joint_state3(joint_vec(q_values));

  bool ok = controller.configure(chain, config);
  ok = ok && controller.reset_from_feedback(state);
  const app::YzPitchPose initial_target = controller.target_yz_pitch();
  const JointVec<3> hold_q = controller.last_valid_q();

  app::ThreePitRemoteInput input;
  input.enabled = true;
  input.z_axis = 1.0f;
  (void)controller.update(state, input, 0.10f);

  input.y_axis = 1.0f;
  input.z_axis = -1.0f;
  const app::ThreePitCartesianAppResult result =
      controller.update(state, input, 0.10f);

  ok = ok && result.command.valid &&
       result.state == app::ThreePitCartesianAppState::kTracking &&
       near(controller.target_yz_pitch().z, initial_target.z, 2.0e-3f) &&
       controller.target_yz_pitch().y > initial_target.y + 0.01f &&
       !joint_vec_close3(result.command.q, hold_q, 1.0e-6f);
  if (!ok) {
    std::printf(
        "3pit recover detail: state=%u hold=%u target_ik=%u step_ik=%u "
        "target_z=%.6f initial_z=%.6f dq=(%.9g,%.9g,%.9g)\n",
        static_cast<unsigned>(result.state),
        static_cast<unsigned>(result.hold_reason),
        static_cast<unsigned>(result.target_ik_result.status),
        static_cast<unsigned>(result.step_ik_result.status),
        controller.target_yz_pitch().z, initial_target.z,
        result.command.q[0][0] - hold_q[0][0],
        result.command.q[1][0] - hold_q[1][0],
        result.command.q[2][0] - hold_q[2][0]);
  }
  check(ok, "3pit Cartesian app resumes after reachable target returns");
}

void test_three_pit_cartesian_app_pitch_follows_joint3_monotonically() {
  namespace app = ::mr::robotics::arm::application;
  const SerialChain<3> chain = make_three_pit_chain();
  app::ThreePitCartesianApp controller;
  app::ThreePitCartesianAppConfig config = make_three_pit_app_config(chain);
  const float q3_samples[7] = {
      -1.95f, -1.30f, -0.65f, 0.0f, 0.65f, 1.30f, 1.95f};

  bool ok = controller.configure(chain, config);
  float previous_pitch = 0.0f;
  for (uint16_t i = 0; ok && i < 7U; ++i) {
    const float q_values[3] = {0.10f, -0.25f, q3_samples[i]};
    JointState<3> state = joint_state3(joint_vec(q_values));
    ok = ok && controller.reset_from_feedback(state);

    const float expected_pitch =
        q_values[0] + q_values[1] + q_values[2];
    const float pitch = controller.target_yz_pitch().pitch;
    ok = ok && near(pitch, expected_pitch, 1.0e-6f);
    if (i > 0U) {
      ok = ok && pitch > previous_pitch + 1.0e-4f;
    }
    previous_pitch = pitch;
  }

  if (!ok) {
    std::printf("3pit pitch monotonic detail: last_pitch=%.6f\n",
                previous_pitch);
  }
  check(ok, "3pit Cartesian app pitch follows joint3 monotonically");
}

void test_three_pit_cartesian_app_output_respects_joint_limits() {
  namespace app = ::mr::robotics::arm::application;
  const SerialChain<3> chain = make_three_pit_chain();
  app::ThreePitCartesianApp controller;
  app::ThreePitCartesianAppConfig config = make_three_pit_app_config(chain);
  config.max_y_velocity = 4.0f;
  config.max_linear_velocity = 5.0f;
  config.max_linear_acceleration = 50.0f;
  config.max_joint_step = 0.003f;
  for (uint16_t i = 0; i < 3U; ++i) {
    config.max_joint_velocity[i][0] = 0.05f;
    config.max_joint_acceleration[i][0] = 0.20f;
  }
  const float q_values[3] = {0.10f, -0.25f, 0.15f};
  JointState<3> state = joint_state3(joint_vec(q_values));

  bool ok = controller.configure(chain, config);
  ok = ok && controller.reset_from_feedback(state);

  app::ThreePitRemoteInput input;
  input.enabled = true;
  input.y_axis = 1.0f;
  const float dt = 0.02f;
  const app::ThreePitCartesianAppResult result =
      controller.update(state, input, dt);

  ok = ok && result.command.valid && result.safety_result.ok &&
       ::mr::robotics::arm::is_within_joint_limits(
           chain.joint_limits(), result.command.q, 1.0e-5f);
  for (uint16_t i = 0; ok && i < 3U; ++i) {
    const float step = result.command.q[i][0] - state.q[i][0];
    ok = ok &&
         std::fabs(result.command.qd[i][0]) <=
             config.max_joint_velocity[i][0] + 1.0e-5f &&
         std::fabs(result.command.qdd[i][0]) <=
             config.max_joint_acceleration[i][0] + 1.0e-5f &&
         std::fabs(step) <= config.max_joint_step + 1.0e-5f;
  }
  check(ok, "3pit Cartesian app output respects joint safety limits");
}

}  // namespace

int main() {
  test_planar_3r_round_trip();
  test_pieper_6r_round_trip();
  test_move_j_synchronized_profile();
  test_joint_limit_application_modes();
  test_move_l_continuity();
  test_servo_l_singularity_scaling();
  test_three_pit_cartesian_app_reachable_target_updates_output();
  test_three_pit_cartesian_app_rejects_workspace_without_target_change();
  test_three_pit_cartesian_app_unreachable_target_holds_output();
  test_three_pit_cartesian_app_recovers_after_target_reachable();
  test_three_pit_cartesian_app_pitch_follows_joint3_monotonically();
  test_three_pit_cartesian_app_output_respects_joint_limits();

  if (g_failures != 0) {
    std::printf("robotics/arm algorithm checks failed: %d\n", g_failures);
    return 1;
  }

  std::printf("robotics/arm algorithm checks passed\n");
  return 0;
}
