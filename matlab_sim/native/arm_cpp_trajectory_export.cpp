#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "robotics/arm/control/joint_servo.h"
#include "robotics/arm/core/arm_command.h"
#include "robotics/arm/frames/arm_frames.h"
#include "robotics/arm/kinematics/fk.h"
#include "robotics/arm/kinematics/ik_dispatch.h"
#include "robotics/arm/kinematics/jacobian.h"
#include "robotics/arm/kinematics/pose_error.h"
#include "robotics/arm/kinematics/task_constraints.h"
#include "robotics/arm/model/chain_builder.h"
#include "robotics/arm/planning/move_j_planner.h"
#include "robotics/arm/safety/joint_command_safety.h"
#include "robotics/arm/servo/servo_l_controller.h"
#include "robotics/arm/trajectory/cartesian_traj.h"

namespace arm = mr::robotics::arm;

constexpr int kDof = 3;
constexpr int kInputRows = 12;
constexpr int kInputCols = 6;

struct InputData {
  float q0[kDof];
  float q1[kDof];
  float max_velocity[kDof];
  float max_acceleration[kDof];
  float movej_dt;
  float cart_start_pose[6];
  float cart_goal_pose[6];
  float cart_limits[4];
  float cart_dt;
  float frame_delta_pose[6];
  int ik_mode;
  float control_duration;
  float max_joint_step;
  float position_tolerance;
  float orientation_tolerance;
  int control_frame;
};

bool read_csv_row(FILE* file, float* row, int cols) {
  for (int i = 0; i < cols; ++i) {
    if (std::fscanf(file, " %f", &row[i]) != 1) {
      return false;
    }
    if (i + 1 < cols) {
      (void)std::fscanf(file, " ,");
    }
  }
  (void)std::fscanf(file, " \n");
  return true;
}

bool read_input(const char* path, InputData* input) {
  if (input == nullptr) {
    return false;
  }

  FILE* file = std::fopen(path, "r");
  if (file == nullptr) {
    return false;
  }

  float rows[kInputRows][kInputCols] = {};
  for (int i = 0; i < kInputRows; ++i) {
    if (!read_csv_row(file, rows[i], kInputCols)) {
      std::fclose(file);
      return false;
    }
  }
  std::fclose(file);

  for (int i = 0; i < kDof; ++i) {
    input->q0[i] = rows[0][i];
    input->q1[i] = rows[1][i];
    input->max_velocity[i] = rows[2][i];
    input->max_acceleration[i] = rows[3][i];
  }
  for (int i = 0; i < 6; ++i) {
    input->cart_start_pose[i] = rows[5][i];
    input->cart_goal_pose[i] = rows[6][i];
    input->frame_delta_pose[i] = rows[9][i];
  }
  input->movej_dt = rows[4][0];
  for (int i = 0; i < 4; ++i) {
    input->cart_limits[i] = rows[7][i];
  }
  input->cart_dt = rows[8][0];
  input->ik_mode = static_cast<int>(rows[10][0]);
  input->control_duration = rows[11][0];
  input->max_joint_step = rows[11][1];
  input->position_tolerance = rows[11][2];
  input->orientation_tolerance = rows[11][3];
  input->control_frame = static_cast<int>(rows[11][4]);
  return true;
}

template <int N>
arm::JointVec<N> make_joint_vec(const float* values) {
  arm::JointVec<N> out = arm::toolbox_adapter::zero_joint_vec<N>();
  for (int i = 0; i < N; ++i) {
    out[i][0] = values[i];
  }
  return out;
}

arm::Vec3 make_vec3(float a, float b, float c) {
  arm::Vec3 out = arm::toolbox_adapter::zero_vec3();
  out[0][0] = a;
  out[1][0] = b;
  out[2][0] = c;
  return out;
}

arm::Transform pose_to_transform(const float* pose) {
  const arm::Vec3 translation = make_vec3(pose[0], pose[1], pose[2]);
  const arm::Vec3 rpy = make_vec3(pose[5], pose[4], pose[3]);
  return arm::toolbox_adapter::transform_from_rpy_translation(rpy, translation);
}

void transform_to_pose(const arm::Transform& transform, float* pose) {
  const arm::Vec3 translation = arm::toolbox_adapter::translation_of(transform);
  const arm::Vec3 rpy =
      arm::toolbox_adapter::rpy_from_rotation(
          arm::toolbox_adapter::rotation_of(transform));
  pose[0] = translation[0][0];
  pose[1] = translation[1][0];
  pose[2] = translation[2][0];
  pose[3] = rpy[2][0];
  pose[4] = rpy[1][0];
  pose[5] = rpy[0][0];
}

arm::ChainJointSpec make_revolute_joint(float lower, float upper) {
  arm::ChainJointSpec joint;
  joint.type = arm::ChainJointType::kRevolute;
  joint.limit_enabled = true;
  joint.lower = lower;
  joint.upper = upper;
  joint.participate_in_ik = true;
  return joint;
}

arm::Transform make_origin(float x,
                           float y,
                           float z,
                           float roll,
                           float pitch,
                           float yaw) {
  const float pose[6] = {x, y, z, roll, pitch, yaw};
  return pose_to_transform(pose);
}

arm::SerialChain<kDof> make_3pit_chain() {
  arm::Link links[kDof];
  const arm::Vec3 negative_z_axis = make_vec3(0.0f, 0.0f, -1.0f);

  links[0] = arm::Link(
      make_origin(0.0f, 0.0f, 0.07f, 1.5708f, 0.0f, -1.5708f),
      negative_z_axis, make_revolute_joint(-1.57f, 1.57f));
  links[1] = arm::Link(
      make_origin(-0.0005f, 0.35f, 0.0f, 0.0f, 0.0f, 0.0f),
      negative_z_axis, make_revolute_joint(-2.32f, 2.32f));
  links[2] = arm::Link(
      make_origin(-0.000357142857063231f, 0.24999974489785f,
                  -0.00262500000000032f, 0.0f, 0.0f, 0.0f),
      negative_z_axis, make_revolute_joint(-1.95f, 1.95f));

  const arm::Transform tool =
      make_origin(0.0034791f, 0.14696f, -0.000025f, 0.0f, 0.0f, 0.0f);

  return arm::SerialChain<kDof>(
      links, arm::toolbox_adapter::identity_transform(), arm::ToolFrame(tool));
}

arm::kinematics::IKOptions make_3pit_ik_options(
    const arm::Transform& target,
    const InputData& input,
    bool control_step = false) {
  arm::kinematics::IKOptions options;
  options = arm::kinematics::make_yz_pitch_ik_options(
      target,
      control_step ? arm::kinematics::IkProfile::kEmbeddedSafe
                   : arm::kinematics::IkProfile::kPositionOnly,
      1.0f);

  options.strategy = arm::kinematics::IkSolveStrategy::kNumericOnly;
  options.analytic_solver_preset =
      arm::kinematics::AnalyticSolverPreset::kDisabled;
  options.max_iterations = control_step ? 48U : 120U;
  options.max_retries = control_step ? 0U : 6U;
  const float position_tolerance =
      (input.position_tolerance > 0.0f) ? input.position_tolerance : 1.0e-3f;
  const float orientation_tolerance =
      (input.orientation_tolerance > 0.0f) ? input.orientation_tolerance : 1.0e-2f;
  options.error_tolerance =
      (orientation_tolerance > position_tolerance) ? orientation_tolerance
                                                   : position_tolerance;
  options.damping = 3.0e-3f;
  options.enable_multi_start = !control_step;
  options.enable_workspace_precheck = false;
  options.enable_model_validation = true;
  options.enable_null_space_control = true;
  options.enable_joint_centering_objective = true;
  options.joint_centering_gain = 0.08f;
  options.enable_reference_objective = true;
  options.reference_bias_gain = 0.08f;
  options.enable_manipulability_objective = true;
  options.manipulability_gain = 0.01f;
  options.enable_singularity_avoidance_objective = true;
  options.singularity_avoidance_gain = 0.04f;
  options.singularity_threshold = 5.0e-3f;
  options.singularity_damping_scale = 20.0f;
  if (input.max_joint_step > 0.0f) {
    options.enable_solution_step_limit = true;
    options.max_solution_joint_step = input.max_joint_step;
  }
  return options;
}

float angle_distance_float(float from, float to) {
  return arm::angle_distance(from, to);
}

float task_error_norm_yz_pitch(const arm::SerialChain<kDof>& chain,
                               const arm::Transform& target,
                               const arm::JointVec<kDof>& q,
                               float* pos_error_out = nullptr,
                               float* pitch_error_out = nullptr) {
  float target_pose[6] = {};
  float actual_pose[6] = {};
  transform_to_pose(target, target_pose);
  transform_to_pose(arm::kinematics::fk(chain, q), actual_pose);
  const float dy = target_pose[1] - actual_pose[1];
  const float dz = target_pose[2] - actual_pose[2];
  const float dpitch = angle_distance_float(actual_pose[4], target_pose[4]);
  const float pos_error = std::sqrt(dy * dy + dz * dz);
  const float pitch_error = std::fabs(dpitch);
  if (pos_error_out != nullptr) {
    *pos_error_out = pos_error;
  }
  if (pitch_error_out != nullptr) {
    *pitch_error_out = pitch_error;
  }
  return std::sqrt(pos_error * pos_error + pitch_error * pitch_error);
}

arm::JointVec<kDof> align_to_seed(const arm::JointVec<kDof>& q,
                                  const arm::JointVec<kDof>& seed) {
  arm::JointVec<kDof> out = q;
  for (int i = 0; i < kDof; ++i) {
    out[i][0] = seed[i][0] + arm::angle_distance(seed[i][0], q[i][0]);
  }
  return out;
}

arm::kinematics::IKResult<kDof> solve_target_ik_numeric(
    const arm::SerialChain<kDof>& chain,
    const arm::Transform& target,
    const arm::JointVec<kDof>& seed,
    const InputData& input,
    bool control_step = false) {
  arm::kinematics::IKRequest<kDof> request;
  request.target = target;
  request.seed = seed;
  request.current = seed;
  request.reference = seed;
  request.use_seed = true;
  request.use_current = true;
  request.use_reference = true;
  return arm::kinematics::solve_ik(chain, request,
                                   make_3pit_ik_options(target, input, control_step));
}

arm::kinematics::IKResult<kDof> solve_target_ik_analytic(
    const arm::SerialChain<kDof>& chain,
    const arm::Transform& target,
    const arm::JointVec<kDof>& seed,
    const InputData& input) {
  arm::kinematics::IKResult<kDof> result;
  float target_pose[6] = {};
  float home_pose[6] = {};
  transform_to_pose(target, target_pose);
  transform_to_pose(arm::kinematics::fk(
                        chain, arm::toolbox_adapter::zero_joint_vec<kDof>()),
                    home_pose);

  const float l1 = 0.35f;
  const float l2 = 0.25f;
  const float l3 = 0.14696f;
  const float y0 = home_pose[1];
  const float z_base = home_pose[2] - l1 - l2 - l3;
  const float phi = target_pose[4];
  const float y_local = -(target_pose[1] - y0) - l3 * std::sin(phi);
  const float z_local = target_pose[2] - z_base - l3 * std::cos(phi);
  const float den = 2.0f * l1 * l2;
  float c2 = ((y_local * y_local) + (z_local * z_local) -
              (l1 * l1) - (l2 * l2)) / den;
  if (!std::isfinite(c2) || c2 < -1.0f - 1.0e-5f ||
      c2 > 1.0f + 1.0e-5f) {
    result.status = arm::IkStatus::kUnreachable;
    result.diagnostics.stage = arm::IkFailStage::kAnalyticSolve;
    result.diagnostics.reason = arm::IkFailureReason::kUnreachable;
    result.q = seed;
    return result;
  }
  c2 = arm::clamp_scalar(c2, -1.0f, 1.0f);
  const float s2_abs = std::sqrt(arm::clamp_scalar(1.0f - c2 * c2, 0.0f, 1.0f));
  const float s2_values[2] = {s2_abs, -s2_abs};

  bool found = false;
  float best_score = 0.0f;
  arm::JointVec<kDof> best_q = seed;
  float best_pos = 0.0f;
  float best_pitch = 0.0f;
  const arm::JointLimits<kDof> limits = chain.joint_limits();

  for (int i = 0; i < 2; ++i) {
    const float s2 = s2_values[i];
    const float q2 = std::atan2(s2, c2);
    const float q1 =
        std::atan2(y_local, z_local) -
        std::atan2(l2 * s2, l1 + l2 * c2);
    const float q3 = phi - q1 - q2;
    arm::JointVec<kDof> q = arm::toolbox_adapter::zero_joint_vec<kDof>();
    q[0][0] = arm::wrap_to_pi(q1);
    q[1][0] = arm::wrap_to_pi(q2);
    q[2][0] = arm::wrap_to_pi(q3);
    q = align_to_seed(q, seed);
    if (!arm::is_within_joint_limits(limits, q, 1.0e-5f)) {
      continue;
    }
    float pos = 0.0f;
    float pitch = 0.0f;
    const float task = task_error_norm_yz_pitch(chain, target, q, &pos, &pitch);
    const float score = task + 1.0e-3f * (q - seed).norm();
    if (!found || score < best_score) {
      found = true;
      best_score = score;
      best_q = q;
      best_pos = pos;
      best_pitch = pitch;
    }
  }

  result.q = best_q;
  result.stats.iterations = 1U;
  result.stats.final_error_norm = best_score;
  result.limit_violation = arm::total_limit_violation(limits, best_q);
  result.singularity_metric =
      arm::kinematics::singularity_metric(arm::kinematics::jacobian(chain, best_q));
  result.used_analytic = true;
  result.diagnostics.used_analytic = true;
  result.diagnostics.final_error_norm = best_score;
  result.diagnostics.iterations = 1U;
  if (!found) {
    result.status = arm::IkStatus::kLimitViolation;
    result.diagnostics.stage = arm::IkFailStage::kAnalyticSolve;
    result.diagnostics.reason = arm::IkFailureReason::kLimitViolation;
    return result;
  }
  const float pos_tol =
      (input.position_tolerance > 0.0f) ? input.position_tolerance : 1.0e-3f;
  const float pitch_tol =
      (input.orientation_tolerance > 0.0f) ? input.orientation_tolerance : 1.0e-2f;
  if (best_pos <= pos_tol && best_pitch <= pitch_tol) {
    result.status = arm::IkStatus::kSuccess;
  } else {
    result.status = arm::IkStatus::kNoConvergence;
    result.diagnostics.stage = arm::IkFailStage::kPostCheck;
    result.diagnostics.reason = arm::IkFailureReason::kNoConvergence;
  }
  return result;
}

arm::kinematics::IKResult<kDof> solve_target_ik(
    const arm::SerialChain<kDof>& chain,
    const arm::Transform& target,
    const arm::JointVec<kDof>& seed,
    const InputData& input,
    bool control_step = false) {
  if (control_step || input.ik_mode == 0) {
    return solve_target_ik_numeric(chain, target, seed, input, control_step);
  }
  if (input.ik_mode == 1) {
    return solve_target_ik_analytic(chain, target, seed, input);
  }

  const arm::kinematics::IKResult<kDof> analytic =
      solve_target_ik_analytic(chain, target, seed, input);
  arm::JointVec<kDof> refine_seed = seed;
  if (arm::is_success(analytic.status) ||
      analytic.status == arm::IkStatus::kNoConvergence) {
    refine_seed = analytic.q;
  }
  arm::kinematics::IKResult<kDof> numeric =
      solve_target_ik_numeric(chain, target, refine_seed, input, control_step);
  if (arm::is_success(numeric.status)) {
    numeric.used_analytic = analytic.used_analytic;
    numeric.used_numeric_refine = true;
    return numeric;
  }
  if (arm::is_success(analytic.status)) {
    return analytic;
  }
  numeric.used_numeric_fallback = true;
  return numeric;
}

arm::kinematics::PoseError6 constrained_error(
    const arm::Transform& target,
    const arm::Transform& actual) {
  const arm::kinematics::TaskConstraintProfile profile =
      arm::kinematics::make_yz_pitch_profile(1.0f);
  return arm::kinematics::evaluate_constrained_pose_error(
      target, actual, arm::kinematics::PoseErrorFrame::kSpatial, profile);
}

bool export_movej(const InputData& input, const char* output_path) {
  const arm::SerialChain<kDof> chain = make_3pit_chain();
  arm::planning::MoveJRequest<kDof> request;
  request.start = make_joint_vec<kDof>(input.q0);
  request.goal = make_joint_vec<kDof>(input.q1);
  request.max_velocity = make_joint_vec<kDof>(input.max_velocity);
  request.max_acceleration = make_joint_vec<kDof>(input.max_acceleration);
  request.limits = chain.joint_limits();
  request.enforce_limits = true;

  arm::planning::MoveJPlanner<kDof> planner;
  if (!planner.configure(request)) {
    return false;
  }

  FILE* file = std::fopen(output_path, "w");
  if (file == nullptr) {
    return false;
  }

  std::fprintf(file, "time");
  for (int i = 0; i < kDof; ++i) std::fprintf(file, ",q%d", i + 1);
  for (int i = 0; i < kDof; ++i) std::fprintf(file, ",qd%d", i + 1);
  for (int i = 0; i < kDof; ++i) std::fprintf(file, ",qdd%d", i + 1);
  std::fprintf(file, "\n");

  const float duration = planner.duration();
  const float dt = (input.movej_dt > 0.0f) ? input.movej_dt : 0.02f;
  for (float t = 0.0f; t < duration; t += dt) {
    const arm::planning::MoveJSample<kDof> sample = planner.sample(t);
    std::fprintf(file, "%.9g", t);
    for (int i = 0; i < kDof; ++i) std::fprintf(file, ",%.9g", sample.position[i][0]);
    for (int i = 0; i < kDof; ++i) std::fprintf(file, ",%.9g", sample.velocity[i][0]);
    for (int i = 0; i < kDof; ++i) std::fprintf(file, ",%.9g", sample.acceleration[i][0]);
    std::fprintf(file, "\n");
  }

  const arm::planning::MoveJSample<kDof> sample = planner.sample(duration);
  std::fprintf(file, "%.9g", duration);
  for (int i = 0; i < kDof; ++i) std::fprintf(file, ",%.9g", sample.position[i][0]);
  for (int i = 0; i < kDof; ++i) std::fprintf(file, ",%.9g", sample.velocity[i][0]);
  for (int i = 0; i < kDof; ++i) std::fprintf(file, ",%.9g", sample.acceleration[i][0]);
  std::fprintf(file, "\n");

  std::fclose(file);
  return true;
}

bool export_cartesian(const InputData& input,
                      const arm::Transform& start,
                      const arm::Transform& goal,
                      const char* output_path) {
  arm::trajectory::CartesianTrajectoryRequest request;
  request.start = start;
  request.goal = goal;
  request.max_linear_velocity = input.cart_limits[0];
  request.max_angular_velocity = input.cart_limits[1];
  request.max_linear_acceleration = input.cart_limits[2];
  request.max_angular_acceleration = input.cart_limits[3];

  arm::trajectory::CartesianTrajectory planner;
  if (!planner.configure(request)) {
    return false;
  }

  FILE* file = std::fopen(output_path, "w");
  if (file == nullptr) {
    return false;
  }

  std::fprintf(file,
               "time,x,y,z,roll,pitch,yaw,"
               "vx,vy,vz,wx,wy,wz,ax,ay,az,alphax,alphay,alphaz\n");

  const float duration = planner.duration();
  const float dt = (input.cart_dt > 0.0f) ? input.cart_dt : 0.02f;
  for (float t = 0.0f; t < duration; t += dt) {
    const arm::trajectory::CartesianTrajectorySample sample = planner.sample(t);
    float pose[6] = {};
    transform_to_pose(sample.pose, pose);
    std::fprintf(file, "%.9g", t);
    for (float value : pose) std::fprintf(file, ",%.9g", value);
    for (int i = 0; i < 6; ++i) std::fprintf(file, ",%.9g", sample.body_twist[i][0]);
    for (int i = 0; i < 6; ++i) std::fprintf(file, ",%.9g", sample.body_acceleration[i][0]);
    std::fprintf(file, "\n");
  }

  const arm::trajectory::CartesianTrajectorySample sample = planner.sample(duration);
  float pose[6] = {};
  transform_to_pose(sample.pose, pose);
  std::fprintf(file, "%.9g", duration);
  for (float value : pose) std::fprintf(file, ",%.9g", value);
  for (int i = 0; i < 6; ++i) std::fprintf(file, ",%.9g", sample.body_twist[i][0]);
  for (int i = 0; i < 6; ++i) std::fprintf(file, ",%.9g", sample.body_acceleration[i][0]);
  std::fprintf(file, "\n");

  std::fclose(file);
  return true;
}

arm::Transform resolve_planar_yz_pitch_delta(
    const arm::Transform& current_base_tcp,
    const InputData& input,
    arm::frames::CartesianCommandFrame frame) {
  float pose[6] = {};
  transform_to_pose(current_base_tcp, pose);

  const float dy = input.frame_delta_pose[1];
  const float dz = input.frame_delta_pose[2];
  const float dpitch = input.frame_delta_pose[4];
  if (frame == arm::frames::CartesianCommandFrame::kTool) {
    const float c = std::cos(pose[4]);
    const float s = std::sin(pose[4]);
    pose[1] += c * dy - s * dz;
    pose[2] += s * dy + c * dz;
  } else {
    pose[1] += dy;
    pose[2] += dz;
  }
  pose[4] += dpitch;
  return pose_to_transform(pose);
}

bool export_frame_targets(const InputData& input,
                          const arm::Transform& current_base_tcp,
                          const arm::Transform& absolute_goal,
                          const char* output_path) {
  const arm::Transform base_target =
      resolve_planar_yz_pitch_delta(
          current_base_tcp, input, arm::frames::CartesianCommandFrame::kBase);
  const arm::Transform tool_target =
      resolve_planar_yz_pitch_delta(
          current_base_tcp, input, arm::frames::CartesianCommandFrame::kTool);
  const arm::Transform targets[3] = {base_target, tool_target, absolute_goal};

  FILE* file = std::fopen(output_path, "w");
  if (file == nullptr) {
    return false;
  }

  std::fprintf(file, "frame,x,y,z,roll,pitch,yaw\n");
  for (int frame = 0; frame < 3; ++frame) {
    float pose[6] = {};
    transform_to_pose(targets[frame], pose);
    std::fprintf(file, "%d", frame);
    for (float value : pose) std::fprintf(file, ",%.9g", value);
    std::fprintf(file, "\n");
  }

  std::fclose(file);
  return true;
}

void write_ik_row(FILE* file,
                  int frame,
                  const arm::SerialChain<kDof>& chain,
                  const arm::Transform& target,
                  const arm::JointVec<kDof>& seed,
                  const InputData& input) {
  const arm::kinematics::IKResult<kDof> result =
      solve_target_ik(chain, target, seed, input);
  const arm::Transform achieved = arm::kinematics::fk(chain, result.q);
  const arm::kinematics::PoseError6 full_error =
      arm::kinematics::evaluate_pose_error(target, achieved);
  float task_position_error = 0.0f;
  float task_orientation_error = 0.0f;
  const float task_error = task_error_norm_yz_pitch(
      chain, target, result.q, &task_position_error, &task_orientation_error);
  const float position_tolerance =
      (input.position_tolerance > 0.0f) ? input.position_tolerance : 1.0e-3f;
  const float orientation_tolerance =
      (input.orientation_tolerance > 0.0f) ? input.orientation_tolerance : 1.0e-2f;
  const bool task_ok = task_position_error <= position_tolerance &&
                       task_orientation_error <= orientation_tolerance &&
                       result.limit_violation <= 1.0e-5f;
  arm::IkStatus row_status = result.status;
  arm::IkFailStage row_stage = result.diagnostics.stage;
  arm::IkFailureReason row_reason = result.diagnostics.reason;
  if (arm::is_success(row_status) && !task_ok) {
    row_status = arm::IkStatus::kNoConvergence;
    row_stage = arm::IkFailStage::kPostCheck;
    row_reason = arm::IkFailureReason::kNoConvergence;
  }
  float achieved_pose[6] = {};
  transform_to_pose(achieved, achieved_pose);

  std::fprintf(file,
               "%d,%d,%u,%u,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%d",
               frame,
               arm::is_success(row_status) ? 1 : 0,
               static_cast<unsigned>(row_status),
               static_cast<unsigned>(row_stage),
               static_cast<unsigned>(row_reason),
               static_cast<unsigned>(result.stats.iterations),
               task_error,
               full_error.position_norm,
               full_error.orientation_norm,
               task_position_error,
               task_orientation_error,
               result.singularity_metric,
               result.limit_violation,
               result.used_analytic ? 1 : 0,
               result.used_numeric_refine ? 1 : 0,
               result.used_numeric_fallback ? 1 : 0);
  for (int i = 0; i < kDof; ++i) {
    std::fprintf(file, ",%.9g", result.q[i][0]);
  }
  for (float value : achieved_pose) {
    std::fprintf(file, ",%.9g", value);
  }
  std::fprintf(file, "\n");
}

bool export_cpp_ik(const InputData& input,
                   const arm::Transform& current_base_tcp,
                   const arm::Transform& absolute_goal,
                   const char* output_path) {
  const arm::SerialChain<kDof> chain = make_3pit_chain();
  const arm::JointVec<kDof> seed = make_joint_vec<kDof>(input.q0);
  const arm::Transform targets[3] = {
      resolve_planar_yz_pitch_delta(
          current_base_tcp, input, arm::frames::CartesianCommandFrame::kBase),
      resolve_planar_yz_pitch_delta(
          current_base_tcp, input, arm::frames::CartesianCommandFrame::kTool),
      absolute_goal,
  };

  FILE* file = std::fopen(output_path, "w");
  if (file == nullptr) {
    return false;
  }

  std::fprintf(file,
               "frame,ok,status,fail_stage,failure_reason,iterations,final_error,"
               "position_error,orientation_error,task_position_error,task_orientation_error,"
               "singularity,limit_violation,used_analytic,used_numeric_refine,"
               "used_numeric_fallback,q1,q2,q3,"
               "achieved_x,achieved_y,achieved_z,"
               "achieved_roll,achieved_pitch,achieved_yaw\n");

  for (int frame = 0; frame < 3; ++frame) {
    write_ik_row(file, frame, chain, targets[frame], seed, input);
  }

  std::fclose(file);
  return true;
}

bool frame_delta_is_nonzero(const InputData& input) {
  float sum_abs = 0.0f;
  for (int i = 0; i < 6; ++i) {
    sum_abs += std::fabs(input.frame_delta_pose[i]);
  }
  return sum_abs > 1.0e-7f;
}

arm::Transform select_control_goal(const InputData& input,
                                   const arm::Transform& current_base_tcp) {
  if (!frame_delta_is_nonzero(input)) {
    return pose_to_transform(input.cart_goal_pose);
  }

  const arm::frames::CartesianCommandFrame frame =
      (input.control_frame == 1) ? arm::frames::CartesianCommandFrame::kTool
                                 : arm::frames::CartesianCommandFrame::kBase;
  return resolve_planar_yz_pitch_delta(current_base_tcp, input, frame);
}

bool export_control_loop(const InputData& input,
                         const arm::Transform& control_goal,
                         const char* output_path) {
  const arm::SerialChain<kDof> chain = make_3pit_chain();
  arm::JointVec<kDof> q = make_joint_vec<kDof>(input.q0);
  arm::JointVec<kDof> qd = arm::toolbox_adapter::zero_joint_vec<kDof>();

  FILE* file = std::fopen(output_path, "w");
  if (file == nullptr) {
    return false;
  }

  std::fprintf(file,
               "time,valid,reached,ik_ok,safety_ok,status,fail_stage,failure_reason,"
               "iterations,final_error,task_position_error,task_orientation_error,"
               "singularity,velocity_scale,joint_step_norm,"
               "q1,q2,q3,qd1,qd2,qd3,"
               "cmd_x,cmd_y,cmd_z,cmd_roll,cmd_pitch,cmd_yaw,"
               "achieved_x,achieved_y,achieved_z,achieved_roll,achieved_pitch,achieved_yaw\n");

  if (input.control_duration < 0.0f) {
    const arm::Transform achieved = arm::kinematics::fk(chain, q);
    const arm::kinematics::PoseError6 task_error =
        constrained_error(control_goal, achieved);
    const float final_task_error =
        std::sqrt(task_error.position_norm * task_error.position_norm +
                  task_error.orientation_norm * task_error.orientation_norm);
    float pose[6] = {};
    transform_to_pose(achieved, pose);
    std::fprintf(file,
                 "0,1,0,1,1,0,0,0,0,0,%.9g,%.9g,0,1,0",
                 task_error.position_norm, task_error.orientation_norm);
    for (int i = 0; i < kDof; ++i) std::fprintf(file, ",%.9g", q[i][0]);
    for (int i = 0; i < kDof; ++i) std::fprintf(file, ",0");
    for (float value : pose) std::fprintf(file, ",%.9g", value);
    for (float value : pose) std::fprintf(file, ",%.9g", value);
    std::fprintf(file, "\n");
    std::fclose(file);
    return true;
  }

  arm::trajectory::CartesianTrajectoryRequest request;
  request.start = arm::kinematics::fk(chain, q);
  request.goal = control_goal;
  request.max_linear_velocity = input.cart_limits[0];
  request.max_angular_velocity = input.cart_limits[1];
  request.max_linear_acceleration = input.cart_limits[2];
  request.max_angular_acceleration = input.cart_limits[3];

  arm::trajectory::CartesianTrajectory planner;
  if (!planner.configure(request)) {
    return false;
  }

  arm::servo::WeightedDlsConfig<kDof> dls_config;
  dls_config.task_weights = arm::toolbox_adapter::zero_twist();
  dls_config.task_weights[0][0] = 0.0f;
  dls_config.task_weights[1][0] = 1.0f;
  dls_config.task_weights[2][0] = 1.0f;
  dls_config.task_weights[3][0] = 0.0f;
  dls_config.task_weights[4][0] = 1.0f;
  dls_config.task_weights[5][0] = 0.0f;
  dls_config.damping = 5.0e-3f;
  dls_config.singularity_threshold = 5.0e-3f;
  dls_config.singularity_damping_scale = 25.0f;
  dls_config.limit_output_norm = true;
  dls_config.max_qdot_norm = 2.0f;

  const arm::JointVec<kDof> max_velocity = make_joint_vec<kDof>(input.max_velocity);
  const arm::JointVec<kDof> max_acceleration =
      make_joint_vec<kDof>(input.max_acceleration);
  const arm::JointLimits<kDof> joint_limits = chain.joint_limits();
  const float max_joint_step =
      (input.max_joint_step > 0.0f) ? input.max_joint_step : 0.08f;

  arm::safety::JointCommandSafetyConfig<kDof> safety_config;
  safety_config.position_limits = chain.joint_limits();
  safety_config.max_velocity = make_joint_vec<kDof>(input.max_velocity);
  safety_config.max_acceleration = make_joint_vec<kDof>(input.max_acceleration);
  safety_config.max_position_step = arm::toolbox_adapter::zero_joint_vec<kDof>();
  safety_config.check_position = true;
  safety_config.check_velocity = true;
  safety_config.check_joint_step = input.max_joint_step > 0.0f;
  for (int i = 0; i < kDof; ++i) {
    safety_config.max_position_step[i][0] = input.max_joint_step;
  }

  const float dt = (input.cart_dt > 0.0f) ? input.cart_dt : 0.02f;
  const float planned_duration = planner.duration();
  float control_duration = input.control_duration;
  if (control_duration <= 0.0f) {
    control_duration = planned_duration + 1.0f;
  }
  if (control_duration < planned_duration) {
    control_duration = planned_duration;
  }

  int max_steps = static_cast<int>(std::ceil(control_duration / dt)) + 1;
  if (max_steps < 2) {
    max_steps = 2;
  }

  for (int step = 0; step < max_steps; ++step) {
    const float t = step * dt;
    const arm::trajectory::CartesianTrajectorySample sample = planner.sample(t);
    const arm::Transform commanded_pose = sample.pose;
    float current_pose_array[6] = {};
    float commanded_pose_array[6] = {};
    transform_to_pose(arm::kinematics::fk(chain, q), current_pose_array);
    transform_to_pose(commanded_pose, commanded_pose_array);

    arm::Twist6 task_twist = arm::toolbox_adapter::zero_twist();
    task_twist[1][0] = 2.0f * (commanded_pose_array[1] - current_pose_array[1]);
    task_twist[2][0] = 2.0f * (commanded_pose_array[2] - current_pose_array[2]);
    task_twist[4][0] =
        1.5f * arm::angle_distance(current_pose_array[4], commanded_pose_array[4]);

    arm::Jacobian6xN<kDof> J = arm::kinematics::jacobian(chain, q);
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < kDof; ++col) {
        J[row][col] = 0.0f;
      }
    }
    constexpr float kTaskJacobianStep = 1.0e-4f;
    for (int joint = 0; joint < kDof; ++joint) {
      arm::JointVec<kDof> q_plus = q;
      arm::JointVec<kDof> q_minus = q;
      q_plus[joint][0] += kTaskJacobianStep;
      q_minus[joint][0] -= kTaskJacobianStep;
      if (joint_limits.enabled[joint]) {
        q_plus[joint][0] =
            arm::clamp_scalar(q_plus[joint][0],
                              joint_limits.lower[joint][0],
                              joint_limits.upper[joint][0]);
        q_minus[joint][0] =
            arm::clamp_scalar(q_minus[joint][0],
                              joint_limits.lower[joint][0],
                              joint_limits.upper[joint][0]);
      }
      const float denominator = q_plus[joint][0] - q_minus[joint][0];
      if (std::fabs(denominator) <= ARM_LIB_EPSILON) {
        continue;
      }
      float plus_pose[6] = {};
      float minus_pose[6] = {};
      transform_to_pose(arm::kinematics::fk(chain, q_plus), plus_pose);
      transform_to_pose(arm::kinematics::fk(chain, q_minus), minus_pose);
      J[1][joint] = (plus_pose[1] - minus_pose[1]) / denominator;
      J[2][joint] = (plus_pose[2] - minus_pose[2]) / denominator;
      J[4][joint] = arm::angle_distance(minus_pose[4], plus_pose[4]) /
                    denominator;
    }
    const arm::servo::WeightedDlsResult<kDof> dls_result =
        arm::servo::solve_weighted_dls(J, task_twist, dls_config);
    bool ik_ok = dls_result.ok;
    float velocity_scale = 1.0f;
    if (dls_config.singularity_threshold > ARM_LIB_EPSILON &&
        dls_result.singularity_metric < dls_config.singularity_threshold) {
      const float ratio =
          arm::clamp_scalar(dls_result.singularity_metric /
                                dls_config.singularity_threshold,
                            0.0f, 1.0f);
      velocity_scale = 0.25f + 0.75f * ratio;
    }

    arm::JointVec<kDof> qd_command =
        ik_ok ? (dls_result.qdot * velocity_scale)
              : arm::toolbox_adapter::zero_joint_vec<kDof>();
    bool limited = false;
    qd_command = arm::servo::clamp_joint_velocity(
        qd_command, max_velocity, &limited);
    qd_command = arm::servo::clamp_joint_acceleration(
        qd_command, qd, max_acceleration, dt, &limited);
    qd_command = arm::servo::clamp_joint_step(
        qd_command, max_joint_step, dt, &limited);
    qd_command = arm::servo::clamp_to_position_limits_as_velocity(
        q, qd_command, joint_limits, dt, &limited);

    arm::JointVec<kDof> q_command = q + qd_command * dt;
    arm::JointVec<kDof> qdd_command = (qd_command - qd) / dt;
    const bool servo_valid = ik_ok;

    arm::JointCommand<kDof> command;
    command.q = q_command;
    command.qd = qd_command;
    command.qdd = qdd_command;
    command.mode = arm::JointCommandMode::kPositionVelocityTorque;
    command.valid = servo_valid;
    safety_config.current_position = q;
    const arm::safety::JointCommandSafetyResult safety =
        arm::safety::validate_joint_command(command, safety_config, 1.0e-5f);
    const bool safety_ok = safety.ok;
    const arm::JointVec<kDof> step_delta = q_command - q;
    const float joint_step_norm = step_delta.norm();

    if (ik_ok && safety_ok) {
      q = q_command;
      qd = qd_command;
    } else {
      qd = arm::toolbox_adapter::zero_joint_vec<kDof>();
    }

    const arm::Transform achieved = arm::kinematics::fk(chain, q);
    float goal_pose_array[6] = {};
    float achieved_pose_array[6] = {};
    transform_to_pose(control_goal, goal_pose_array);
    transform_to_pose(achieved, achieved_pose_array);
    const float dy = goal_pose_array[1] - achieved_pose_array[1];
    const float dz = goal_pose_array[2] - achieved_pose_array[2];
    const float dpitch =
        arm::angle_distance(achieved_pose_array[4], goal_pose_array[4]);
    const float task_position_error = std::sqrt(dy * dy + dz * dz);
    const float task_orientation_error = std::fabs(dpitch);
    const float final_task_error =
        std::sqrt(task_position_error * task_position_error +
                  task_orientation_error * task_orientation_error);
    const bool reached =
        task_position_error <=
            ((input.position_tolerance > 0.0f) ? input.position_tolerance
                                               : 1.0e-3f) &&
        task_orientation_error <=
            ((input.orientation_tolerance > 0.0f) ? input.orientation_tolerance
                                                  : 1.0e-2f);

    std::fprintf(file,
                 "%.9g,%d,%d,%d,%d,%u,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
                 t,
                 (ik_ok && safety_ok) ? 1 : 0,
                 reached ? 1 : 0,
                 ik_ok ? 1 : 0,
                 safety_ok ? 1 : 0,
                 static_cast<unsigned>(dls_result.status),
                 static_cast<unsigned>(dls_result.status),
                 static_cast<unsigned>(safety.reason),
                 1.0f,
                 final_task_error,
                 task_position_error,
                 task_orientation_error,
                 dls_result.singularity_metric,
                 velocity_scale,
                 joint_step_norm);
    for (int i = 0; i < kDof; ++i) std::fprintf(file, ",%.9g", q[i][0]);
    for (int i = 0; i < kDof; ++i) std::fprintf(file, ",%.9g", qd_command[i][0]);
    for (float value : commanded_pose_array) std::fprintf(file, ",%.9g", value);
    for (float value : achieved_pose_array) std::fprintf(file, ",%.9g", value);
    std::fprintf(file, "\n");

    if (reached && t >= planned_duration) {
      break;
    }
  }

  std::fclose(file);
  return true;
}

int main(int argc, char** argv) {
  if (argc != 7) {
    std::fprintf(stderr,
                 "usage: %s input.csv movej.csv cartesian.csv frame_targets.csv ik.csv servo.csv\n",
                 argv[0]);
    return 2;
  }

  InputData input = {};
  if (!read_input(argv[1], &input)) {
    std::fprintf(stderr, "failed to read input: %s\n", argv[1]);
    return 3;
  }

  const arm::SerialChain<kDof> chain = make_3pit_chain();
  const arm::JointVec<kDof> q0 = make_joint_vec<kDof>(input.q0);
  const arm::Transform current_base_tcp = arm::kinematics::fk(chain, q0);
  const arm::Transform absolute_goal = pose_to_transform(input.cart_goal_pose);
  const arm::Transform control_goal = select_control_goal(input, current_base_tcp);

  if (!export_movej(input, argv[2])) {
    std::fprintf(stderr, "failed to export 3pit MoveJ trajectory\n");
    return 4;
  }
  if (!export_cartesian(input, current_base_tcp, control_goal, argv[3])) {
    std::fprintf(stderr, "failed to export 3pit Cartesian trajectory\n");
    return 5;
  }
  if (!export_frame_targets(input, current_base_tcp, absolute_goal, argv[4])) {
    std::fprintf(stderr, "failed to export 3pit frame targets\n");
    return 6;
  }
  if (!export_cpp_ik(input, current_base_tcp, absolute_goal, argv[5])) {
    std::fprintf(stderr, "failed to export 3pit C++ IK results\n");
    return 7;
  }
  if (!export_control_loop(input, control_goal, argv[6])) {
    std::fprintf(stderr, "failed to export 3pit control loop\n");
    return 8;
  }
  return 0;
}
