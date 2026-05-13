#include "ik_6r_pieper.h"

#include <cmath>

#include "../../core/arm_common.h"
#include "../../math/linear_solve.h"
#include "../fk.h"
#include "../pose_error.h"

namespace mr::robotics::arm {
namespace kinematics {
namespace analytic {
namespace {

static constexpr Scalar kPi = 3.14159265358979323846f;
static constexpr Scalar kWristIntersectionTolerance = 5.0e-3f;
static constexpr Scalar kWristCenterTolerance = 2.0e-4f;
static constexpr Scalar kWristOrientationTolerance = 2.0e-4f;
static constexpr Scalar kDamping = 1.0e-4f;
static constexpr Scalar kMaxPositionStep = 0.25f;
static constexpr Scalar kMaxOrientationStep = 0.35f;

struct AxisLine {
  Vec3 origin;
  Vec3 axis;

  AxisLine()
      : origin(toolbox_adapter::zero_vec3()),
        axis(toolbox_adapter::zero_vec3()) {}
};

struct PieperWristModel {
  Vec3 wrist_center_frame3;
  Vec3 tool_offset_tcp;
  Scalar intersection_error;

  PieperWristModel()
      : wrist_center_frame3(toolbox_adapter::zero_vec3()),
        tool_offset_tcp(toolbox_adapter::zero_vec3()),
        intersection_error(0.0f) {}
};

inline Scalar dot3(const Vec3& lhs, const Vec3& rhs) {
  return lhs[0][0] * rhs[0][0] + lhs[1][0] * rhs[1][0] +
         lhs[2][0] * rhs[2][0];
}

inline Vec3 transform_point(const Transform& transform, const Vec3& point) {
  Vec3 out = toolbox_adapter::zero_vec3();
  for (uint16_t row = 0; row < 3U; ++row) {
    out[row][0] = transform[row][3];
    for (uint16_t col = 0; col < 3U; ++col) {
      out[row][0] += transform[row][col] * point[col][0];
    }
  }
  return out;
}

inline void clamp_joint_to_limits(const SerialChain<6>& chain,
                                  uint16_t index,
                                  JointVec<6>* q) {
  if (q == nullptr || index >= 6U) {
    return;
  }
  const Link& link = chain.link(index);
  if (link.limit_enabled()) {
    (*q)[index][0] = clamp_scalar((*q)[index][0], link.lower_limit(),
                                  link.upper_limit());
  } else if (link.joint_type() == ChainJointType::kRevolute) {
    (*q)[index][0] = wrap_to_pi((*q)[index][0]);
  }
}

AxisLine joint_axis_line(const SerialChain<6>& chain,
                         const JointVec<6>& q,
                         uint16_t index) {
  AxisLine line;
  Transform prefix = chain.base_frame();
  for (uint16_t i = 0; i < index && i < 6U; ++i) {
    prefix = prefix * chain.link(i).transform(q[i][0]);
  }

  const Link& link = chain.link(index);
  const Transform joint_frame = prefix * link.joint_origin_transform();
  line.origin = toolbox_adapter::translation_of(joint_frame);
  line.axis =
      toolbox_adapter::rotation_of(joint_frame) * link.joint_axis_local();
  const Scalar norm = line.axis.norm();
  if (norm > ARM_LIB_EPSILON) {
    line.axis /= norm;
  }
  return line;
}

Scalar line_distance(const AxisLine& a, const AxisLine& b) {
  const Vec3 normal = vector3f::cross(a.axis, b.axis);
  const Scalar normal_norm = normal.norm();
  const Vec3 delta = b.origin - a.origin;
  if (normal_norm <= ARM_LIB_EPSILON) {
    return vector3f::cross(delta, a.axis).norm();
  }
  return abs_scalar(dot3(delta, normal) / normal_norm);
}

bool closest_line_midpoint(const AxisLine& a,
                           const AxisLine& b,
                           Vec3* midpoint) {
  if (midpoint == nullptr) {
    return false;
  }

  const Vec3 r = a.origin - b.origin;
  const Scalar aa = dot3(a.axis, a.axis);
  const Scalar bb = dot3(b.axis, b.axis);
  const Scalar ab = dot3(a.axis, b.axis);
  const Scalar ar = dot3(a.axis, r);
  const Scalar br = dot3(b.axis, r);
  const Scalar denominator = aa * bb - ab * ab;
  if (abs_scalar(denominator) <= ARM_LIB_EPSILON) {
    return false;
  }

  const Scalar s = (ab * br - bb * ar) / denominator;
  const Scalar t = (aa * br - ab * ar) / denominator;
  const Vec3 pa = a.origin + a.axis * s;
  const Vec3 pb = b.origin + b.axis * t;
  *midpoint = (pa + pb) * 0.5f;
  return true;
}

bool compute_pieper_wrist_model(const SerialChain<6>& chain,
                                PieperWristModel* model) {
  if (model == nullptr || !chain.validate()) {
    return false;
  }

  for (uint16_t i = 0; i < 6U; ++i) {
    const Link& link = chain.link(i);
    if (link.joint_type() != ChainJointType::kRevolute ||
        !link.participates_in_ik()) {
      return false;
    }
  }

  const JointVec<6> q_zero = chain.zero_configuration();
  const AxisLine wrist_axes[3] = {
      joint_axis_line(chain, q_zero, 3U),
      joint_axis_line(chain, q_zero, 4U),
      joint_axis_line(chain, q_zero, 5U),
  };

  const Scalar d45 = line_distance(wrist_axes[0], wrist_axes[1]);
  const Scalar d56 = line_distance(wrist_axes[1], wrist_axes[2]);
  const Scalar d46 = line_distance(wrist_axes[0], wrist_axes[2]);
  const Scalar max_error = fmaxf(d45, fmaxf(d56, d46));
  if (max_error > kWristIntersectionTolerance) {
    return false;
  }

  Vec3 center_sum = toolbox_adapter::zero_vec3();
  uint16_t center_count = 0U;
  Vec3 midpoint = toolbox_adapter::zero_vec3();
  if (closest_line_midpoint(wrist_axes[0], wrist_axes[1], &midpoint)) {
    center_sum += midpoint;
    ++center_count;
  }
  if (closest_line_midpoint(wrist_axes[1], wrist_axes[2], &midpoint)) {
    center_sum += midpoint;
    ++center_count;
  }
  if (closest_line_midpoint(wrist_axes[0], wrist_axes[2], &midpoint)) {
    center_sum += midpoint;
    ++center_count;
  }
  if (center_count == 0U) {
    return false;
  }

  const Vec3 wrist_center_base = center_sum / static_cast<Scalar>(center_count);
  const Transform t03_zero = fk_prefix(chain, q_zero, 3U);
  model->wrist_center_frame3 =
      transform_point(toolbox_adapter::inverse_transform(t03_zero),
                      wrist_center_base);

  const Transform tcp_zero = fk(chain, q_zero);
  const Rotation r_tcp_zero = toolbox_adapter::rotation_of(tcp_zero);
  const Vec3 p_tcp_zero = toolbox_adapter::translation_of(tcp_zero);
  model->tool_offset_tcp = r_tcp_zero.trans() * (p_tcp_zero - wrist_center_base);
  model->intersection_error = max_error;
  return true;
}

Vec3 wrist_center_position(const SerialChain<6>& chain,
                           const PieperWristModel& model,
                           const JointVec<6>& q) {
  return transform_point(fk_prefix(chain, q, 3U), model.wrist_center_frame3);
}

Matrixf<3, 3> wrist_center_jacobian(const SerialChain<6>& chain,
                                    const PieperWristModel& model,
                                    const JointVec<6>& q) {
  Matrixf<3, 3> jac = matrixf::zeros<3, 3>();
  const Vec3 wrist_center = wrist_center_position(chain, model, q);
  for (uint16_t i = 0; i < 3U; ++i) {
    const AxisLine line = joint_axis_line(chain, q, i);
    const Vec3 linear = vector3f::cross(line.axis, wrist_center - line.origin);
    jac[0][i] = linear[0][0];
    jac[1][i] = linear[1][0];
    jac[2][i] = linear[2][0];
  }
  return jac;
}

bool solve_wrist_center(const SerialChain<6>& chain,
                        const PieperWristModel& model,
                        const Vec3& target_wrist_center,
                        JointVec<6>* q) {
  if (q == nullptr) {
    return false;
  }

  for (uint16_t iter = 0; iter < 48U; ++iter) {
    const Vec3 current = wrist_center_position(chain, model, *q);
    const Vec3 error = target_wrist_center - current;
    if (error.norm() <= kWristCenterTolerance) {
      return true;
    }

    const Matrixf<3, 3> jac = wrist_center_jacobian(chain, model, *q);
    const Matrixf<3, 3> normal =
        jac.trans() * jac + kDamping * matrixf::eye<3, 3>();
    Matrixf<3, 1> step = matrixf::zeros<3, 1>();
    if (!math::solve_symmetric_ldlt(normal, jac.trans() * error, &step)) {
      return false;
    }
    const Scalar step_norm = step.norm();
    if (step_norm > kMaxPositionStep) {
      step *= (kMaxPositionStep / step_norm);
    }

    for (uint16_t i = 0; i < 3U; ++i) {
      (*q)[i][0] += step[i][0];
      clamp_joint_to_limits(chain, i, q);
    }
  }

  return (target_wrist_center - wrist_center_position(chain, model, *q)).norm() <=
         5.0f * kWristCenterTolerance;
}

Matrixf<3, 3> wrist_orientation_jacobian(const SerialChain<6>& chain,
                                         const JointVec<6>& q) {
  Matrixf<3, 3> jac = matrixf::zeros<3, 3>();
  for (uint16_t i = 0; i < 3U; ++i) {
    const AxisLine line = joint_axis_line(chain, q, i + 3U);
    jac[0][i] = line.axis[0][0];
    jac[1][i] = line.axis[1][0];
    jac[2][i] = line.axis[2][0];
  }
  return jac;
}

bool solve_wrist_orientation(const SerialChain<6>& chain,
                             const Transform& target,
                             JointVec<6>* q) {
  if (q == nullptr) {
    return false;
  }

  for (uint16_t iter = 0; iter < 56U; ++iter) {
    const PoseError6 error = evaluate_pose_error(target, fk(chain, *q));
    if (error.orientation_norm <= kWristOrientationTolerance) {
      return true;
    }

    Matrixf<3, 1> angular = matrixf::zeros<3, 1>();
    angular[0][0] = error.twist[3][0];
    angular[1][0] = error.twist[4][0];
    angular[2][0] = error.twist[5][0];

    const Matrixf<3, 3> jac = wrist_orientation_jacobian(chain, *q);
    const Matrixf<3, 3> normal =
        jac.trans() * jac + kDamping * matrixf::eye<3, 3>();
    Matrixf<3, 1> step = matrixf::zeros<3, 1>();
    if (!math::solve_symmetric_ldlt(normal, jac.trans() * angular, &step)) {
      return false;
    }
    const Scalar step_norm = step.norm();
    if (step_norm > kMaxOrientationStep) {
      step *= (kMaxOrientationStep / step_norm);
    }

    for (uint16_t i = 0; i < 3U; ++i) {
      const uint16_t joint = i + 3U;
      (*q)[joint][0] += step[i][0];
      clamp_joint_to_limits(chain, joint, q);
    }
  }

  return evaluate_pose_error(target, fk(chain, *q)).orientation_norm <=
         5.0f * kWristOrientationTolerance;
}

bool add_unique_solution(const SerialChain<6>& chain,
                         const JointVec<6>& q,
                         AnalyticIKSolutionSet<6>* out) {
  if (out == nullptr || out->count >= ARM_LIB_ANALYTIC_IK_MAX_SOLUTIONS) {
    return false;
  }

  for (uint16_t i = 0; i < out->count; ++i) {
    if (solver::configuration_distance(chain, out->q[i], q) <= 1.0e-4f) {
      return false;
    }
  }

  out->q[out->count] = q;
  ++out->count;
  return true;
}

JointVec<6> base_seed_for_request(const SerialChain<6>& chain,
                                  const IKRequest<6>& request) {
  if (request.use_seed) {
    return request.seed;
  }
  if (request.use_reference) {
    return request.reference;
  }
  return chain.zero_configuration();
}

}  // namespace

bool is_pieper_6r_compatible(const SerialChain<6>& chain) {
  PieperWristModel model;
  return compute_pieper_wrist_model(chain, &model);
}

bool generate_pieper_6r_solutions(const SerialChain<6>& chain,
                                  const IKRequest<6>& request,
                                  AnalyticIKSolutionSet<6>* out) {
  if (out == nullptr) {
    return false;
  }
  out->count = 0U;

  PieperWristModel model;
  if (!compute_pieper_wrist_model(chain, &model)) {
    return false;
  }

  const Rotation target_rotation = toolbox_adapter::rotation_of(request.target);
  const Vec3 target_position = toolbox_adapter::translation_of(request.target);
  const Vec3 target_wrist_center =
      target_position - target_rotation * model.tool_offset_tcp;

  const JointVec<6> base_seed = base_seed_for_request(chain, request);
  JointVec<6> seeds[4];
  seeds[0] = base_seed;
  seeds[1] = chain.zero_configuration();
  seeds[2] = base_seed;
  seeds[2][1][0] += 0.75f;
  seeds[2][2][0] -= 0.75f;
  seeds[3] = base_seed;
  seeds[3][1][0] -= 0.75f;
  seeds[3][2][0] += 0.75f;

  for (uint16_t seed_index = 0; seed_index < 4U; ++seed_index) {
    JointVec<6> q_position = seeds[seed_index];
    for (uint16_t i = 0; i < 3U; ++i) {
      clamp_joint_to_limits(chain, i, &q_position);
    }

    if (!solve_wrist_center(chain, model, target_wrist_center, &q_position)) {
      continue;
    }

    JointVec<6> wrist_seeds[2];
    wrist_seeds[0] = q_position;
    wrist_seeds[1] = q_position;
    wrist_seeds[1][3][0] += kPi;
    wrist_seeds[1][4][0] = -wrist_seeds[1][4][0];
    wrist_seeds[1][5][0] += kPi;

    for (uint16_t wrist_seed = 0; wrist_seed < 2U; ++wrist_seed) {
      JointVec<6> q = wrist_seeds[wrist_seed];
      for (uint16_t i = 3U; i < 6U; ++i) {
        clamp_joint_to_limits(chain, i, &q);
      }

      if (!solve_wrist_orientation(chain, request.target, &q)) {
        continue;
      }

      add_unique_solution(chain, q, out);
      if (out->count >= ARM_LIB_ANALYTIC_IK_MAX_SOLUTIONS) {
        return true;
      }
    }
  }

  return out->count > 0U;
}

}  // namespace analytic
}  // namespace kinematics
}  // namespace mr::robotics::arm
