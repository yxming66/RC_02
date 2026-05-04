#include "ik_3r_planar.h"

#include <cmath>

namespace mr::robotics::arm {
namespace kinematics {
namespace analytic {
namespace {

bool transform_is_planar(const Transform& transform, Scalar tolerance) {
  const Vec3 translation = toolbox_adapter::translation_of(transform);
  if (abs_scalar(translation[2][0]) > tolerance) {
    return false;
  }

  const Vec3 ypr =
      toolbox_adapter::rpy_from_rotation(toolbox_adapter::rotation_of(transform));
  return (abs_scalar(ypr[1][0]) <= tolerance) &&
         (abs_scalar(ypr[2][0]) <= tolerance);
}

}  // namespace

bool is_planar_3r_compatible(const SerialChain<3>& chain) {
  if (!transform_is_planar(chain.base_frame(), 1.0e-4f) ||
      !transform_is_planar(chain.tool_frame().transform(), 1.0e-4f)) {
    return false;
  }

  for (uint16_t i = 0; i < 3; ++i) {
    const Link& link = chain.link(i);
    if (link.kinematics_model() != LinkKinematics::kDh) {
      return false;
    }
    if (link.joint_type() != ChainJointType::kRevolute) {
      return false;
    }
    if (link.convention() != DHConvention::kStandard) {
      return false;
    }
    if (abs_scalar(link.dh().d) > 1.0e-5f || abs_scalar(link.dh().alpha) > 1.0e-5f) {
      return false;
    }
  }
  return true;
}

bool generate_planar_3r_solutions(const SerialChain<3>& chain,
                                  const IKRequest<3>& request,
                                  AnalyticIKSolutionSet<3>* out) {
  if (out == nullptr || !is_planar_3r_compatible(chain)) {
    return false;
  }

  out->count = 0U;

  const Transform target_local =
      toolbox_adapter::inverse_transform(chain.base_frame()) * request.target *
      toolbox_adapter::inverse_transform(chain.tool_frame().transform());
  if (!transform_is_planar(target_local, 1.0e-4f)) {
    return false;
  }

  const Vec3 p = toolbox_adapter::translation_of(target_local);
  const Vec3 ypr =
      toolbox_adapter::rpy_from_rotation(toolbox_adapter::rotation_of(target_local));
  const Scalar x = p[0][0];
  const Scalar y = p[1][0];
  const Scalar phi = ypr[0][0];

  const Scalar a1 = chain.link(0).dh().a;
  const Scalar a2 = chain.link(1).dh().a;
  const Scalar a3 = chain.link(2).dh().a;
  const Scalar denominator = 2.0f * a1 * a2;
  if (abs_scalar(denominator) <= ARM_LIB_EPSILON) {
    return false;
  }

  const Scalar xw = x - a3 * cosf(phi);
  const Scalar yw = y - a3 * sinf(phi);
  Scalar c2 =
      ((xw * xw) + (yw * yw) - (a1 * a1) - (a2 * a2)) / denominator;
  if (c2 < -1.0f - 1.0e-5f || c2 > 1.0f + 1.0e-5f) {
    return false;
  }
  c2 = clamp_scalar(c2, -1.0f, 1.0f);

  const Scalar s2_abs = sqrtf(clamp_scalar(1.0f - c2 * c2, 0.0f, 1.0f));
  const Scalar s2_candidates[2] = {s2_abs, -s2_abs};

  for (uint16_t i = 0; i < 2U; ++i) {
    if (out->count >= ARM_LIB_ANALYTIC_IK_MAX_SOLUTIONS) {
      break;
    }

    const Scalar s2 = s2_candidates[i];
    const Scalar t2 = atan2f(s2, c2);
    const Scalar t1 =
        atan2f(yw, xw) - atan2f(a2 * s2, a1 + a2 * c2);
    const Scalar t3 = phi - t1 - t2;

    JointVec<3> q = toolbox_adapter::zero_joint_vec<3>();
    q[0][0] = wrap_to_pi(t1 - chain.link(0).dh().theta -
                         chain.link(0).joint_offset());
    q[1][0] = wrap_to_pi(t2 - chain.link(1).dh().theta -
                         chain.link(1).joint_offset());
    q[2][0] = wrap_to_pi(t3 - chain.link(2).dh().theta -
                         chain.link(2).joint_offset());

    bool duplicate = false;
    for (uint16_t existing = 0; existing < out->count; ++existing) {
      if (solver::configuration_distance(chain, out->q[existing], q) <= 1.0e-5f) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      out->q[out->count] = q;
      ++out->count;
    }
  }

  return out->count > 0U;
}

}  // namespace analytic
}  // namespace kinematics
}  // namespace mr::robotics::arm
