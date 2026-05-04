#ifndef ARM_LIB_KINEMATICS_POSE_ERROR_H
#define ARM_LIB_KINEMATICS_POSE_ERROR_H

#include "../core/arm_status.h"

namespace mr::robotics::arm {
namespace kinematics {

enum class PoseErrorFrame : uint8_t {
  kSpatial = 0,
  kBody = 1,
};

struct PoseError6 {
  Twist6 twist;
  Scalar position_norm;
  Scalar orientation_norm;

  PoseError6()
      : twist(matrixf::zeros<6, 1>()),
        position_norm(0.0f),
        orientation_norm(0.0f) {}

  Scalar total_norm() const { return twist.norm(); }
};

Vec3 position_error(const Transform& target, const Transform& current);
Vec3 position_error(const Transform& target, const Transform& current,
                    PoseErrorFrame frame);
Vec3 orientation_error(const Transform& target, const Transform& current);
Vec3 orientation_error(const Transform& target, const Transform& current,
                       PoseErrorFrame frame);
Twist6 pose_error_twist(const Transform& target, const Transform& current);
Twist6 pose_error_twist(const Transform& target, const Transform& current,
                        PoseErrorFrame frame);
PoseError6 evaluate_pose_error(const Transform& target, const Transform& current);
PoseError6 evaluate_pose_error(const Transform& target, const Transform& current,
                               PoseErrorFrame frame);

}  // namespace kinematics
}  // namespace mr::robotics::arm

#endif
