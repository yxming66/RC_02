#include "pose_error.h"

#include "../adapter/toolbox_adapter.h"

namespace arm_lib {
namespace kinematics {

Vec3 position_error(const Transform& target, const Transform& current) {
  return position_error(target, current, PoseErrorFrame::kSpatial);
}

Vec3 position_error(const Transform& target, const Transform& current,
                    PoseErrorFrame frame) {
  if (frame == PoseErrorFrame::kBody) {
    const Transform delta =
        toolbox_adapter::inverse_transform(current) * target;
    const Twist6 twist = toolbox_adapter::twist_from_transform(delta);
    return twist.block<3, 1>(0, 0);
  }

  return toolbox_adapter::translation_of(target) -
         toolbox_adapter::translation_of(current);
}

Vec3 orientation_error(const Transform& target, const Transform& current) {
  return orientation_error(target, current, PoseErrorFrame::kSpatial);
}

Vec3 orientation_error(const Transform& target, const Transform& current,
                       PoseErrorFrame frame) {
  const Transform delta =
      (frame == PoseErrorFrame::kBody)
          ? (toolbox_adapter::inverse_transform(current) * target)
          : (target * toolbox_adapter::inverse_transform(current));
  const Twist6 twist = toolbox_adapter::twist_from_transform(delta);
  return twist.block<3, 1>(3, 0);
}

Twist6 pose_error_twist(const Transform& target, const Transform& current) {
  return pose_error_twist(target, current, PoseErrorFrame::kSpatial);
}

Twist6 pose_error_twist(const Transform& target, const Transform& current,
                        PoseErrorFrame frame) {
  Twist6 error = matrixf::zeros<6, 1>();
  const Vec3 dp = position_error(target, current, frame);
  const Vec3 dw = orientation_error(target, current, frame);

  for (uint16_t i = 0; i < 3; ++i) {
    error[i][0] = dp[i][0];
    error[i + 3][0] = dw[i][0];
  }

  return error;
}

PoseError6 evaluate_pose_error(const Transform& target, const Transform& current) {
  return evaluate_pose_error(target, current, PoseErrorFrame::kSpatial);
}

PoseError6 evaluate_pose_error(const Transform& target, const Transform& current,
                               PoseErrorFrame frame) {
  PoseError6 error;
  error.twist = pose_error_twist(target, current, frame);
  error.position_norm = position_error(target, current, frame).norm();
  error.orientation_norm = orientation_error(target, current, frame).norm();
  return error;
}

}  // namespace kinematics
}  // namespace arm_lib
